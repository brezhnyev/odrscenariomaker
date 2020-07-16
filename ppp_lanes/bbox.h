#pragma once

#include "point.h"

#include <eigen3/Eigen/Eigen>

#include <deque>
#include <algorithm>
#include <numeric>

#define MAXVAL 1000000000
struct BBox
{
    void addPoint(float v [3])
    {
        if (v[0] < minp[0]) minp[0] = v[0];
        if (v[1] < minp[1]) minp[1] = v[1];
        if (v[2] < minp[2]) minp[2] = v[2];
        if (v[0] > maxp[0]) maxp[0] = v[0];
        if (v[1] > maxp[1]) maxp[1] = v[1];
        if (v[2] > maxp[2]) maxp[2] = v[2];
    }
    bool crossing(const BBox & other)
    {
        if (minp[0] > other.maxp[0] || other.minp[0] > maxp[0]) return false;
        if (minp[1] > other.maxp[1] || other.minp[1] > maxp[1]) return false;
        if (minp[2] > other.maxp[2] || other.minp[2] > maxp[2]) return false;

        return true;
    }
    void clear()
    {
        minp = Eigen::Vector3f( MAXVAL, MAXVAL, MAXVAL);
        maxp = Eigen::Vector3f(-MAXVAL,-MAXVAL,-MAXVAL);
    }
    Eigen::Vector3f center()
    {
        return 0.5f*(minp + maxp);
    }
    Eigen::Vector3f minp = Eigen::Vector3f{ MAXVAL,  MAXVAL,  MAXVAL};
    Eigen::Vector3f maxp = Eigen::Vector3f{-MAXVAL, -MAXVAL, -MAXVAL};
};


struct BBoxPC : public std::deque<Point>
{
    BBoxPC() = default;
    BBoxPC(const BBoxPC & b) = default;
    BBoxPC & operator = (const BBoxPC & b) = default;
    ~BBoxPC() = default;

    BBoxPC(BBoxPC && b)
    {
        *this = move(b);
    }
    BBoxPC & operator = (BBoxPC && b)
    {
        this->std::deque<Point>::operator=(move(b));
        bbox = b.bbox;
        b.bbox.clear();
        return *this;
    }

    void push_back(Point p)
    {
        std::deque<Point>::push_back(p);
        bbox.addPoint(p.v);
    }
    void clear()
    {
        std::deque<Point>::clear();
        bbox.clear();
    }
    BBox bbox;

    static void removeOutliers(BBoxPC & pc)
    {
        using namespace Eigen;

        Vector3f center(0,0,0);
        for (auto && p : pc)
            center += Vector3f(p.v[0], p.v[1], p.v[2]);
        center /= pc.size();
        float R = 0;
        for (auto && p : pc)
            R += (Vector3f(p.v[0], p.v[1], p.v[2]) - center).norm();
        R /= pc.size();
        auto newpc = pc; newpc.clear(); newpc.bbox.clear();
        for (auto && p : pc)
            if ((Vector3f(p.v[0], p.v[1], p.v[2]) - center).norm() < 5*R)
                newpc.push_back(p);

        pc = newpc;
    }

    // static void removeOutliers(BBoxPC & pc)
    // {
    //     using namespace Eigen;

    //     if (pc.size() < 3) return;

    //     deque<Vector3f> vv;
    //     for (auto && p : pc) vv.push_back(Vector3f(p.v));
    //     Vector3f center(0,0,0);
    //     for (auto && v : vv) center += v; center /= vv.size();

    //     auto res = getPCEigenvalues<decltype(vv), Vector3f, Matrix3f>(vv);
    //     // eigenvalues are variances! Get standard deviation:
    //     float devX = sqrt(res.first[0]);
    //     float devY = sqrt(res.first[1]);

    //     BBoxPC newpc;

    //     for (int i = 0; i < vv.size(); ++i)
    //     {
    //         Vector3f v = res.second*(vv[i] - center);
    //         // check only x and y:
    //         if ( (abs(v[0]) < 3*devX) && (abs(v[1]) < 3*devY) )
    //             newpc.push_back(pc[i]);
    //     }

    //     pc = move(newpc);
    // }
};

template <typename ContainerT, typename VectorT, typename MatrixT>
std::pair<VectorT, MatrixT> getPCEigenvalues(const ContainerT & values, bool doSort = false)
{
    using namespace std;
    using namespace Eigen;

    typedef typename VectorT::value_type T; // standard type (float, double ...)

    const int DIM = VectorT::RowsAtCompileTime; // must be implemented in the customer's class!

    VectorT retV; // eigenvalues
    MatrixT retM; // eigenvectors

    if (values.empty()) return pair<VectorT, MatrixT>(retV, retM); // zero values

    // make structure of arrays out of array of structures
    // this will make more efficient multiplication row x col to find covariance matrix

    // values (size n+1 x 3):       components (size 3 x n+1):

    //  v0 v1 v2     vn             x| x0 x1 x2 ... xn
    // |x0|x1|x2|...|xn|              ---------------
    // |y0|y1|y2|...|yn|     ->     y| y0 y1 y2 ... yn
    // |z0|z1|z2|...|zn|              ---------------
    //                              z| z0 z1 z2 ... zn

    vector<vector<T>> components; components.resize(DIM);
    for (int cc = 0; cc < DIM; ++cc) components[cc].reserve(values.size());

    // find the center of the neighbours PC:
    VectorT center; // unfortunately Eigen does not initialize with zeros, we must fill out to be sure:
    for (int cc = 0; cc < DIM; ++cc) center[cc] = 0;
    for (const VectorT & v : values) center += v; center /= (double)values.size();
    // fill out the X,Y and Z:
    for (size_t vc = 0; vc < values.size(); ++vc) // vc == vector or value counter
        for (int cc = 0; cc < DIM; ++cc)
            components[cc].emplace_back(values[vc][cc] - center[cc]);

    // fill out co-variance matrix:
    Eigen::Matrix<T, DIM, DIM> m;
    for (int cc1 = 0; cc1 < DIM; ++cc1)
        for (int cc2 = 0; cc2 < DIM; ++cc2)
            m(cc1, cc2) = (T)inner_product(components[cc1].begin(), components[cc1].end(), components[cc2].begin(), T())/components[cc1].size();

    // use the Eigen solver to find out the eigen-values and eigen-vectors:
    Eigen::EigenSolver<decltype(m)> es(m);
    auto evec = es.eigenvectors();
    auto eval = es.eigenvalues();

    // sometimes sorting is needed (ex finding out normals of point cloud)
    if (doSort) // was not tested after code modification
    {
        // first sort the columns in the ascending order by corresponding eigenvalue:
        map<T, VectorT> val2vec;
        for (int cc1 = 0; cc1 < DIM; ++cc1)
        {
            for (int cc2 = 0; cc2 < DIM; ++cc2)
                val2vec[(T)eval(cc1, 0).real()][cc2] = (T)evec(cc2, cc1).real();
        }
        // then fill out the output:
        int vc = 0; // eigen vectors/values counter
        for (auto it = val2vec.begin(); it != val2vec.end(); ++it, ++vc)
        {
            retV[vc] = it->first;
            retM.block(0,vc,3,1) = it->second;
        }
    }
    else
    {
        retV = eval.real();
        retM = evec.real();
    }

    return std::pair<VectorT, MatrixT>(retV, retM);
}

