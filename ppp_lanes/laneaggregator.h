#include "pathmerger.h"

#define MINLANESZ 16 // minimum 15 points making lane


class LaneAggregator : public PathMerger
{
public:
    LaneAggregator(float _cS) : PathMerger(_cS) {}

public:

    void process(BBoxPC & container, std::map<int, BBoxPC> & lanesmap)
    {
        PathMerger::process(container);

        using namespace std;
        using namespace Eigen;

        lanesmap.clear();

        if (lanes.empty())
        {
            lanes = paths;
            direction.setZero();
            bbox = container.bbox;
            container.clear();
            return;
        }
        else if (direction.isZero())
        {
            direction = container.bbox.center() - bbox.center();
            bbox = container.bbox;
            for (auto && lane : lanes)
            {
                if ((Vector3f(lane.back().v) - Vector3f(lane.front().v)).dot(direction) < 0)
                    reverse(lane.begin(), lane.end());
            }
        }
        else
        {
            direction = container.bbox.center() - bbox.center();
            bbox = container.bbox;
        }

        for (auto &&path : paths)
        {
            if ((Vector3f(path.back().v) - Vector3f(path.front().v)).dot(direction) < 0)
                reverse(path.begin(), path.end());
        }

        auto isOverlapping = [&](auto P, auto && it, auto itend) -> bool
        {
            int col = (P[0] - container.bbox.minp[0]) / cS;
            int row = (P[1] - container.bbox.minp[1]) / cS;
            for (; it != itend; ++it)
            {
                Point p = (*it);
                int pcol = (p[0] - container.bbox.minp[0]) / cS;
                int prow = (p[1] - container.bbox.minp[1]) / cS;
                if (abs(pcol - col) < 2 && abs(prow - row) < 2)
                {
                    return true;
                }
            }
            return false;
        };

        auto extendPath = [&](auto P, auto it, auto itend, auto & path, bool pushfront) -> bool
        {
            if (isOverlapping(P, it, itend))
            {
                if (pushfront)
                    for (; it != itend; ++it) path.push_front(*it);
                else
                    for (; it != itend; ++it) path.push_back(*it);

                return true;
            }
            return false;
        };

        for (auto &&lane : lanes)
        {
            bool isOverlap = false;
            // WE ARE CHANGING THE PATHS!!!
            for (auto &&path : paths)
            {
                // front extension of path:
                isOverlap = extendPath(path.front(), lane.rbegin(), lane.rend(), path, true);
                // back extension of path:
                isOverlap = extendPath(path.back(), lane.begin(), lane.end(), path, false) || isOverlap;

                // if lane is part of path then the lane will be removed
                if (!isOverlap)
                    isOverlap = isOverlapping(lane.front(), path.rbegin(), path.rend());
                if (!isOverlap)
                    isOverlap = isOverlapping(lane.back(), path.begin(), path.end()) || isOverlap;

                if (isOverlap)
                {
                    lane.clear();
                    break;
                }
            }
            if (!isOverlap)
            {
                if (lane.size() > MINLANESZ) // filter out too short lanes
                    lanesmap[laneID++] = move(lane);
                else
                    lane.clear();
            }
        }

        // get rid of all empty lanes:
        auto lanescp = move(lanes);
        for (auto && lane : lanescp) if (!lane.empty()) lanes.push_back(lane);
        // rest of the non-overlapping paths:
        for (auto && path : paths) lanes.push_front(move(path));

        // PATHS ARE EMPTY at this point !!!!!!!!!

        getLanes(container, lanesmap);
    }

public:

    void getLanes(BBoxPC & container, std::map<int, BBoxPC> & lanesmap, bool isFinal = false)
    {
        using namespace Eigen;

        container.clear();
        if (isFinal)
            for (auto && lane : lanes) if (lane.size() > MINLANESZ) lanesmap[laneID++] = move(lane);

        if (lanesmap.empty()) return;

        // Fillout the container
        int li = 0;
        for (auto && it : lanesmap)
        {
            auto && lane = it.second;

            resample(lane);
            // After resampling the lane may lose all points, i.e. become empty, discard it
            if (lane.empty()) continue;

            for (int i = 0; i < lane.size(); ++i)
            {
                if (i != 0)
                    memcpy(lane[i].color, &colors[li][0], 3*sizeof(unsigned char)); 
                else 
                {
                    unsigned char RGB[3] = {255,255,255}; // This will visually mark the start of the path
                    memcpy(lane[i].color, RGB, 3*sizeof(unsigned char));
                }
                container.push_back(lane[i]);
            }
            ++li;
        }
    }

protected:

    virtual void resample(BBoxPC & lane)
    {
        using namespace Eigen;

        // Remember the start point of lane:
        auto st = lane.front();
        // resample with a new step size (ex. 1 meter), we set the closing holes a bit larger than before due to new (larger) quantizing
        PathMerger pf(1.0f, 10.0f); pf.process(lane);
        if (lane.empty()) return;
        // The lane may be now reversed, check this:
        if ((Vector3f(lane.front().v) - Vector3f(st.v)).norm() > (Vector3f(lane.back().v) - Vector3f(st.v)).norm())
            reverse(lane.begin(), lane.end());
    }

private:
    BBox bbox;
    Eigen::Vector3f direction;
    std::deque<BBoxPC> lanes;
    static int laneID;
};



class LaneAggregatorCont : public LaneAggregator
{
public:
    LaneAggregatorCont(float _cS) : LaneAggregator(_cS) {}
};

class LaneAggregatorCurb : public LaneAggregator
{
public:
    LaneAggregatorCurb(float _cS) : LaneAggregator(_cS) {}
};

class LaneAggregatorDash : public LaneAggregator
{
public:
    LaneAggregatorDash(float _cS) : LaneAggregator(_cS) {}
};


class LaneAggregatorStop : public LaneAggregator
{
public:
    LaneAggregatorStop(float _cS) : LaneAggregator(_cS) {}

    void resample(BBoxPC & lane) override
    {
        // The stop line is short and usually dense PC. Get the PCA thereof to get its main axis and resample along it.
        using namespace Eigen;
        using namespace std;

        deque<Vector3f> vv;
        for (auto && p : lane) vv.push_back(Vector3f(p.v));
        auto eig = getPCEigenvalues<decltype(vv), Vector3f, Matrix3f>(vv);

        Vector3f center(0,0,0);
        for (auto && v : vv) center += v; center /= vv.size();
        map<float, int, greater<float>> maxEig; // the first element (i.e. begin) will be the largest eig value
        maxEig[eig.first[0]] = 0;
        maxEig[eig.first[1]] = 1;
        maxEig[eig.first[2]] = 2;
        BBoxPC newlane;
        // with step 0.5 meters:
        for (float f = -sqrt(maxEig.begin()->first); f < sqrt(maxEig.begin()->first); f += 0.5f)
        {
            Vector3f v(0.0f, 0.0f, 0.0f); v[maxEig.begin()->second] = f;
            v = eig.second * v + center; // transform to World
            newlane.push_back(Point(v[0], v[1], v[2]));
        }
        lane = move(newlane);
    }
};