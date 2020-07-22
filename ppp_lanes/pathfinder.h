#pragma once

#include "quantizer.h"

#include <map>
#include <list>

#include <eigen3/Eigen/Eigen>

#define NSCAN 4 // distance (in cells) for neighbours scan for longest path detection

class PathFinder : public Quantizer
{
public:
    PathFinder(float _cS) : Quantizer(_cS) {}

    void process(BBoxPC & container)
    {
        Quantizer::process(container);
        paths.clear();

        using namespace Eigen;
        using namespace std;

        // figure out the longest paths from the unvisited point:
        for (auto && bucket : buckets)
        {
            Point & bP = bucket.second.front(); // we do not check if the front exists, cause it should, since the bucket is here.
            
            if (bP.isVisited)
                continue;

            BBoxPC path;
            searchPath(bP, path);

            if (!path.empty()) paths.push_back(path);
        }

        // set all the points in the buckets again as isVisited = false
        for (auto && bucket : buckets) bucket.second.front().isVisited = false;

        removeRedundansies();

        // since buckets is a map, a bucket thereof can be located ANYWHERE
        // this means that we could have started collecting a chain of points starting from a middle of a path
        // do another iteration to make sure the paths are spanning really the extreme start-end points:
        auto pathscp = move(paths);
        for (auto && l : pathscp)
        {
            Point & bP = l.back();

            BBoxPC path;
            searchPath(bP, path);

            if (!path.empty()) paths.push_back(path);
        }

        removeRedundansies();

        finishProcess(container);
    }


private:

    void searchPath(Point & bP, BBoxPC & path)
    {
        using namespace Eigen;
        using namespace std;

        buckets[bP.index].front().isVisited = true;
        path.push_back(bP);

        BBoxPC local;
        int index = bP.index;
        int r = index / W;
        int c = index - r * W;
        for (int nc = 0; nc >= -NSCAN; nc = nc < 0 ? -nc : -(nc + 1) ) // neighbour col
        {
            for (int nr = 0; nr >= -NSCAN; nr = nr < 0 ? -nr : -(nr + 1) ) // neighbour row
            {
                int i = (r + nr) * W + (c + nc);
                if (i >= W*H) continue;
                auto it = buckets.find(i);
                if (it == buckets.end())
                    continue;
                if (it->second.front().isVisited)
                    continue;
                // the row and column are NOT enough, since the radius must be checked:

//   | 4*NSCAN |
//   |      .  |B
//   |         *------------
//   |         |  .      | 4*NSCAN however the distance between two points A and B is diagonal, i.e. a larger distance
//   |         |     .   |
//   *         |        -----
//  A  .       |
//       .
//         .
                Point nP = it->second.front();
                Vector3f nPbP = Vector3f(bP.v) - Vector3f(nP.v); // vector between nP and bP (A and B on the )
                if (nPbP.squaredNorm() > (NSCAN*cS)*(NSCAN*cS)) continue;

                // still another additional test could be very helpfull:
                // try to figure out if the neighbour will make a rapid turn (diverge from a straight line)
                // In this case we will avoid jumping from A to B even if the distance is small between them
                if (path.size() > NSCAN)
                {
                    deque<Vector3f> vv;
                    // collect the NSCAN last neighbours of the path (plus the bP point), making 4 intervals (one interval is ~cS long)
                    for (int i = 0; i < NSCAN + 1; ++i) vv.push_back(Vector3f(path[path.size() - 1 - i].v));
                    auto eig = getPCEigenvalues<decltype(vv), Vector3f, Matrix3f>(vv, true);
                    // make sure the main component (standard deviation) is significantly larger than the next largest one.
                    // thereby the following assumption is taken: the current marking segment is ONE interval width (~cS) and NSCAN intervals long:
                    if (sqrt(eig.first[1])/sqrt(eig.first[0]) < 1.0f/NSCAN) // NSCAN times larger than the other one
                    {
                        // make sure the nP is within the BBox of 8*NSCAN x 4*NSCAN (local x and y, assuming x is path direction) from the bP:
                        BBox bbox;
                        bbox.addPoint(Point(-4*NSCAN*cS, -2*NSCAN*cS, -2*NSCAN*cS).v);
                        bbox.addPoint(Point( 4*NSCAN*cS,  2*NSCAN*cS,  2*NSCAN*cS).v);
                        // is nP in the bbox?
                        // Find the nP as vector from the bP and transform into local CS of the bbox:
                        Vector3f nPlocal = eig.second.transpose() * nPbP;
                        if (!bbox.hasPoint(Point(nPlocal[0], nPlocal[1], nPlocal[2])))
                            continue;
                    }
                }
                // ----- end of the additional test (the block is not tested thoroughly yet)

                local.push_back(it->second.front());
            }
        }

        if (local.empty()) return;

        bool firstChild = true;
        BBoxPC snapshot;
        if (local.size() > 1) snapshot = path; // do copy only if needed (optimization)

        for (auto && p : local)
        {
            if (firstChild)
            {
                searchPath(p, path);
                firstChild = false;
            }
            else
            {
                BBoxPC cpl = snapshot;
                searchPath(p, cpl);
                paths.push_back(move(cpl));
            }
        }
    }

    // remove the commong points, ex.:

    // the two paths having common points:
    // ........................    path 1
    // ..............
    //                .
    //                  . path 2

    // will be reduced to:

    // ........................    path 1
    //                .
    //                  . path 2 (only two points left in path2)
    void removeRedundansies()
    {
        using namespace std;

        multimap<size_t, BBoxPC*> pathsM; // paths by size
        for (auto && l : paths) pathsM.insert(make_pair<size_t, BBoxPC*>(l.size(), &l));

        for (auto it = pathsM.begin(); it != pathsM.end(); ++it)
        {
            auto && path = *(*it).second;
            int lMax2Pop = 0;
            auto nit = it; nit++;
            for (; nit != pathsM.end(); ++nit) // next iterator after it
            {
                auto && npath = *(*nit).second;
                int nMax2Pop = 0;
                for (int i = 0; i < path.size(); ++i)
                {
                    if (path[i].index == npath[i].index) ++nMax2Pop;
                }
                if (lMax2Pop < nMax2Pop) lMax2Pop = nMax2Pop;
            }
            for (int i = 0; i < lMax2Pop; i++) path.pop_front(); // KB: the BBox is not updated!!
        }

        auto pathscp = move(paths);
        for (auto && l : pathscp) if (l.size() >= NSCAN) paths.push_back(l);
    }


protected:

    void finishProcess(BBoxPC & container)
    {
        container.clear();
        auto pathscp = move(paths);
        // Fillout the container
        int li = 0;
        for (auto && l : pathscp)
        {
            if (l.size() < NSCAN) continue;

            for (int i = 0; i < l.size(); ++i)
            {
                if (i != 0)
                    memcpy(l[i].color, &colors[li][0], 3*sizeof(unsigned char)); 
                else 
                {
                    unsigned char RGB[3] = {255,255,255}; // This will visually mark the start of the path
                    memcpy(l[i].color, RGB, 3*sizeof(unsigned char));
                }
                container.push_back(l[i]);
            }
            ++li;
            paths.push_back(l);
        }
    }


public:
    std::deque<BBoxPC> paths;
};