#include "pathmerger.h"

#define MINLANESZ 8 // minimum 15 points making lane
#define LANEW 0.4

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
            container.clear();
            return;
        }

        for (auto &&lane : lanes)
        {
            bool isOverlap = false;
            // WE ARE CHANGING THE PATHS!!!
            for (auto &&path : paths)
            {
                if (!lane.bbox.crossing(path.bbox)) continue;
                BBoxPC flatLane = lane;
                for (auto && p : path) flatLane.push_back(p);
                auto s = flatLane.size();
                PathMerger pm(cS, holesSZ + 1.0f); // holes closing should be a bit larger than default
                pm.process(flatLane);
                // If more than one path is generated, it means that path and lane are NOT overlapping:
                if (pm.paths.size() > 1) continue;

                path = pm.paths[0];
                isOverlap = true;
            }
            if (!isOverlap)
            {
                if (lane.size() > MINLANESZ) // filter out too short lanes
                    lanesmap[laneID++] = move(lane);
                else
                    lane.clear();
            }
            else lane.clear();
        }

        // All lanes MUST be empty now, so clear lanes container:
        lanes.clear();
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
        // After resampling the lane may lose all points, i.e. become empty, discard it
        if (lane.empty()) return;
        // The lane may be now reversed, check this:
        if ((Vector3f(lane.front().v) - Vector3f(st.v)).norm() > (Vector3f(lane.back().v) - Vector3f(st.v)).norm())
            reverse(lane.begin(), lane.end());
    }

private:
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