#include "pathmerger.h"

#include <iostream>

#define MINLANESZ 8 // minimum number of points making marking segment
#define LANEW 0.4

class MarkingDetector : public PathMerger
{
public:
    MarkingDetector(float _cS) : PathMerger(_cS) {}

public:

    void process(BBoxPC & container, std::map<int, BBoxPC> & marksmap)
    {
        PathMerger::process(container);

        using namespace std;
        using namespace Eigen;

        bbox = container.bbox; // remember for figuring out the direction of motion (nice to have feature)

        marksmap.clear();
        container.clear();

        if (paths.empty()) return;

        auto aggregate = [](BBoxPC & path1, BBoxPC & path2, float cS, float hS) -> bool
        {
            if (!path1.bbox.crossing(path2.bbox)) return false;
            BBoxPC flatContainer = path1;
            for (auto && p : path2) flatContainer.push_back(p);
            PathMerger pm(cS, hS); // holes closing should be a bit larger than default
            pm.process(flatContainer);
            // If more than one path is generated, it means that (most probably) the paths are NOT overlapping:
            if (pm.paths.size() > 1) return false;
            // Nice to have: figure out the  begin and end:
            path1 = pm.paths[0]; // move operator causes crash, hmmmm
            return true;
        };

        for (auto &&mark : marks)
        {
            bool isOverlap = false;
            for (auto &&path : paths)
            {
                if (!aggregate(path, mark, cS, holesSZ + 1.0f)) continue;
                isOverlap = true;
            }
            if (!isOverlap)
            {
                if (mark.size() > MINLANESZ) // filter out too short marks
                    marksmap[markID++] = move(mark);
                else
                    mark.clear();
            }
            else mark.clear();
        }

        // This check can significantly reduce the performance but produces cleaner output
        // since ex. two paths may be spanned by one mark -> redundancy
        for (auto it1 = paths.begin(); it1 != paths.end(); ++it1)
        {
            auto && path1 = *it1;
            for (auto it2 = it1 + 1; it2 != paths.end(); ++it2)
            {
                auto && path2 = *it2;
                if (path2.empty()) continue;

                if (!aggregate(path1, path2, cS, holesSZ + 1.0f)) continue;
                path2.clear();
            }
        }

        // All marks MUST be empty now, so clear marks container:
        marks.clear();
        // rest of the non-overlapping paths:
        for (auto && path : paths) if (!path.empty()) marks.push_front(move(path));

        // PATHS ARE EMPTY at this point !!!!!!!!!
        getResults(container, marksmap);
    }

public:

    void getResults(BBoxPC & container, std::map<int, BBoxPC> & marksmap, bool isFinal = false)
    {
        using namespace Eigen;

        container.clear();
        if (isFinal)
            for (auto && mark : marks) if (mark.size() > MINLANESZ) marksmap[markID++] = move(mark);

        if (marksmap.empty()) return;

        // Fillout the container
        int li = 0;
        for (auto && it : marksmap)
        {
            auto && mark = it.second;

            resample(mark);
            // After resampling the mark may lose all points, i.e. become empty, discard it
            if (mark.empty()) continue;

            // set proper begin - end for the mark, depending on the motion
            Vector3f direction = bbox.center() - mark.bbox.center();
            if ((Vector3f(mark.back().v) - Vector3f(mark.front().v)).dot(direction) < 0)
                reverse(mark.begin(), mark.end());

            for (int i = 0; i < mark.size(); ++i)
            {
                if (i != 0)
                    memcpy(mark[i].color, &colors[li][0], 3*sizeof(unsigned char)); 
                else 
                {
                    unsigned char RGB[3] = {255,255,255}; // This will visually mark the start of the path
                    memcpy(mark[i].color, RGB, 3*sizeof(unsigned char));
                }
                container.push_back(mark[i]);
            }
            ++li;
        }
    }

protected:

    virtual void resample(BBoxPC & mark)
    {
        using namespace Eigen;

        // Remember the start point of mark:
        auto st = mark.front();
        // resample with a new step size (ex. 1 meter), we set the closing holes a bit larger than before due to new (larger) quantizing
        PathMerger pf(1.0f, 10.0f); pf.process(mark);
        // After resampling the mark may lose all points, i.e. become empty, discard it
        if (mark.empty()) return;
        // The mark may be now reversed, check this:
        if ((Vector3f(mark.front().v) - Vector3f(st.v)).norm() > (Vector3f(mark.back().v) - Vector3f(st.v)).norm())
            reverse(mark.begin(), mark.end());
    }

private:
    std::deque<BBoxPC> marks;
    BBox bbox; // to figure out the direction of motion
    static int markID;
};



class MarkingDetectorCont : public MarkingDetector
{
public:
    MarkingDetectorCont(float _cS) : MarkingDetector(_cS) {}
};

class MarkingDetectorCurb : public MarkingDetector
{
public:
    MarkingDetectorCurb(float _cS) : MarkingDetector(_cS) {}
};

class MarkingDetectorDash : public MarkingDetector
{
public:
    MarkingDetectorDash(float _cS) : MarkingDetector(_cS) {}
};


class MarkingDetectorStop : public MarkingDetector
{
public:
    MarkingDetectorStop(float _cS) : MarkingDetector(_cS) {}

    void resample(BBoxPC & mark) override
    {
        // The stop line is short and usually dense PC. Get the PCA thereof to get its main axis and resample along it.
        using namespace Eigen;
        using namespace std;

        deque<Vector3f> vv;
        for (auto && p : mark) vv.push_back(Vector3f(p.v));
        auto eig = getPCEigenvalues<decltype(vv), Vector3f, Matrix3f>(vv, true);

        Vector3f center(0,0,0);
        for (auto && v : vv) center += v; center /= vv.size();
        BBoxPC newmark;
        // with step 0.5 meters:
        for (float f = -sqrt(eig.first[0]); f < sqrt(eig.first[0]); f += 0.5f)
        {
            Vector3f v(f, 0.0f, 0.0f);
            v = eig.second * v + center; // transform to World
            newmark.push_back(Point(v[0], v[1], v[2]));
        }
        mark = move(newmark);
    }
};