#include <vector>

#include <eigen3/Eigen/Eigen>

enum Level {X,Y,Z};

struct __attribute__((packed)) Point
{
    Point() {}
    Point(float x, float y, float z)
    {
        v[0] = x;
        v[1] = y;
        v[2] = z;
    }
    float v [3];
    int32_t intensity;
    int32_t t_lo, t_hi;
    float operator[](int i)  { return v[i]; }
};

struct KdNode
{
    KdNode() : parent(nullptr), lchild(nullptr), rchild(nullptr) {}
    KdNode(Point _p) : parent(nullptr), lchild(nullptr), rchild(nullptr), p(_p) {}
    KdNode * parent;
    KdNode * lchild;
    KdNode * rchild;
    float c(Level l) { return p[int(l)]; } // component
    Point p;

    friend std::ostream & operator << (std::ostream & os, const KdNode * node)
    {
        if (node->lchild) os << node->lchild;
        if (node->rchild) os << node->rchild;
        os << node->p.v[0] << " " << node->p.v[1] << " " << node->p.v[2] << std::endl;
    }
};


class KdTree
{
public:
    KdTree() : root(nullptr) {}
    ~KdTree()
    {
        clear();
    }
    void clear()
    {
        clear(root);
    }
    Level next(Level l) { return Level((int(l)+1)%3); }
    void addNode(KdNode * node)
    {
        if (!root)
        {
            root = node;
            return;
        }
        addNode(root, node, X);
    }

    friend std::ostream & operator << (std::ostream & os, const KdTree & tree)
    {
        os << tree.root;
    }

private:
    void clear(KdNode * node)
    {
        if (node->lchild) clear(node->lchild);
        if (node->rchild) clear(node->rchild);
        delete node;
    }

    void addNode(KdNode * parent, KdNode * node, Level level)
    {
        if (node->c(level) < parent->c(level))
        {
            if (!parent->lchild) parent->lchild = node;
            else addNode(parent->lchild, node, next(level));
        }
        else
        {
            if (!parent->rchild) parent->rchild = node;
            else addNode(parent->rchild, node, next(level));
        }
    }



private:
    KdNode * root;
};