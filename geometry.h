#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <iostream>
#include <cmath>

#include "opencv2/opencv.hpp"

#include <QDebug>

namespace Geometry {
class Point: public cv::Point {
public:
    double EPS = 1E-9;
    Point(int x=0, int y=0, double eps = 1E-9): cv::Point(x, y), EPS(eps) {}
    Point(const cv::Point &p): cv::Point(p.x, p.y) {}
    bool operator<(const Point &p) const { return ((x<p.x-EPS) || (std::abs(x-p.x)<EPS && y<p.y-EPS)); }
    Point &operator =(const cv::Point2d &p) { x = p.x; y = p.y; return *this;}
    Point &operator =(const cv::Point &p) { x = p.x; y = p.y; return *this;}
    int &operator[](int idx) { return (idx==0)? x : y; }
    const int &operator[](int idx) const { return (idx==0)? x : y; }
    size_t size() const { return 2; }
};
};

namespace Geometry {
class Line {
public:
    double a, b, c;
    double EPS = 1E-9;
    Line() {}
    Line (Point p, Point q) {
        a = p.y - q.y;
        b = q.x - p.x;
        c = - a * p.x - b * p.y;
        norm();
    }

    void norm() { double z = sqrt(a*a + b*b); if ( std::abs(z)>EPS ) { a /= z,  b /= z,  c /= z; } }
    double dist(Point p) const { return a * p.x + b * p.y + c; }
};
};


namespace Geometry {
class Intersect
{
public:
    Intersect();

    #define EPS 1E-9

    #define det(a,b,c,d)  (a*d-b*c)




    static inline bool betw(double l, double r, double x) { return ( (std::min(l,r)<=x+EPS) && (x<=std::max(l,r)+EPS) ); }
    static inline bool intersect_1d(double a, double b, double c, double d) { if ( a>b ) { std::swap(a, b); } if ( c>d ) { std::swap(c, d); } return ( std::max(a, c)<=std::min(b, d)+EPS ); }

    static bool intersect(Point a, Point b, Point c, Point d, Point &dot)
    {
        if ( !intersect_1d(a.x, b.x, c.x, d.x) || !intersect_1d(a.y, b.y, c.y, d.y) ) { return false; }
        Line m(a, b);
        Line n(c, d);
        double zn = det(m.a, m.b, n.a, n.b);

        if ( std::abs(zn)<EPS ) { return false; }

        dot.x = -det(m.c, m.b, n.c, n.b)/zn;
        dot.y = -det(m.a, m.c, n.a, n.c)/zn;

        return betw(a.x, b.x, dot.x) && betw(a.y, b.y, dot.y) && betw(c.x, d.x, dot.x) && betw(c.y, d.y, dot.y);
    }

    /**
    * @brief belong
    * @param a
    * @param b
    * @param p
    * @return
    */
    static bool belong(Point p1, Point p2, Point t)
    {
        if ( t==p1 || t==p2 ) { return true; }

        double a = p2.y - p1.y;
        double b = p1.x - p2.x;
        double c = -a * p1.x-b*p1.y;
        if ( std::abs(a*t.x+b*t.y+c)>EPS ) { return false; }

        return ( (t.x>=std::min(p1.x,p2.x)) && (t.x<=std::max(p1.x,p2.x)) && (t.y>=std::min(p1.y,p2.y)) && (t.y<=std::max(p1.y,p2.y)) );
    }

};
}

#endif // GEOMETRY_H
