#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <iostream>
#include <cmath>

#include "opencv2/opencv.hpp"

#include <QDebug>

namespace Geometry {

class Point {
public:
    Point(int x=0, int y=0, double eps = 1E-9): x(x), y(y), eps(eps) {}
    Point(const Point &p): x(p.x), y(p.y), eps(p.eps) {}

    bool operator<(const Point &p) const { return ((x<p.x-eps) || (std::abs(x-p.x)<eps && y<p.y-eps)); }
    bool operator==(const Point &p) const { return (std::abs(x-p.x)<eps && std::abs(y-p.y)<eps); }
    bool operator!=(const Point &p) const { return !(*this==p); }

    int &operator[](int idx) { return (idx==0)? x : y; }
    const int &operator[](int idx) const { return (idx==0)? x : y; }

    size_t size() const { return 2; }

    double dist(const Point &p) const { return std::sqrt(std::pow(p.x-x, 2.0f) +std::pow(p.y - y, 2.0f)); }

    int x,y;
    double eps;
};

class Line {
public:
    double a, b, c;
    double EPS;
    Line(double eps = 1E-9): EPS(eps) {}
    Line (Point p, Point q, double eps = 1E-9): EPS(eps) {
        a = p.y - q.y;
        b = q.x - p.x;
        c = - a * p.x - b * p.y;
        norm();
    }

    void norm() { double z = sqrt(a*a + b*b); if ( std::abs(z)>EPS ) { a /= z,  b /= z,  c /= z; } }
    double dist(Point p) const { return a * p.x + b * p.y + c; }
};

class Rect {
public:
    Rect(double l=0, double t=0, double r=0, double b=0): l(l), t(t), r(r), b(b) {}

    void set(double sl, double st, double sr, double sb) { l = sl; t = st; r = sr; b = sb; }
    void set(const Geometry::Point &tl, const Geometry::Point &br) { t = tl.y; l = tl.x; b = br.y; r = br.x; }
    void setTL(const Geometry::Point &tl) { t = tl.y; l = tl.x; }
    void setBR(const Geometry::Point &br) { b = br.y; r = br.x; }
    void setWidth(double w) { r = l+w; }
    void setHeight(double h) { b = t+h; }

    Geometry::Point tl() const { return Geometry::Point(l,t); }
    Geometry::Point tr() const { return Geometry::Point(r,t); }
    Geometry::Point bl() const { return Geometry::Point(l,b); }
    Geometry::Point br() const { return Geometry::Point(r,b); }
    double width() const { return r-l; }
    double height() const { return b-t; }
    double area() const { return width()*height(); }

    double l,t,r,b;
};

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

    /**
    * @brief pointInTriangle
    * @param a
    * @param b
    * @param c
    * @param p
    * @return
    */
    static bool pointInTriangle(const Geometry::Point &a, const Geometry::Point &b, const Geometry::Point &c, const Geometry::Point &p)
    {
        return (c.x - p.x) * (a.y - p.y) - (a.x - p.x) * (c.y - p.y) >= 0 &&
               (a.x - p.x) * (b.y - p.y) - (b.x - p.x) * (a.y - p.y) >= 0 &&
               (b.x - p.x) * (c.y - p.y) - (c.x - p.x) * (b.y - p.y) >= 0;
    }

};

}

#endif // GEOMETRY_H
