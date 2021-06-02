#ifndef TRIANGULATION_H
#define TRIANGULATION_H


/**
* @brief The Triangulation class
* @ref http://www.opita.net/node/29
*/

#include "geometry.h"
#include "vectorcircular.h"

namespace Geometry {


class Triangle {
public:
    Triangle(Point p1, Point p2, Point p3): p1(p1), p2(p2), p3(p3) {}

    Point p1, p2, p3;
};

class Triangulation
{
public:
    Triangulation();
    Triangulation(const std::vector<Point> &listPoints);

    std::vector<Triangle> triangulation();

    static bool isInTriangle(const Point &point, const Point &t1, const Point &t2, const Point &t3);
    static int vp(const Point &a, const Point &b, const Point &c);
    static int scp(const Point &a, const Point &b, const Point &c);


    typedef VectorCircular<Point> TPointsList;

private:
    TPointsList lp;
    int direction;
    void determineDirection();
};
}


#endif // TRIANGULATION_H
