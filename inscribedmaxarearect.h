#ifndef INSCRIBEDMAXAREARECT_H
#define INSCRIBEDMAXAREARECT_H

#include <vector>
#include <stack>

#include "earcut.hpp/include/mapbox/earcut.hpp"
#include "geometry.h"

class InscribedMaxAreaRect
{
public:
    InscribedMaxAreaRect();
    InscribedMaxAreaRect(const std::vector<Geometry::Point> &polygon);

    void setPolygon(const std::vector<Geometry::Point> &polygon);
    const std::vector<Geometry::Point> & polygon() const;
    const std::vector<int>& raysXAnchors() const;
    const std::vector<int>& raysYAnchors() const;
    const std::vector<int>& raysXEdges() const;
    const std::vector<int>& raysYEdges() const;
    const std::vector<Geometry::Point> edgesIntersetcPoints() const;
    const Geometry::Rect& boundingBox() const;
    const std::vector<Geometry::Rect>& subRects() const;
    const std::vector<uint32_t>& trianglesIndices() const;
    const std::vector<double>& subRectsAreasInscribed() const;
    size_t subRectsInscribedCount() const;
    const Geometry::Rect& inscribedMaxAreaRect() const;
    double polygonArea() const;

    bool findMaxAreaInscribedRect();

    bool makeAnchrorRays();
    bool makeEdgeIntersectRays();
    bool makeRectangles();
    bool makePolygonTriangulation();
    bool makeAreasSubRectsInscribed();
    bool makeMaxAreaRectBySubRects();
    void reset();

private:
    std::vector<Geometry::Point> _polygon; //-- Input polygon
    std::vector<int> _raysXAnchors, _raysYAnchors; //-- Ray coordinate by axis X, Y
    std::vector<int> _raysXEdges, _raysYEdges; //-- Ray coordinate intersect edges axis X, Y
    std::vector<Geometry::Point> _edgesIntersectPoints;
    Geometry::Rect _bb; //-- Bounding box of polygon
    std::vector<Geometry::Rect> _subRects; //-- Sub rectangles inscribed boundiong box
    std::vector<uint32_t> _trianglesIndices; //-- Triangle point indeces by _polygon points
    double _polygonArea;
    std::vector<double> _subRectsAreas; //-- Areas of inscribed sub rectangles. Positive values - rectangle inside polygon.
    size_t _subeRectsInscribedCount; //-- Count of sub rects in polygon
    Geometry::Rect _inscribedMaxAreaRect; //-- Result max area inscribed rectangle

    double kadane(double col[], int64_t &start, int64_t &finish, int64_t n);
};


#endif // INSCRIBEDMAXAREARECT_H
