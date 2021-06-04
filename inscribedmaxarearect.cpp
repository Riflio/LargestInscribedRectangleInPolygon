#include "inscribedmaxarearect.h"

/**
* @brief Find max area inscribed rectangle in polygon
* @return
*/
bool InscribedMaxAreaRect::findMaxAreaInscribedRect()
{
    if ( _polygon.size()<4 ) { return false; }
    if ( !makeAnchrorRays() ) { return false; }
    if ( !makeEdgeIntersectRays() ) { return false; }
    if ( !makeRectangles() ) { return false; }
    if ( !makePolygonTriangulation() ) { return false; }
    if ( !makeAreasSubRectsInscribed() ) { return false; }
    if ( !makeMaxAreaRectBySubRects() ) { return false; }

    return true;
}


InscribedMaxAreaRect::InscribedMaxAreaRect()
{
    reset();
}

InscribedMaxAreaRect::InscribedMaxAreaRect(const std::vector<Geometry::Point> &polygon)
{
    setPolygon(polygon);
}

void InscribedMaxAreaRect::setPolygon(const std::vector<Geometry::Point> &polygon)
{
    reset();
    _polygon = polygon;
}

/**
* @brief Create rays from polygon anchors point
* @return
*/
bool InscribedMaxAreaRect::makeAnchrorRays()
{
    _bb = Geometry::Rect(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0, 0);

    for (size_t i=0; i<_polygon.size(); ++i) {
        const Geometry::Point &point = _polygon.at(i);
        if ( point.x<_bb.l ) { _bb.l = point.x; }
        if ( point.x>_bb.r ) { _bb.r = point.x; }
        if ( point.y<_bb.t ) { _bb.t = point.y; }
        if ( point.y>_bb.b ) { _bb.b = point.y; }
        _raysXAnchors.push_back(point.x);
        _raysYAnchors.push_back(point.y);
    }

    return true;
}

/**
* @brief Find edge intersect rays and create new by other axis recursive
* @return
*/
bool InscribedMaxAreaRect::makeEdgeIntersectRays()
{
    _raysXEdges.insert(_raysXEdges.end(), _raysXAnchors.begin(), _raysXAnchors.end());
    _raysYEdges.insert(_raysYEdges.end(), _raysYAnchors.begin(), _raysYAnchors.end());

    size_t xi = 0, xii =0,  yi = 0, yii = 0;
    size_t polygonPointsCount = _polygon.size();
    size_t raysXCount = _raysXEdges.size(), raysYCount = _raysYEdges.size();
    size_t iter = 0;
    Geometry::Point intersectPoint;

    while ( xi<raysXCount || yi<raysYCount ) {
        for (size_t ei=1; ei<=polygonPointsCount; ++ei) {
            const Geometry::Point &edgeStartPoint = _polygon[ei-1];
            const Geometry::Point &edgeEndPoint = (ei==polygonPointsCount)? _polygon[0] : _polygon[ei];

            if ( xi<raysXCount ) {
                if ( Geometry::Intersect::intersect(Geometry::Point(_raysXEdges[xi], _bb.t), Geometry::Point(_raysXEdges[xi], _bb.b), edgeStartPoint, edgeEndPoint, intersectPoint) ) {
                    if ( intersectPoint!=edgeEndPoint && intersectPoint!=edgeStartPoint ) {
                        for (yii=0; yii<raysYCount; ++yii) { if ( std::abs(_raysYEdges[yii]-intersectPoint.y)<=1 ) { break; } }
                        if ( yii==raysYCount ) { _raysYEdges.push_back(intersectPoint.y); _edgesIntersectPoints.push_back(intersectPoint); }
                    }
                }
            }

            if ( yi<raysYCount ) {
                if ( Geometry::Intersect::intersect(Geometry::Point(_bb.l, _raysYEdges[yi]), Geometry::Point(_bb.r, _raysYEdges[yi]), edgeStartPoint, edgeEndPoint, intersectPoint) ) {
                    if ( intersectPoint!=edgeEndPoint && intersectPoint!=edgeStartPoint ) {
                        for (xii=0; xii<raysXCount; ++xii) { if ( std::abs(_raysXEdges[xii]-intersectPoint.x)<=1 ) { break; } }
                        if ( xii==raysXCount ) { _raysXEdges.push_back(intersectPoint.x); _edgesIntersectPoints.push_back(intersectPoint); }
                    }
                }
            }
        }

        raysXCount = _raysXEdges.size();
        raysYCount = _raysYEdges.size();

        if ( xi<raysXCount ) { xi++; }
        if ( yi<raysYCount ) { yi++; }

        iter++;
    }

    return true;
}


/**
* @brief Create rectangles from XY rays
* @return
*/
bool InscribedMaxAreaRect::makeRectangles()
{
    std::sort(_raysXEdges.begin(), _raysXEdges.end());
    std::sort(_raysYEdges.begin(), _raysYEdges.end());

    for (size_t yi=1; yi<_raysYEdges.size(); ++yi) {
        double top = _raysYEdges[yi-1];
        double bot = _raysYEdges[yi];
        for (size_t xi=1; xi<_raysXEdges.size(); ++xi) {
            double left = _raysXEdges[xi-1];
            double right = _raysXEdges[xi];
            _subRects.push_back(Geometry::Rect(left, top, right, bot));
        }
    }

    return true;
}

/**
* @brief Make polygon triangulation
* @return
*/
bool InscribedMaxAreaRect::makePolygonTriangulation()
{
    std::vector<std::vector<Geometry::Point>> ecCpolygon = {_polygon};
    _trianglesIndices = mapbox::earcut<uint32_t>(ecCpolygon);

    _polygonArea = 0;
    for (size_t ti=0; ti<_trianglesIndices.size(); ti+=3) {
        const Geometry::Point &p1 = _polygon[_trianglesIndices[ti+0]], &p2 = _polygon[_trianglesIndices[ti+1]], &p3 = _polygon[_trianglesIndices[ti+2]];
        _polygonArea += std::abs((p2.x-p1.x)*(p3.y-p1.y)-(p3.x-p1.x)*(p2.y-p1.y))/2.0f;
    }

    return ( _trianglesIndices.size()>0 );
}

/**
* @brief Find subRects inscribed in polygon by triangulation and make areas
* @return inside count
*/
bool InscribedMaxAreaRect::makeAreasSubRectsInscribed()
{
    for (size_t ri=0; ri<_subRects.size(); ++ri) {
        const Geometry::Rect &rect = _subRects.at(ri);
        bool rp1 = false, rp2 = false, rp3 = false, rp4 = false;

        for (size_t ti=0; ti<_trianglesIndices.size(); ti+=3) { //-- Все 4 точки прямоугольника должны принадлежать какому-либо треугольнику
            const Geometry::Point &tp1 = _polygon[_trianglesIndices[ti]], &tp2 = _polygon[_trianglesIndices[ti+1]], &tp3 = _polygon[_trianglesIndices[ti+2]];

            if ( !rp1 && Geometry::Intersect::pointInTriangle(tp1, tp2, tp3, rect.tl()) ) { rp1 = true; }
            if ( !rp2 && Geometry::Intersect::pointInTriangle(tp1, tp2, tp3, rect.tr()) ) { rp2 = true; }
            if ( !rp3 && Geometry::Intersect::pointInTriangle(tp1, tp2, tp3, rect.br()) ) { rp3 = true; }
            if ( !rp4 && Geometry::Intersect::pointInTriangle(tp1, tp2, tp3, rect.bl()) ) { rp4 = true; }

            if ( rp1 && rp2 && rp3 && rp4 ) { break; }
        }

        bool isInside = (rp1 && rp2 && rp3 && rp4);

        _subRectsAreas.push_back((isInside)? rect.area() : INT_MIN); //TODO: Make proportional area by bounding box
        _subeRectsInscribedCount+= (isInside)? 1: 0;
    }

    return (_subeRectsInscribedCount>0);
}

/**
* @brief Implementation of Kadane's algorithm for 1D array.
* @param arr
* @param start
* @param finish
* @param n
* @return
*/
int64_t InscribedMaxAreaRect::kadane(int64_t col[], int64_t* start, int64_t* finish, int64_t n)
{
    int64_t sum = 0, maxSum = INT_MIN, i;
    *finish = -1;
    int64_t local_start = 0;

    for (i=0; i<n; ++i) {
        sum += col[i];
        if ( sum<0 ) {
            sum = 0;
            local_start = i + 1;
        } else
        if ( sum>maxSum ) {
            maxSum = sum;
            *start = local_start;
            *finish = i;
        }
    }

    if ( *finish!=-1 ) { return maxSum; }

    maxSum = col[0];
    *start = *finish = 0;

    for (i=1; i<n; i++) {
        if ( col[i]>maxSum ) {
            maxSum = col[i];
            *start = *finish = i;
        }
    }
    return maxSum;
}


/**
* @brief Make from subRects max rect by area
* @return
*/
bool InscribedMaxAreaRect::makeMaxAreaRectBySubRects()
{

    size_t rows = _raysYEdges.size()-1, cols = _raysXEdges.size()-1;
    int64_t maxSum = INT_MIN, rLeft, rRight, rTop, rBottom;
    int64_t temp[rows], sum, start, finish;

    int64_t * M = &_subRectsAreas[0];

    for (size_t left=0; left<cols; ++left) {
        memset(temp, 0, sizeof(temp));
        for (size_t right=left; right<cols; ++right) {
            for (size_t i=0; i<rows; ++i) { temp[i] += *(M+i*cols+right); }
            sum = kadane(temp, &start, &finish, rows);
            if ( sum>maxSum ) {
                maxSum = sum;
                rLeft = left;
                rRight = right;
                rTop = start;
                rBottom = finish;
            }
        }
    }

    _inscribedMaxAreaRect.set(_subRects[rTop*cols+rLeft].tl(), _subRects[rBottom*cols+rRight].br());

    return true;
}

/**
* @brief Reset all
*/
void InscribedMaxAreaRect::reset()
{
    _polygon.clear();
    _raysXAnchors.clear();
    _raysYAnchors.clear();
    _raysXEdges.clear();
    _raysYEdges.clear();
    _edgesIntersectPoints.clear();
    _bb = Geometry::Rect();
    _subRects.clear();
    _trianglesIndices.clear();
    _polygonArea = 0;
    _subRectsAreas.clear();
    _subeRectsInscribedCount = 0;
    _inscribedMaxAreaRect = Geometry::Rect();
}

namespace mapbox { namespace util {
    template <>
    struct nth<0, Geometry::Point> {
        inline static int get(const Geometry::Point &t) {
            return t.x;
        };
    };
    template <>
    struct nth<1, Geometry::Point> {
        inline static int get(const Geometry::Point &t) {
            return t.y;
        };
    };
}}

const std::vector<Geometry::Rect> &InscribedMaxAreaRect::subRects() const
{
    return _subRects;
}

const std::vector<uint32_t> &InscribedMaxAreaRect::trianglesIndices() const
{
    return _trianglesIndices;
}

const std::vector<int64_t> &InscribedMaxAreaRect::subRectsAreasInscribed() const
{
    return _subRectsAreas;
}

size_t InscribedMaxAreaRect::subRectsInscribedCount() const
{
    return _subeRectsInscribedCount;
}

const Geometry::Rect &InscribedMaxAreaRect::boundingBox() const
{
    return _bb;
}

const std::vector<Geometry::Point> &InscribedMaxAreaRect::polygon() const
{
    return _polygon;
}

const std::vector<int>& InscribedMaxAreaRect::raysXAnchors() const
{
    return _raysXAnchors;
}

const std::vector<int>& InscribedMaxAreaRect::raysYAnchors() const
{
    return _raysYAnchors;
}

const std::vector<int> &InscribedMaxAreaRect::raysXEdges() const
{
    return _raysXEdges;
}

const std::vector<int> &InscribedMaxAreaRect::raysYEdges() const
{
    return _raysYEdges;
}

const std::vector<Geometry::Point> InscribedMaxAreaRect::edgesIntersetcPoints() const
{
    return _edgesIntersectPoints;
}

double InscribedMaxAreaRect::polygonArea() const
{
   return _polygonArea;
}

const Geometry::Rect &InscribedMaxAreaRect::inscribedMaxAreaRect() const
{
    return _inscribedMaxAreaRect;
}

