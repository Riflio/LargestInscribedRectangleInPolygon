#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

#include <iostream>

#include "geometry.h"

#include <QDebug>


#include <stack>

#include "earcut.hpp/include/mapbox/earcut.hpp"

namespace mapbox {
namespace util {

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

} // namespace util
} // namespace mapbox




std::string _wndTitle = "MaxAreaRect";
std::vector<Geometry::Point> _polygon;




/**
* @brief findMaxRectBruteforce
* @param polygon
* @return
*/
cv::Rect findMaxRectBruteforce(const std::vector<cv::Point> &polygon)
{
    if ( polygon.size()<3 ) { return cv::Rect(0,0,0,0); }

    cv::Rect bb = cv::boundingRect(polygon);

    cv:: Mat1b M(bb.height, bb.width, uchar(0));

    std::vector<std::vector<cv::Point>> polygons = {polygon};

    if ( polygons[0][0]!=polygons[0][polygons[0].size()-1] ) { polygons[0].push_back(polygons[0][0]); } //-- Close contour

    cv::drawContours(M, polygons, 0, cv::Scalar(255), cv::FILLED);

    cv::bitwise_not(M, M);

    cv::Mat1f W(M.rows, M.cols, float(0));
    cv::Mat1f H(M.rows, M.cols, float(0));

    cv::Rect maxRect(0, 0, 0, 0);
    float maxArea = 0.0f;

    for (int r = 0; r<M.rows; ++r) {
        for (int c = 0; c<M.cols; ++c) {
            if ( M(r, c)==0 ) {
                H(r, c) = 1.0f + ((r>0)? H(r-1, c) : 0);
                W(r, c) = 1.0f + ((c>0)? W(r, c-1) : 0);
            }

            float minw = W(r, c);
            for (int h = 0; h < H(r, c); ++h) {
                minw = std::min(minw, W(r-h, c));
                float area = (h+1) * minw;
                if ( area>maxArea ) {
                    maxArea = area;
                    maxRect = cv::Rect(cv::Point(c-minw+1, r-h), cv::Point(c+1, r+1));
                }
            }
        }
    }

    return maxRect;
}



/**
* @brief Find intersect point of 2 lines
* @param p1 - line 1 start
* @param p2 - line 1 ent
* @param p3 - line 2 start
* @param p4 - line 2 end
* @param tol - tolerance compare coordinates
* @return
*/
cv::Point intersectLinesPoint(const cv::Point &p1, const cv::Point &p2, const cv::Point &p3, const cv::Point &p4, double tol=1E-9)
{
    cv::Point dot(-1, -1);
    double n;

    if ( fabs(p2.y-p1.y)>tol ) {
        double q = (p2.x-p1.x) / (p1.y-p2.y);
        double sn = (p3.x-p4.x) + (p3.y-p4.y)*q;
        if ( fabs(sn)<=tol ) { return dot; }
        double fn = (p3.x-p1.x) + (p3.y-p1.y)*q;
        n = fn/sn;
    } else {
        if ( fabs(p3.y-p4.y)<=tol ) { return dot; }
        n = (p3.y-p1.y) / (p3.y-p4.y);
    }

    dot.x = p3.x + (p4.x-p3.x)*n;
    dot.y = p3.y + (p4.y-p3.y)*n;
    return dot;
}


/**
* @brief pointInTriangle
* @param a
* @param b
* @param c
* @param p
* @return
*/
bool pointInTriangle(const Geometry::Point &a, const Geometry::Point &b, const Geometry::Point &c, const Geometry::Point &p)
{
    return (c.x - p.x) * (a.y - p.y) - (a.x - p.x) * (c.y - p.y) >= 0 &&
           (a.x - p.x) * (b.y - p.y) - (b.x - p.x) * (a.y - p.y) >= 0 &&
           (b.x - p.x) * (c.y - p.y) - (c.x - p.x) * (b.y - p.y) >= 0;
}


/*
uint64_t maxHist(int64_t row[], int64_t count, int64_t &left, int64_t &right)
{
    left = 0; right = 0;
    std::stack<int64_t> stack;
    uint64_t area = 0, areaMax = 0;
    int64_t posLeft = 0, posRight = 0;
    uint64_t topVal;

    while ( posRight<count ) {
        if ( stack.empty() || (row[stack.top()]<=row[posRight]) ) {
            stack.push(posRight++);
        } else {
            posLeft = stack.top();
            topVal = row[posLeft];
            stack.pop();
            area = topVal*posRight;

            if ( !stack.empty() ) {
                posLeft = stack.top()+1;
                area = topVal*(posRight-posLeft);
            }

            if ( area>areaMax ) {
                areaMax = area;
                left = posLeft;
                right = posRight-1;
            }

        }
    }

    while ( !stack.empty() ) {
        posLeft = stack.top();
        topVal = row[posLeft];
        stack.pop();
        area = topVal*posRight;
        if ( !stack.empty() ) {
            posLeft = stack.top()+1;
            area = topVal*(posRight-posLeft);
        }
        if ( area>areaMax ) {
            areaMax = area;
            left = posLeft;
            right = count-1;
        }
    }

    return areaMax;
}*/



/**
* @brief maxRectangle
* @param M - linear matrix R*C
* @param rows
* @param cols
* @ref https://www.geeksforgeeks.org/maximum-size-rectangle-binary-sub-matrix-1s/
* @return
*/
/*cv::Rect maxRectangle(int64_t *M, int64_t rows, int64_t cols)
{
    int64_t left = 0, right = 0 , top = 0, bottom = 0, tLeft = 0, tRight = 0;
    uint64_t area = 0, maxArea = maxHist(M+0, cols, left, right);

    for (int64_t i=1; i<rows; ++i) {
        for (int64_t j=0; j<cols; ++j) {
            if ( *(M+i*cols+j) ) { *(M+i*cols+j) += *(M+(i-1)*cols+j); }
        }

        area = maxHist(M+i*cols, cols, tLeft, tRight);

        if ( area>maxArea ) {
            maxArea = area;
            left = tLeft;
            right = tRight;
            bottom = i;
            top = bottom-(area/(right-left+1))+1;
        }
    }

    return cv::Rect(left, top, right-left, bottom-top);
}*/



/**
* @brief Implementation of Kadane's algorithm for 1D array.
* @param arr
* @param start
* @param finish
* @param n
* @return
*/
int64_t kadane(int64_t col[], int64_t* start, int64_t* finish, int64_t n)
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
* @brief findMaxSum
* @param M
* @param rows
* @param cols
* @ref https://www.geeksforgeeks.org/maximum-sum-rectangle-in-a-2d-matrix-dp-27/
*/
cv::Rect findMaxSum(int64_t M[], int64_t rows, int64_t cols)
{
    int64_t maxSum = INT_MIN, finalLeft, finalRight, finalTop, finalBottom;
    int64_t left, right, i;
    int64_t temp[rows], sum, start, finish;

    for (left=0; left<cols; ++left) {
        memset(temp, 0, sizeof(temp));
        for (right=left; right<cols; ++right) {
            for (i=0; i<rows; ++i) { temp[i] += *(M+i*cols+right); }
            sum = kadane(temp, &start, &finish, rows);
            if ( sum>maxSum ) {
                maxSum = sum;
                finalLeft = left;
                finalRight = right;
                finalTop = start;
                finalBottom = finish;
            }
        }
    }

    return cv::Rect(finalLeft, finalTop, finalRight-finalLeft, finalBottom-finalTop);
}



cv::Rect findMaxRectSubRectangles(const std::vector<Geometry::Point> &polygon, cv::Mat surface, size_t maxIters)
{
    if ( polygon.size()<4 ) { return cv::Rect(0,0,0,0); }

    double FL = std::numeric_limits<double>::max(), FR = 0, FT = std::numeric_limits<double>::max(), FB = 0; //-- Frame bounding box
    std::vector<int> XRays, YRays; //-- Ray coordinate by axis

    //-- Создаём лучи из каждой якорной точки контура
    for (size_t i=0; i<polygon.size(); ++i) {
        const Geometry::Point &point = polygon.at(i);
        if ( point.x>FR ) { FR = point.x; }
        if ( point.x<FL ) { FL = point.x; }
        if ( point.y>FB ) { FB = point.y; }
        if ( point.y<FT ) { FT = point.y; }

        XRays.push_back(point.x);
        YRays.push_back(point.y);
    }

    //-- Находим точки перемечения лучей и рёбер, выпускаем луч по другой оси и заново, пока есть.
    size_t xi = 0, xii =0,  yi = 0, yii = 0;
    size_t polygonPointsCount = polygon.size();
    size_t XRaysCount = XRays.size(), YRaysCount = YRays.size();
    size_t iter = 0;
    Geometry::Point intersectPoint;

    while ( xi<XRaysCount || yi<YRaysCount ) {
        for (size_t ei=1; ei<polygonPointsCount+1; ++ei) {
            Geometry::Point edgeStartPoint = polygon[ei-1];
            Geometry::Point edgeEndPoint = (ei==polygonPointsCount)? polygon[0] : polygon[ei];
            if ( edgeStartPoint==edgeEndPoint ) { break; }

            if ( xi<XRaysCount ) {
                if ( Geometry::Intersect::intersect(Geometry::Point(XRays[xi], FT), Geometry::Point(XRays[xi], FB), edgeStartPoint, edgeEndPoint, intersectPoint) ) {
                    if ( intersectPoint!=edgeEndPoint && intersectPoint!=edgeStartPoint ) {

                        for (yii=0; yii<YRaysCount; ++yii) {
                            if ( std::abs(YRays[yii]-intersectPoint.y)<=1 ) { break; }
                        }

                        if ( yii==YRaysCount ) {
                            YRays.push_back(intersectPoint.y);
                            cv::circle(surface, intersectPoint, 5, cv::Scalar(0,0,0), cv::FILLED);
                        }
                    }
                }
            }

            if ( yi<YRaysCount ) {
                if ( Geometry::Intersect::intersect(Geometry::Point(FL, YRays[yi]), Geometry::Point(FR, YRays[yi]), edgeStartPoint, edgeEndPoint, intersectPoint) ) {
                    if ( intersectPoint!=edgeEndPoint && intersectPoint!=edgeStartPoint ) {

                        for (xii=0; xii<XRaysCount; ++xii) {
                            if ( std::abs(XRays[xii]-intersectPoint.x)<=1 ) { break; }
                        }

                        if ( xii==XRaysCount ) {
                            XRays.push_back(intersectPoint.x);
                            cv::circle(surface, intersectPoint, 5, cv::Scalar(0,0,0), cv::FILLED);
                        }
                    }
                }
            }
        }


        XRaysCount = XRays.size();
        YRaysCount = YRays.size();

       // qDebug()<<"ITER"<<xi<<yi<<XRaysCount<<YRaysCount;

        if ( xi<XRaysCount ) { xi++; }
        if ( yi<YRaysCount ) { yi++; }

        iter++;
        //if ( iter>maxIters ) { qWarning()<<"OVER ITERATIONS!!!!"; break; }
    }


    //-- Покажем получившиеся лучи
    for (size_t xi=0;  xi<XRays.size(); ++xi) { cv::line(surface, cv::Point(XRays[xi], FT), cv::Point(XRays[xi], FB), cv::Scalar(150, 150,150), 1); }
    for (size_t yi=0;  yi<YRays.size(); ++yi) { cv::line(surface, cv::Point(FL, YRays[yi]), cv::Point(FR, YRays[yi]), cv::Scalar(150, 150,150), 1); }



    //-- Создаём прямоугольники. Отсчёт слева направо и сверху вниз.
    std::sort(XRays.begin(), XRays.end());
    std::sort(YRays.begin(), YRays.end());

    //if ( XRaysCount%2!=0 ) { XRays.push_back(XRays[XRaysCount-1]+2); XRaysCount++; }
    //if ( YRaysCount%2!=0 ) { YRays.push_back(YRays[YRaysCount-1]+2); YRaysCount++; }

    std::vector<cv::Rect> subRects;
    for (yi=1; yi<YRaysCount; ++yi) {
        int top = YRays[yi-1];
        int bot = YRays[yi];
        for (xi=1; xi<XRaysCount; ++xi) {
            int left = XRays[xi-1];
            int right = XRays[xi];
            subRects.push_back(cv::Rect(left, top, right-left, bot-top));
        }
    }


    //-- Делаем триангуляцию полигона, что бы могли побыстраляну избавиться от прямоугольников вне полигона
    std::vector<std::vector<Geometry::Point>> ecCpolygon = {polygon};
    std::vector<uint32_t> trianglesIndices = mapbox::earcut<uint32_t>(ecCpolygon);

    for (size_t i=0; i<trianglesIndices.size(); i+=3) {

        const Geometry::Point &p1 = polygon[trianglesIndices[i]];
        const Geometry::Point &p2 = polygon[trianglesIndices[i+1]];
        const Geometry::Point &p3 = polygon[trianglesIndices[i+2]];

        cv::line(surface, p1, p2, cv::Scalar(127,0, 250), 1);
        cv::line(surface, p2, p3, cv::Scalar(127,0, 250), 1);
        cv::line(surface, p3, p1, cv::Scalar(127,0, 250), 1);
    }



    std::vector<int64_t> subRectsAreas;
    int subRectsInsideCount = 0;

    for (size_t ri=0; ri<subRects.size(); ++ri) {

        const cv::Rect &rect = subRects.at(ri);
        bool rp1=false, rp2=false, rp3=false, rp4=false;

        for (size_t ti=0; ti<trianglesIndices.size(); ti+=3) { //-- Все 4 точки прямоугольника должны принадлежать какому-либо треугольнику

            const Geometry::Point &tp1 = polygon[trianglesIndices[ti]];
            const Geometry::Point &tp2 = polygon[trianglesIndices[ti+1]];
            const Geometry::Point &tp3 = polygon[trianglesIndices[ti+2]];

            if ( !rp1 && pointInTriangle(tp1, tp2, tp3, rect.tl()) ) { rp1 = true; }
            if ( !rp2 && pointInTriangle(tp1, tp2, tp3, rect.tl()+cv::Point(rect.width, 0)) ) { rp2 = true; }
            if ( !rp3 && pointInTriangle(tp1, tp2, tp3, rect.br()) ) { rp3 = true; }
            if ( !rp4 && pointInTriangle(tp1, tp2, tp3, rect.br()-cv::Point(rect.width, 0)) ) { rp4 = true; }

            if ( rp1 && rp2 && rp3 && rp4 ) { break; }
        }


        bool isInside = (rp1 && rp2 && rp3 && rp4);
        subRectsAreas.push_back((isInside)? rect.area() : INT_MIN);
        if ( isInside ) { subRectsInsideCount++; }
    }

    //qDebug()<<"Sub rects inside count:"<<subRectsInsideCount;

    //-- Нарисуем все, которые внутри
    for (size_t ri=0; ri<subRects.size(); ++ri) {
        if ( subRectsAreas[ri]>0 ) {
            const cv::Rect &rect = subRects.at(ri);
            cv::rectangle(surface, rect, cv::Scalar(255,255,0), 1);
        }
    }


    int rows = YRaysCount-1, cols = XRaysCount-1;

    cv::Rect maxRectIDs = findMaxSum(&subRectsAreas[0], rows, cols);
    int tl = maxRectIDs.y*cols+maxRectIDs.x;
    int br = tl+ maxRectIDs.height*cols+maxRectIDs.width;

    cv::Rect resRect = cv::Rect(subRects[tl].tl(), subRects[br].br());

    return resRect;
}


void onMouseClick(int event, int x, int y, int flags, void* userdata)
{

    if  ( event==cv::EVENT_LBUTTONDOWN ) {
        _polygon.push_back(cv::Point(x, y));
    }

}


int main(int argc, char *argv[])
{
    cv::namedWindow(_wndTitle);
    cv::setMouseCallback(_wndTitle, onMouseClick, NULL);

    int iteratCount = 0;
    cv::createTrackbar("Iterat", _wndTitle, &iteratCount, 20);


    cv::Scalar WHITE(255,255,255);
    cv::Scalar BLACK(0,0,0);
    cv::Scalar RED(0,0,255);
    cv::Scalar GREEN(0,255,0);
    cv::Scalar BLUE(255,0,0);
    cv::Scalar YELLOW(0,200,200);
    cv::Scalar ORANGE(0,150,255);


    cv::Mat surface(768, 1024, CV_8UC3);


    cv::Rect subrectanglesMethodRect;

    //_polygon={{155,591},{68,100},{949,31},{819,650}};
    _polygon = {{293,456},{166,308},{471,145},{471,145},{691,206},{817,567},{650,746},{285,728},{73,648},{109,474}};
    //_polygon = {{538,384},{665,293},{452,154},{207,153},{141,387},{411,432},{318,712},{665,721},{939,473}};

    while ( cv::getWindowProperty(_wndTitle, cv::WND_PROP_VISIBLE ) ) {

        surface = WHITE;

        //-- Draw anchor points and edges
        for (size_t i=1; i<_polygon.size()+1; ++i) {
            cv::Point prevPoint = _polygon[i-1];
            cv::Point curPoint = (i==_polygon.size())? _polygon[0] : _polygon[i];
            cv::line(surface, prevPoint, curPoint, RED, 3);
            cv::circle(surface, prevPoint, 8, ((i==1)? GREEN : BLACK), cv::FILLED);
        }

        subrectanglesMethodRect = findMaxRectSubRectangles(_polygon, surface, iteratCount);

        cv::rectangle(surface, subrectanglesMethodRect, ORANGE, 5);

        cv::imshow(_wndTitle, surface);

        int keyPressed = cv::waitKey(10);

        if ( keyPressed==27 ) {
            _polygon.clear();            
            subrectanglesMethodRect = cv::Rect(0,0,0,0);
        } else
        if ( keyPressed==13 ) {
            //subrectanglesMethodRect = findMaxRectSubRectangles(_polygon);
        } else
        if ( keyPressed==115 ) { //-- Save

            std::cout<<"_polygon = {";
            for (size_t i=0; i<_polygon.size(); ++i) {
                std::cout<<"{"<<_polygon[i].x<<","<<_polygon[i].y<<"}";
                if ( i<_polygon.size()-1 ) { std::cout<<","; }
            }
            std::cout<<"};"<<std::endl;

        }

    }



    return 0;
}
