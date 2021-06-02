#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

#include <iostream>

#include "geometry.h"

#include "triangulation.h"


std::string _wndTitle = "MaxAreaRect";
std::vector<Geometry::Point> _polygon;


#include <QDebug>



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

        qDebug()<<"ITER"<<xi<<yi<<XRaysCount<<YRaysCount;

        if ( xi<XRaysCount ) { xi++; }
        if ( yi<YRaysCount ) { yi++; }

        iter++;
        //if ( iter>maxIters ) { qWarning()<<"OVER ITERATIONS!!!!"; break; }
    }


    //-- Покажем получившиеся лучи
    for (size_t xi=0;  xi<XRays.size(); ++xi) { cv::line(surface, cv::Point(XRays[xi], FT), cv::Point(XRays[xi], FB), cv::Scalar(255,255,0), 1, cv::LINE_AA); }
    for (size_t yi=0;  yi<YRays.size(); ++yi) { cv::line(surface, cv::Point(FL, YRays[yi]), cv::Point(FR, YRays[yi]), cv::Scalar(255,255,0), 1, cv::LINE_AA); }



    //-- Создаём прямоугольники. Отсчёт слева направо и сверху вниз.
    std::sort(XRays.begin(), XRays.end());
    std::sort(YRays.begin(), YRays.end());

    //if ( YRaysCount%2!=0 ) { YRays.push_back(YRays[YRaysCount-1]+1); YRaysCount++; }
    //if ( XRaysCount%2!=0 ) { XRays.push_back(XRays[XRaysCount-1]+1); XRaysCount++; }


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


    qDebug()<<"SUB RECTS COUNT"<<subRects.size();

    //-- Делаем триангуляцию полигона, что бы могли побыстраляну избавиться от прямоуголдьников вне полигона

    Geometry::Triangulation trinagulator(polygon);

    std::vector<Geometry::Triangle> triangles =  trinagulator.triangulation();

    qDebug()<<"TRIANGLES COUNT"<<triangles.size();

    for (size_t ti=0; ti<std::min(maxIters,triangles.size()); ++ti) {
        const Geometry::Triangle &tr = triangles[ti];

        cv::line(surface, tr.p1, tr.p2, cv::Scalar(127,0, 127), 2, cv::LINE_AA);
        cv::line(surface, tr.p1, tr.p3, cv::Scalar(127,0, 127), 2, cv::LINE_AA);
        cv::line(surface, tr.p3, tr.p1, cv::Scalar(127,0, 127), 2, cv::LINE_AA);
    }


    //-- Нужно избавиться от тех, что вне контура частично или полностью
    for (size_t ri=0; ri<subRects.size(); ++ri) {


    }

    cv::RNG rng(12345);
    for (size_t ri=0; ri<subRects.size(); ++ri) {
        const cv::Rect &rect = subRects.at(ri);
        cv::Scalar color = cv::Scalar(rng.uniform(10,255), rng.uniform(10, 255), rng.uniform(10, 255));
        //cv::rectangle(surface, rect, color, cv::FILLED);
    }





    return cv::Rect(0,0,0,0);
}


void onMouseClick(int event, int x, int y, int flags, void* userdata)
{

    if  ( event==cv::EVENT_LBUTTONDOWN ) {
        _polygon.push_back(cv::Point(x, y));
    }

}


#include "vectorcircular.h"
void debugVector(QString title, const VectorCircular<int> &v)
{
    QStringList vl;
    for(size_t i=0; i<v.size(); ++i) {
        vl.append(QString::number(v[i]));
    }
    qDebug()<<title<<vl.join(" ");

}


int main(int argc, char *argv[])
{
    VectorCircular<int> v;

    v.push_back(0);
    v.push_back(1);
    v.push_back(2);
    v.push_back(3);
    v.push_back(4);
    v.push_back(5);

    debugVector("push_back", v);


    VectorCircular<int>::Iterator i = v.begin();

    for (i=v.begin()+3; i!=v.begin(); ++i) {
        qDebug()<<"ITER 2:"<<*i<<"idx"<<i.key();
    }


    qDebug()<<"FORWARD";
    for (i=v.begin(); i!=v.end(); ++i) {
        qDebug()<<"ITER:"<<*i<<"idx"<<i.key();
    }


    qDebug()<<"LAST FOR ITERR KEY"<<i.key()<<*i<<"IDX"<<i.idx();
    ++i;
    qDebug()<<"LAST INC ITERR KEY"<<i.key()<<*i<<"IDX"<<i.idx();


    VectorCircular<int>::Iterator i4 = i+4, i5 = i+5, i6 = i+6;


    qDebug()<<"INC 4 VAL 5 NNN"<<*i4;
    qDebug()<<"INC 5 REAL 0 NNN"<<*i5;
    qDebug()<<"INC 6 REAL 1 NNN"<<*i6;


    qDebug()<<"REMOVE: "<<*i5<<i5.key();

    VectorCircular<int>::Iterator remIt = v.erase(i5);

    qDebug()<<"VAL 4 REAL 1 REM"<<*i4;
    qDebug()<<"VAL 6 REAL 2 REM"<<*i6;


    for (i=v.begin(); i!=v.end(); ++i) {
        qDebug()<<"ITER"<<*i<<"idx"<<i.key();
    }


    qDebug()<<"------------------------------------------------";
    VectorCircular<int>::Iterator it = v.begin();

    for (int i=0; i<20; ++i) {
        qDebug()<<"Need"<<(i%v.size())<<"get"<<(*(it+i))-1;
    }


    return 0;

    cv::namedWindow(_wndTitle, cv::WINDOW_FREERATIO);
    cv::setMouseCallback(_wndTitle, onMouseClick, NULL);

    int iteratCount = 40;
    cv::createTrackbar("Iterat", _wndTitle, &iteratCount, 1020);


    cv::Scalar WHITE(255,255,255);
    cv::Scalar BLACK(0,0,0);
    cv::Scalar RED(0,0,255);
    cv::Scalar GREEN(0,255,0);
    cv::Scalar BLUE(255,0,0);
    cv::Scalar YELLOW(0,200,200);
    cv::Scalar ORANGE(0,165,255);


    cv::Mat surface(768, 1024, CV_8UC3);


    cv::Rect brutforceMethodRect;
    cv::Rect subrectanglesMethodRect;

    //_polygon={{155,591},{68,100},{949,31},{819,650}};
    //_polygon = {{293,456},{166,308},{471,145},{471,145},{691,206},{817,567},{650,746},{285,728},{73,648},{109,474}};
   // _polygon = {{538,384},{665,293},{452,154},{207,153},{141,387},{411,432},{318,712},{665,721},{939,473}};

    while ( cv::getWindowProperty(_wndTitle, cv::WND_PROP_VISIBLE ) ) {

        surface = WHITE;

        //-- Draw anchor points and edges
        for (size_t i=1; i<_polygon.size()+1; ++i) {
            cv::Point prevPoint = _polygon[i-1];
            cv::Point curPoint = (i==_polygon.size())? _polygon[0] : _polygon[i];
            cv::line(surface, prevPoint, curPoint, RED, 3, cv::LINE_AA);
            cv::circle(surface, prevPoint, 10, ((i==1)? GREEN : BLUE), cv::FILLED);
        }


        findMaxRectSubRectangles(_polygon, surface, iteratCount);

        cv::rectangle(surface, brutforceMethodRect, YELLOW, 2);
        cv::rectangle(surface, subrectanglesMethodRect, ORANGE, 2);

        cv::imshow(_wndTitle, surface);

        int keyPressed = cv::waitKey(0);

        if ( keyPressed==27 ) {
            _polygon.clear();
            brutforceMethodRect = cv::Rect(0,0,0,0);
            subrectanglesMethodRect = cv::Rect(0,0,0,0);
        } else
        if ( keyPressed==13 ) {
            //brutforceMethodRect = findMaxRectBruteforce(_polygon);
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


        qDebug()<<"FFF"<<keyPressed;


    }



    return 0;
}
