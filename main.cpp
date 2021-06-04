#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

#include <iostream>

#include "inscribedmaxarearect.h"

#include <QDebug>


std::string _wndTitle = "MaxAreaRect";
std::vector<cv::Point> _polygon;
bool _needRepaint;

cv::Scalar WHITE(255,255,255);
cv::Scalar BLACK(0,0,0);
cv::Scalar GRAY(150,150,150);
cv::Scalar LGRAY(220,220,220);
cv::Scalar RED(0,0,255);
cv::Scalar GREEN(0,255,0);
cv::Scalar BLUE(255,0,0);
cv::Scalar TURQUOISE(255,255,0);
cv::Scalar PINK(127,0, 250);
cv::Scalar YELLOW(0,200,200);
cv::Scalar ORANGE(0,150,255);


void onMouseClick(int event, int x, int y, int, void*)
{
    if  ( event==cv::EVENT_LBUTTONDOWN ) {
        _polygon.push_back(cv::Point(x, y));
        _needRepaint = true;
    }
}

void onCBBtnClick(int state, void* userData)
{
    *static_cast<bool*>(userData) = state;
    _needRepaint = true;
}

int main(int argc, char *argv[])
{
    Q_UNUSED(argc); Q_UNUSED(argv);

    bool showAnchorPointRays = true, showEdgeIntersectRays = true, showPolygonTriangulation = true, showInscribedSubRectangles = true, showHint = true;

    cv::Size wndSize(1024, 768);

    cv::namedWindow(_wndTitle, cv::WINDOW_KEEPRATIO);
    cv::resizeWindow(_wndTitle, wndSize);
    cv::setMouseCallback(_wndTitle, onMouseClick, NULL);

    cv::createButton("Show anchor point rays", onCBBtnClick, &showAnchorPointRays, cv::QT_CHECKBOX, showAnchorPointRays);
    cv::createButton("Show edge intersect rays", onCBBtnClick, &showEdgeIntersectRays, cv::QT_CHECKBOX, showEdgeIntersectRays);
    cv::createButton("Show polygon triangulation", onCBBtnClick, &showPolygonTriangulation, cv::QT_CHECKBOX, showPolygonTriangulation);
    cv::createButton("Show inscribed sub rectangles", onCBBtnClick, &showInscribedSubRectangles, cv::QT_CHECKBOX, showInscribedSubRectangles);
    cv::createButton("Show hint", onCBBtnClick, &showHint, cv::QT_CHECKBOX, showHint);

    cv::Mat surface(wndSize.height, wndSize.width, CV_8UC3);
    _needRepaint = true;

    _polygon = {{191,210},{75,556},{432,634},{895,551}};

    InscribedMaxAreaRect IMAR;

    while ( cv::getWindowProperty(_wndTitle, cv::WND_PROP_VISIBLE ) ) {


        int keyPressed = cv::waitKey(10);

        if ( keyPressed==27 ) { //-- Esc
            _polygon.clear();
            _needRepaint = true;
        } else
        if ( keyPressed==13 ) { //-- Enter
            _needRepaint = true;
        } else
        if ( keyPressed==115 ) { //-- Save

            std::cout<<"_polygon = {";
            for (size_t i=0; i<_polygon.size(); ++i) {
                std::cout<<"{"<<_polygon[i].x<<","<<_polygon[i].y<<"}";
                if ( i<_polygon.size()-1 ) { std::cout<<","; }
            }
            std::cout<<"};"<<std::endl;

            _needRepaint = true;
        }



        if ( _needRepaint ) {

            surface = WHITE;

            if ( showHint ) {
                cv::putText(surface, "Press CTRL+P for display options.", cv::Point(5,20), cv::FONT_HERSHEY_COMPLEX, 0.5, LGRAY);
                cv::putText(surface, "Press ESC for clear.", cv::Point(5,40), cv::FONT_HERSHEY_COMPLEX, 0.5, LGRAY);
                cv::putText(surface, "Press Mouse LB for draw new point.", cv::Point(5,60), cv::FONT_HERSHEY_COMPLEX, 0.5, LGRAY);
                cv::putText(surface, "Press S for save current polygon (console output).", cv::Point(5,80), cv::FONT_HERSHEY_COMPLEX, 0.5, LGRAY);
            }

            //=============================================
            std::vector<Geometry::Point> polygonForFind;
            for (size_t i=0; i<_polygon.size(); ++i) {
                const cv::Point &p = _polygon.at(i);
                polygonForFind.push_back(Geometry::Point(p.x, p.y));
            }

            IMAR.setPolygon(polygonForFind);

            if ( IMAR.findMaxAreaInscribedRect() ) {

                Geometry::Rect boundingBox = IMAR.boundingBox();

                if ( showEdgeIntersectRays ) {
                    std::vector<int> raysX = IMAR.raysXEdges(), raysY = IMAR.raysYEdges();
                    for (size_t xi=0;  xi<raysX.size(); ++xi) { cv::line(surface, cv::Point(raysX[xi], boundingBox.t), cv::Point(raysX[xi], boundingBox.b), GRAY, 1); }
                    for (size_t yi=0;  yi<raysY.size(); ++yi) { cv::line(surface, cv::Point(boundingBox.l, raysY[yi]), cv::Point(boundingBox.r, raysY[yi]), GRAY, 1); }

                    std::vector<Geometry::Point> edgeIntersectsPoints = IMAR.edgesIntersetcPoints();
                    for (size_t ei=0; ei<edgeIntersectsPoints.size(); ++ei) { const Geometry::Point &p = edgeIntersectsPoints.at(ei); cv::circle(surface, cv::Point(p.x, p.y), 5, BLACK, cv::FILLED); }
                }

                if ( showAnchorPointRays ) {
                    std::vector<int> raysX = IMAR.raysXAnchors(), raysY = IMAR.raysYAnchors();
                    for (size_t xi=0;  xi<raysX.size(); ++xi) { cv::line(surface, cv::Point(raysX[xi], boundingBox.t), cv::Point(raysX[xi], boundingBox.b), BLACK, 1); }
                    for (size_t yi=0;  yi<raysY.size(); ++yi) { cv::line(surface, cv::Point(boundingBox.l, raysY[yi]), cv::Point(boundingBox.r, raysY[yi]), BLACK, 1); }
                }

                if ( showPolygonTriangulation ) {
                    std::vector<uint32_t> trianglesIndices = IMAR.trianglesIndices();
                    for (size_t i=0; i<trianglesIndices.size(); i+=3) {
                        const cv::Point &p1 = _polygon[trianglesIndices[i+0]], &p2 = _polygon[trianglesIndices[i+1]], &p3 = _polygon[trianglesIndices[i+2]];
                        cv::line(surface, p1, p2, PINK, 1);
                        cv::line(surface, p2, p3, PINK, 1);
                        cv::line(surface, p3, p1, PINK, 1);
                    }
                }

                if ( showInscribedSubRectangles ) {
                    std::vector<Geometry::Rect> subRects = IMAR.subRects();
                    std::vector<double> subRectsAreas = IMAR.subRectsAreasInscribed();

                    for (size_t ri=0; ri<subRects.size(); ++ri) {
                        if ( subRectsAreas[ri]>0 ) {
                            const Geometry::Rect &rect = subRects.at(ri);
                            cv::rectangle(surface, cv::Rect(rect.l, rect.t, rect.width(), rect.height()), TURQUOISE, 1);
                        }
                    }
                }

                double polygonArea = IMAR.polygonArea(), inscribedRectArea = IMAR.inscribedMaxAreaRect().area();
                uint64_t findElapsedMcS = IMAR.findElapsedMcS();
                char areasText[100];
                std::snprintf(areasText, sizeof(areasText), "Polygon area: %.1f, rect area: %.1f (%.1f%%). Elapsed: %lu mcs.", polygonArea, inscribedRectArea, (inscribedRectArea*100.0f)/polygonArea, findElapsedMcS);
                cv::putText(surface, std::string(areasText), cv::Point(5, surface.rows-10), cv::FONT_HERSHEY_COMPLEX, 0.5, BLACK);
            }

            //-- Draw anchor points and edges of polygon
            for (size_t i=1; i<_polygon.size()+1; ++i) {
                const cv::Point &prevPoint = _polygon[i-1];
                const cv::Point &curPoint = (i==_polygon.size())? _polygon[0] : _polygon[i];
                cv::line(surface, prevPoint, curPoint, RED, 3);
                cv::circle(surface, prevPoint, 8, ((i==1)? GREEN : BLACK), cv::FILLED);
            }

            //-- Result the max area inscribed rectangle
            Geometry::Rect maxAreaInscribedRed = IMAR.inscribedMaxAreaRect();
            cv::rectangle(surface, cv::Rect(maxAreaInscribedRed.l, maxAreaInscribedRed.t, maxAreaInscribedRed.width(), maxAreaInscribedRed.height()), ORANGE, 5);

            cv::imshow(_wndTitle, surface);

            _needRepaint = false;
        }

    }

    return 0;
}
