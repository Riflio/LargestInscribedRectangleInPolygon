#include "triangulation.h"


using namespace Geometry;


Triangulation::Triangulation()
{

}

Triangulation::Triangulation(const std::vector<Point> &listPoints)
{
    lp.reserve(listPoints.size()+1);
    for (size_t i=0; i<listPoints.size(); ++i) { lp.push_back(listPoints[i]); }
}


/**
* @brief Функция определяет принадлежность точки point треугольнику t1_t2_t3
* @param point
* @param t1
* @param t2
* @param t3
* @return
*/
bool Triangulation::isInTriangle(const Point &point, const Point &t1, const Point &t2, const Point &t3)
{
    int a = vp(point, t1, t2);
    int b = vp(point, t2, t3);
    int c = vp(point, t3, t1);

    //-- Если все три двойки векторов однонаправленные, то point внутри треугольника
    return ( (a<0 && b<0 && c< 0) || (a>0 && b>0 && c>0) );
}

/**
* @brief Векторное произведение
* @param a
* @param b
* @param c
* @return
*/
int Triangulation::vp(const Point &a, const Point &b, const Point &c)
{
    return (a.x*b.y - a.x*c.y - b.x*a.y + b.x*c.y + c.x*a.y - c.x*b.y );
}

/**
* @brief Скалярное произведение
* @param a
* @param b
* @param c
* @return
*/
int Triangulation::scp(const Point &a, const Point &b, const Point &c)
{
    return ( (c.x-b.x)*(a.x-b.x) + (c.y-b.y)*(a.y-b.y) );
}

/**
* @brief Определяем направление обхода многоугольника
*/
void Triangulation::determineDirection()
{
    int maxx = 0;
    TPointsList::Iterator np, mp = lp.end();

    //-- Находим точку с максимальным значением по Х
    for (np=lp.begin(); np!=lp.end(); ++np) {
        if ( maxx<np->x ) { maxx=np->x; mp = np; }
    }

    Point p1 = *(mp+0);
    Point p2 = *(mp+1);
    Point p3 = *(mp+2);

    //с помощью векторного произведения определяем направление тройки векторов
    direction = (vp(p1, p2, p3)>0)? 1 : -1;
}


/**
* @brief Триангулируем
*
*/
std::vector<Triangle> Triangulation::triangulation()
{
    bool newTriangle;

    std::vector<Triangle> vn;

    //-- Первые 3 вершины
    TPointsList::Iterator pn1 = lp.begin(), pn2 = lp.begin()+1, pn3 = lp.begin()+2, it1;

    determineDirection();

    qDebug()<<"FFFFFFFFFFFFFFFFFFFFFFFF";

    while ( lp.size()!=3 ) {
        //qDebug()<<"AZAZA";
        if ( vp(*pn2, *pn1, *pn3)*direction<0 ) { //-- Если вектора образуют правую тройку
            newTriangle = true;
            //-- Проверка не попала ли вершина в отсекаемый треугольник
            for (it1=pn3+1; it1!=pn1; ++it1) {
                //qDebug()<<"IT1: "<<it1.key()<<"pn3: "<<(pn3+1).key()<<"pn1: "<<pn1.key();
                if( isInTriangle(*it1, *pn1, *pn2, *pn3) ) {
                    newTriangle = false; //-- Вершина попала в отсекаемый треугольник
                    ++pn1; //-- Переходим к следующей вершине
                    break;
                }
            }
            if ( newTriangle ) { //-- Вершина не попала в отсекаемый треугольник. Заносим в vector новый треугольник.
                vn.push_back(Triangle(*pn1, *pn2, *pn3));                
                lp.erase(pn2); //-- Удаляем вершину из списка
                //qDebug()<<"ERASE";
            }
            //-- Сдвигаем 2ю и 3ю вершины
            pn2 = pn3;
            pn3 = pn3+1;
        } else { //-- Вектора не образуют правую тройку, сдвигаем 3 вершины
            pn1 = pn2;
            pn2 = pn3;
            pn3 = pn3+1;
        }
    }


    qDebug()<<"PZDC";
    //-- Заносим в vector оставшийся треугольник
    //vn.push_back(Triangle(*(lp.begin()+0), *(lp.begin()+1), *(lp.begin()+2)));

    return vn;
}
