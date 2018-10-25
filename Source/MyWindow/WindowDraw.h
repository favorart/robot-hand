#include "StdAfx.h"

#ifndef  _DRAW_H_
#define  _DRAW_H_

#include "WindowHeader.h"
#include "Robo.h"
//------------------------------------------------------------------------------
void drawDecardsCoordinates(HDC hdc);
void drawCoordinates(HDC hdc, bool show_marks);

//------------------------------------------------------------------------------
void drawTrajectory(HDC hdc, const Robo::Trajectory &trajectory, HPEN hPen);
inline void drawTrajectory(HDC hdc, const Robo::Trajectory &trajectory)
{ drawTrajectory(hdc, trajectory, (HPEN)GetStockObject(BLACK_PEN)); }

//------------------------------------------------------------------------------
void drawLine(HDC hdc, const Point &s, const Point &e, HPEN hPen);
inline void drawLine(HDC hdc, const Point &s, const Point &e)
{ drawLine(hdc, s, e, (HPEN)GetStockObject(BLACK_PEN)); }

//------------------------------------------------------------------------------
void  drawCross(HDC hdc, const Point &center, double radius, HPEN hPen);
inline void drawLine(HDC hdc, const Point &center, double radius)
{ drawCross(hdc, center, radius, (HPEN)GetStockObject(BLACK_PEN)); }

//------------------------------------------------------------------------------
void drawCircle(HDC hdc, const Point &center, double radius, HPEN hPen);
inline void drawCircle(HDC hdc, const Point &center, double radius)
{ drawCircle(hdc, center, radius, (HPEN)GetStockObject(BLACK_PEN)); }

//------------------------------------------------------------------------------
enum class MyFigure : uint8_t { EmptyFig = 0, Ellipse = 1, Rectangle, Triangle, Polygon, _Last_ };
void drawMyFigure(HDC hdc, const Point &center, double w, double h, double angle, MyFigure figure, HPEN hPen);

//------------------------------------------------------------------------------
template <template <typename, typename> class Container,
    typename Value = Point,
    typename Allocator = std::allocator<Point> >
void drawMyFigure(HDC hdc, const Container<Value, Allocator> &container, MyFigure figure, HPEN hPen)
{
    if (figure >= MyFigure::_Last_)
        throw std::logic_error("drawMyFigure: Invalid figure=" + std::to_string(int(figure)));
    if (figure == MyFigure::EmptyFig)
        return;

    HPEN hPen_old = (HPEN)SelectObject(hdc, hPen);
    //-----------------------------------
    MoveToEx(hdc, Tx(container.back().x), Ty(container.back().y), NULL);
    for (auto &p : container)
        LineTo(hdc, Tx(p.x), Ty(p.y));
    //-----------------------------------
    SelectObject(hdc, hPen_old);
}

//------------------------------------------------------------------------------
using color_interval_t = std::pair<COLORREF, COLORREF>;
using color_gradient_t = std::vector<COLORREF>;

void makeGradient(color_interval_t colors, size_t color_gradations, color_gradient_t &gradient);
//------------------------------------------------------------------------------
class GradPens : public std::function<HPEN(size_t)>
{
    const Robo::frames_t _robo_max_last;
    size_t _colorGradations = 15;
    color_interval_t _colors{ RGB(150, 10, 245), RGB(245, 10, 150) };
    //{ RGB(0,0,130), RGB(255,0,0) } // 128
    //{ RGB(130,0,0), RGB(255,155,155) }
    //gradient_t gradient({ RGB(25, 255, 25), RGB(25, 25, 255), RGB(255, 25, 25) });
    color_gradient_t _gradient;
    std::vector<HPEN> _gradientPens;
public:
    GradPens(Robo::frames_t robo_max_last);
    HPEN operator()(Robo::frames_t longs) const;
    void setColors(color_interval_t colors, size_t gradations = 15);
    ~GradPens()
    {
        for (HPEN pen : _gradientPens)
            DeleteObject(pen);
    }
};
//------------------------------------------------------------------------------
//template <Var ...>
inline GradPens makeGrad(CGradient cg/*, ...*/)
{
    //GradPens(Robo::frames_t robo_max_last);
    switch (cg)
    {
    case CGradient::None:
        break;
    case CGradient::Longz:
        break;
    case CGradient::Lasts:
        break;
    case CGradient::Strats:
        break;
    }
    return GradPens{ 0 };
}
//------------------------------------------------------------------------------
#endif // _DRAW_H_
