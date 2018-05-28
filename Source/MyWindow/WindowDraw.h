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
void drawCircle(HDC hdc, const Point &center, double radius, HPEN hPen);
inline void drawCircle(HDC hdc, const Point &center, double radius)
{ drawCircle(hdc, center, radius, (HPEN)GetStockObject(BLACK_PEN)); }

//------------------------------------------------------------------------------
enum class MyFigure : uint8_t { EmptyFig = 0, Ellipse = 1, Rectangle = 2 };
void drawMyFigure(HDC hdc, const Point &center, double w, double h, double angle, MyFigure figure, HPEN hPen);

//------------------------------------------------------------------------------
using color_interval_t = std::pair<COLORREF, COLORREF>;
using color_gradient_t = std::vector<COLORREF>;

void makeGradient(color_interval_t colors, size_t color_gradations, color_gradient_t &gradient);
//------------------------------------------------------------------------------
class GradPens : public std::function<HPEN(size_t)>
{
    const Robo::frames_t robo_max_last;
    const size_t colorGradations = 15;
    const color_interval_t colors{ RGB(150, 10, 245), RGB(245, 10, 150) };
    //{ RGB(0,0,130), RGB(255,0,0) } // 128
    //{ RGB(130,0,0), RGB(255,155,155) }
    //gradient_t gradient({ RGB(25, 255, 25), RGB(25, 25, 255), RGB(255, 25, 25) });
    color_gradient_t gradient;
    std::vector<HPEN> gradientPens;
public:
    GradPens(Robo::frames_t robo_max_last) : robo_max_last(robo_max_last)
    {
        makeGradient(colors, colorGradations, gradient);

        gradientPens.resize(gradient.size());
        for (auto i = 0U; i < gradient.size(); ++i)
            gradientPens[i] = CreatePen(PS_SOLID, 1, gradient[i]);
    }
    HPEN operator()(size_t longs) /*const*/
    {
        return gradientPens[Utils::interval_map(longs, { 0u, robo_max_last }, { 0u, gradientPens.size() })];
    }
    ~GradPens()
    {
        for (auto pen : gradientPens)
            DeleteObject(pen);
    }
};
//------------------------------------------------------------------------------
#endif // _DRAW_H_
