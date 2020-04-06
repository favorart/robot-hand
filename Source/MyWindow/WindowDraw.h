#include "StdAfx.h"

#ifndef  _DRAW_H_
#define  _DRAW_H_

#include "WindowHeader.h"
#include "Robo.h"
//------------------------------------------------------------------------------
#ifdef MY_WINDOW
void drawDecardsCoordinates(HDC hdc);
void drawCoordinates(HDC hdc, bool show_marks);

extern const HPEN defaultPen;
//------------------------------------------------------------------------------
void drawTrajectory(HDC hdc, const Robo::Trajectory &trajectory, HPEN hPen= defaultPen);

//------------------------------------------------------------------------------
void drawStateTrajectory(HDC hdc, const Robo::StateTrajectory &trajectory, HPEN hPen = defaultPen);

//------------------------------------------------------------------------------
void drawLine(HDC hdc, const Point &s, const Point &e, HPEN hPen=defaultPen);

//------------------------------------------------------------------------------
void drawCross(HDC hdc, const Point &center, double radius, HPEN hPen=defaultPen);

//------------------------------------------------------------------------------
void drawCircle(HDC hdc, const Point &center, double radius, HPEN hPen=defaultPen);

//------------------------------------------------------------------------------
enum class MyFigure : uint8_t { EmptyFig = 0, Ellipse = 1, Rectangle, Triangle, Polygon, _Last_ };
void drawMyFigure(HDC hdc, const Point &center, double w, double h, double angle, MyFigure figure, HPEN hPen=defaultPen);

//------------------------------------------------------------------------------
template <template <typename, typename> class Container,
    typename Value = Point,
    typename Allocator = std::allocator<Point> >
void drawMyFigure(HDC hdc, const Container<Value, Allocator> &container, MyFigure figure, HPEN hPen=defaultPen)
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
using hpens_gradient_t = std::vector<HPEN>;

void makeGradient(color_interval_t colors, size_t color_gradations, color_gradient_t &gradient);
//------------------------------------------------------------------------------
class GradPens : public std::function<HPEN(size_t)>
{
    const Robo::frames_t _maxLasts;
    size_t _colorGradations = 15;
    color_interval_t _colors{ RGB(150, 10, 245), RGB(245, 10, 150) };
    hpens_gradient_t _gradientPens;
public:
    GradPens(Robo::frames_t maxLasts) : _maxLasts(maxLasts) { restoreGradient(); }
    GradPens(const GradPens&) = delete;
    GradPens(GradPens&&) = default;
    HPEN operator()(Robo::frames_t longz) const;
    void shuffleGradient();
    void restoreGradient();
    void setColors(color_interval_t colors, size_t gradations = 15);
    ~GradPens()
    {
        for (HPEN pen : _gradientPens)
            DeleteObject(pen);
    }
};
//------------------------------------------------------------------------------
#endif //MY_WINDOW

#endif // _DRAW_H_
