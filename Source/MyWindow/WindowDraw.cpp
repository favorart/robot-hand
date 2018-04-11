#include "StdAfx.h"
#include "WindowDraw.h"

//------------------------------------------------------------------------------
void  drawDecardsCoordinates(HDC hdc)
{
    uint_t i = 90U;
    //---Ox---
    MoveToEx (hdc, Tx(-1.000), Ty( 0.000), NULL);
    LineTo   (hdc, Tx( 1.000), Ty( 0.000));
    LineTo   (hdc, Tx( 0.970), Ty( 0.010));
    LineTo   (hdc, Tx( 0.990), Ty( 0.000));
    LineTo   (hdc, Tx( 0.970), Ty(-0.010));
    LineTo   (hdc, Tx( 1.000), Ty( 0.000));
    //---Oy---              
    MoveToEx (hdc, Tx( 0.000), Ty(-1.000), NULL);
    LineTo   (hdc, Tx( 0.000), Ty( 1.000));
    LineTo   (hdc, Tx(-0.005), Ty( 0.960));
    LineTo   (hdc, Tx( 0.000), Ty( 0.990));
    LineTo   (hdc, Tx( 0.008), Ty( 0.960));
    LineTo   (hdc, Tx( 0.000), Ty( 1.000));
    //--------
    for (i = 0; i < 19; i++)
    {
        MoveToEx (hdc, Tx(-0.90 + 0.1*i), Ty(-0.01), NULL);
        LineTo   (hdc, Tx(-0.90 + 0.1*i), Ty( 0.01));
        MoveToEx (hdc, Tx(-0.01), Ty(-0.90 + 0.1*i), NULL);
        LineTo   (hdc, Tx( 0.01), Ty(-0.90 + 0.1*i));
    }
}
//------------------------------------------------------------------------------
void  drawLine(HDC hdc, const Point &s, const Point &e, HPEN hPen)
{
    HPEN hPen_old = (HPEN)SelectObject(hdc, hPen);
    //-----------------------------------
    MoveToEx(hdc, Tx(s.x), Ty(s.y), NULL);
    LineTo  (hdc, Tx(e.x), Ty(e.y));
    //-----------------------------------
    SelectObject(hdc, hPen_old);
}
//------------------------------------------------------------------------------
//typedef std::list<std::shared_ptr<Point>>  trajectory_refs_t;
//void  DrawTrajectory(HDC hdc, const trajectory_refs_t &trajectory, HPEN hPen)
//{
//    if (!trajectory.empty())
//    {
//        HPEN hPen_old = (HPEN)SelectObject(hdc, hPen);
//        //------------------------------------------------------------------
//        const auto &p = trajectory.front();
//        MoveToEx(hdc, Tx(p->x), Ty(p->y), NULL);
//        for (const auto &p : trajectory)
//        { LineTo(hdc, Tx(p->x), Ty(p->y)); }
//        //------------------------------------------------------------------
//        // отменяем ручку
//        SelectObject(hdc, hPen_old);
//    }
//}
//------------------------------------------------------------------------------
void  drawTrajectory(HDC hdc, const Robo::Trajectory &trajectory, HPEN hPen)
{
    if (!trajectory.empty())
    {
        HPEN hPen_old = (HPEN)SelectObject(hdc, hPen);
        //------------------------------------------------------------------
        const Point &p = trajectory.front();
        MoveToEx(hdc, Tx(p.x), Ty(p.y), NULL);

        for (const Point &p : trajectory)
        { LineTo(hdc, Tx(p.x), Ty(p.y)); }
        //------------------------------------------------------------------
        // отменяем ручку
        SelectObject(hdc, hPen_old);
    }
}
//------------------------------------------------------------------------------
void  drawMyFigure(HDC hdc, const Point &center, double w, double h, double angle, MyFigure figure, HPEN hPen)
{
    HPEN hPen_old = (HPEN)SelectObject(hdc, hPen);
    //-----------------------------------
    switch (figure)
    {
    case MyFigure::Ellipse:
        if (angle == 0.)
            Ellipse(hdc, Tx(center.x - w), Ty(center.y + h),
                    Tx(center.x + w), Ty(center.y - h));
        else
            throw std::exception("Not implemented");
        break;
    //-----------------------------------
    case MyFigure::Rectangle:
        if (angle == 0.)
            Rectangle(hdc, Tx(center.x - w), Ty(center.y + h),
                           Tx(center.x + w), Ty(center.y - h));
        else
        {
            Point p;
            p = { center.x - w, center.y + h };
            p.rotate(center, angle);
            MoveToEx (hdc, Tx(p.x), Ty(p.y), NULL);

            p = { center.x + w, center.y + h };
            p.rotate(center, angle);
            LineTo(hdc, Tx(p.x), Ty(p.y));

            p = { center.x + w, center.y - h };
            p.rotate(center, angle);
            LineTo(hdc, Tx(p.x), Ty(p.y));

            p = { center.x - w, center.y - h };
            p.rotate(center, angle);
            LineTo(hdc, Tx(p.x), Ty(p.y));

            p = { center.x - w, center.y + h };
            p.rotate(center, angle);
            LineTo   (hdc, Tx(p.x), Ty(p.y));
        }
        break;
    //-----------------------------------
    default:
        throw std::logic_error("Not Implemented");
    }
    //-----------------------------------
    SelectObject(hdc, hPen_old);
}
//------------------------------------------------------------------------------
void  makeGradient(color_interval_t colors, size_t color_gradations, color_gradient_t &gradient)
{
    gradient.clear();
    gradient.resize(color_gradations);

    COLORREF cf = colors.first, cs = colors.second;
    /* create the gradient */
    for (size_t i = 0; i < color_gradations; ++i)
    {
        BYTE r, g, b;
        /* Determine the colors */
        r = static_cast<BYTE>(GetRValue(cf) + (i * (GetRValue(cs) - GetRValue(cf)) / color_gradations));
        g = static_cast<BYTE>(GetGValue(cf) + (i * (GetGValue(cs) - GetGValue(cf)) / color_gradations));
        b = static_cast<BYTE>(GetBValue(cf) + (i * (GetBValue(cs) - GetBValue(cf)) / color_gradations));
        /* Append new color */
        gradient[i] = RGB(r, g, b);
    }
}
//------------------------------------------------------------------------------
