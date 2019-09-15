#include "WindowDraw.h"
#include <random>

const HPEN defaultPen = (HPEN)GetStockObject(BLACK_PEN);
//------------------------------------------------------------------------------
void  drawDecardsCoordinates(HDC hdc)
{
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
    for (auto i = 0u; i < 19u; ++i)
    {
        MoveToEx (hdc, Tx(-0.90 + 0.1*i), Ty(-0.01), NULL);
        LineTo   (hdc, Tx(-0.90 + 0.1*i), Ty( 0.01));
        MoveToEx (hdc, Tx(-0.01), Ty(-0.90 + 0.1*i), NULL);
        LineTo   (hdc, Tx( 0.01), Ty(-0.90 + 0.1*i));
    }
}

//------------------------------------------------------------------------------
void  drawCoordinates(HDC hdc, bool marks)
{
    //---Ox---
    MoveToEx (hdc, Tx(-1.000), Ty(-1.000), NULL);
    LineTo   (hdc, Tx( 1.000), Ty(-1.000));
    LineTo   (hdc, Tx( 0.970), Ty(-0.990));
    LineTo   (hdc, Tx( 0.990), Ty(-1.000));
    LineTo   (hdc, Tx( 0.970), Ty(-1.010));
    LineTo   (hdc, Tx( 1.000), Ty(-1.000));
    //---Oy---              
    MoveToEx (hdc, Tx(-1.000), Ty(-1.000), NULL);
    LineTo   (hdc, Tx(-1.000), Ty( 1.000));
    LineTo   (hdc, Tx(-1.005), Ty( 0.960));
    LineTo   (hdc, Tx(-1.000), Ty( 0.990));
    LineTo   (hdc, Tx(-0.992), Ty( 0.960));
    LineTo   (hdc, Tx(-1.000), Ty( 1.000));
    //--------
    for (auto i = 0u; i < 19u; ++i)
    {
        MoveToEx (hdc, Tx(-0.90 + 0.1*i), Ty(-1.01), NULL);
        LineTo   (hdc, Tx(-0.90 + 0.1*i), Ty(-0.99));

        MoveToEx (hdc, Tx(-1.01), Ty(-0.90 + 0.1*i), NULL);
        LineTo   (hdc, Tx(-0.99), Ty(-0.90 + 0.1*i));
        //--------
        if (marks)
        {
            tstring mark = to_tstring((i + 1) * 5);
            TextOut(hdc,
                    Tx(-0.98), Ty(-0.90 + 0.1*i + 0.02),  /* Location of the text */
                    mark.c_str(),                                /* Text to print */
                    (int)mark.size());                        /* Size of the text */
            TextOut(hdc,
                    Tx(-0.90 + 0.1*i - 0.01), Ty(-0.95),  /* Location of the text */
                    mark.c_str(),                                /* Text to print */
                    (int)mark.size());                        /* Size of the text */
        }
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
void  drawCross(HDC hdc, const Point &center, double radius, HPEN hPen)
{
    HPEN hPen_old = (HPEN)SelectObject(hdc, hPen);
    //-----------------------------------
    MoveToEx(hdc, Tx(center.x - radius), Ty(center.y - radius), NULL);
    LineTo  (hdc, Tx(center.x + radius), Ty(center.y + radius));

    MoveToEx(hdc, Tx(center.x - radius), Ty(center.y + radius), NULL);
    LineTo  (hdc, Tx(center.x + radius), Ty(center.y - radius));
    //-----------------------------------
    SelectObject(hdc, hPen_old);
}

//------------------------------------------------------------------------------
void  drawCircle(HDC hdc, const Point &center, double radius, HPEN hPen)
{
    if (radius > Utils::EPSILONT)
    {
        HPEN Pen_old = (HPEN)SelectObject(hdc, hPen);
        //-----------------------------------
        Ellipse(hdc, Tx(-radius + center.x), Ty(+radius + center.y),
                     Tx(+radius + center.x), Ty(-radius + center.y));
        //-----------------------------------
        SelectObject(hdc, Pen_old);
    }
    else
    {
        LOGPEN pinf;
        if (GetObject(hPen, sizeof(LOGPEN), &pinf))
        {
            // pinf: .lopnColor .lopnWidth .lopnStyle
            SetPixel(hdc, Tx(center.x), Ty(center.y), pinf.lopnColor);
        }
        else throw std::runtime_error("Can't get color");
    }
}

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
void  drawStateTrajectory(HDC hdc, const Robo::StateTrajectory &trajectory, HPEN hPen)
{
    if (!trajectory.empty())
    {
        HPEN hPen_old = (HPEN)SelectObject(hdc, hPen);
        //------------------------------------------------------------------
        const Point &p = trajectory.front().spec();
        MoveToEx(hdc, Tx(p.x), Ty(p.y), NULL);

        for (const auto &state : trajectory)
        { LineTo(hdc, Tx(state.spec().x), Ty(state.spec().y)); }
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
            Ellipse(hdc, Tx(center.x - w / 2), Ty(center.y + h / 2),
                         Tx(center.x + w / 2), Ty(center.y - h / 2));
        else
            throw std::logic_error("Not implemented");
        break;
    //-----------------------------------
    case MyFigure::Rectangle:
        if (angle == 0.)
            Rectangle(hdc, Tx(center.x - w / 2), Ty(center.y + h / 2),
                           Tx(center.x + w / 2), Ty(center.y - h / 2));
        else
        {
            Point p;

            p = { center.x - w / 2, center.y + h / 2 };
            p.rotate(center, angle);
            MoveToEx(hdc, Tx(p.x), Ty(p.y), NULL);

            p = { center.x + w / 2, center.y + h / 2 };
            p.rotate(center, angle);
            LineTo(hdc, Tx(p.x), Ty(p.y));

            p = { center.x + w / 2, center.y - h / 2 };
            p.rotate(center, angle);
            LineTo(hdc, Tx(p.x), Ty(p.y));

            p = { center.x - w / 2, center.y - h / 2 };
            p.rotate(center, angle);
            LineTo(hdc, Tx(p.x), Ty(p.y));

            p = { center.x - w / 2, center.y + h / 2 };
            p.rotate(center, angle);
            LineTo(hdc, Tx(p.x), Ty(p.y));
        }
        break;
        //-----------------------------------
    case MyFigure::Triangle:
        if (angle == 0.)
        {
            Point p;

            p = { center.x, center.y + h / 2 };
            MoveToEx(hdc, Tx(p.x), Ty(p.y), NULL);

            p = { center.x - w / 2, center.y - h / 2 };
            LineTo(hdc, Tx(p.x), Ty(p.y));

            p = { center.x + w / 2, center.y - h / 2 };
            LineTo(hdc, Tx(p.x), Ty(p.y));

            p = { center.x, center.y + h / 2 };
            LineTo(hdc, Tx(p.x), Ty(p.y));
        }
        else
        {
            Point p;

            p = { center.x, center.y + h / 2 };
            p.rotate(center, angle);
            MoveToEx(hdc, Tx(p.x), Ty(p.y), NULL);

            p = { center.x - w / 2, center.y - h / 2 };
            p.rotate(center, angle);
            LineTo(hdc, Tx(p.x), Ty(p.y));

            p = { center.x + w / 2, center.y - h / 2 };
            p.rotate(center, angle);
            LineTo(hdc, Tx(p.x), Ty(p.y));

            p = { center.x, center.y + h / 2 };
            p.rotate(center, angle);
            LineTo(hdc, Tx(p.x), Ty(p.y));
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
HPEN genStoreGradientPen(size_t longs)
{
    HPEN hpen{};
    // color_gradient_t gradient;
    // std::vector<HPEN> hPens(gradient.size());
    // for (auto i = 0U; i < gradient.size(); ++i)
    //     hPens[i] = CreatePen(PS_SOLID, 1, gradient[i]);
    // // --------------------------
    // 
    // // std::vector<HPEN> hPens(gradient.size());
    // // for (auto i = 0U; i < gradient.size(); ++i)
    // // {
    // //     // tcout << '(' << GetRValue (c) << ' '
    // //     //                   << GetGValue (c) << ' '
    // //     //                   << GetBValue (c) << ' '
    // //     //            << ')' << ' '; // std::endl;
    // // 
    // //     HPEN hPen = CreatePen(PS_SOLID, 1, gradient[i]);
    // //     // DrawCircle(hdc, Point(0.01 * i - 0.99, 0.9), 0.01, hPen);
    // //     hPens[i] = hPen;
    // // }
    // 
    // // // double elegance = rec.eleganceMove ();
    // // // tcout << elegance << std::endl;
    // // // size_t index = static_cast<size_t> (elegance * (gradient.size () - 1));
    // // // // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // // // index = index >= gradient.size () ? gradient.size () - 1 : index;
    // // 
    // // // double longs = static_cast<double> (rec.longestMusclesControl () - minTimeLong);
    // // size_t longs = rec.longestMusclesControl();
    // // 
    // // // double step  = static_cast<double> (maxTimeLong - minTimeLong) / gradient.size ();
    // // // size_t index = static_cast<size_t> (longs / step);
    // // 
    // // // size_t index = static_cast<size_t>
    // // //               (((longs       - minTimeLong) /
    // // //                 (maxTimeLong - minTimeLong)) * (gradient.size (); - 1));
    // // 
    // // // COLORREF col = GetPixel (hdc, Tx (rec.hit.x), Ty (rec.hit.y));
    // // // if ( col >= RGB(255,0,0) && col <= RGB(0,0,255) && gradient[index] < col )
    // 
    // size_t index;
    // if (longs > 500U)
    //     index = 2U;
    // else if (longs > 200U)
    //     index = 1U;
    // else
    //     index = 0U;
    // 
    // return hPens[index];
    // // --------------------------
    // for (auto hPen : hPens) { DeleteObject(hPen); }
    return hpen;
}

//{ RGB(0,0,130), RGB(255,0,0) } // 128
//{ RGB(130,0,0), RGB(255,155,155) }
//gradient_t gradient({ RGB(25, 255, 25), RGB(25, 25, 255), RGB(255, 25, 25) });
//------------------------------------------------------------------------------
//GradPens::GradPens(GradPens &&gp)
//{
//    _maxLasts = gp._maxLasts; gp._maxLasts = 0;
//    _colors = gp._colors; gp._colors = { 0,0 };
//    _colorGradations = gp._colorGradations; gp._colorGradations = 0;
//    _gradient = gp._gradient; gp._gradient.clear();
//    _gradientPens = gp._gradientPens gp._gradientPens.clear();
//}

//------------------------------------------------------------------------------
void GradPens::shuffleGradient()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    //std::random_shuffle(_gradientPens.begin(), _gradientPens.end());
    std::shuffle(_gradientPens.begin(), _gradientPens.end(), gen);
}

//------------------------------------------------------------------------------
void GradPens::restoreGradient()
{
    color_gradient_t _gradient;
    makeGradient(_colors, _colorGradations, _gradient);

    CDEBUG("restoreGradient-------------");
    _gradientPens.resize(_colorGradations);
    for (auto i = 0U; i < _colorGradations; ++i)
    {
        _gradientPens[i] = CreatePen(PS_SOLID, 1, _gradient[i]);
        CDEBUG(GetRValue(_gradient[i]) << ' ' <<
               GetGValue(_gradient[i]) << ' ' <<
               GetBValue(_gradient[i]));
    }
    CDEBUG("----------------------------");
}

//------------------------------------------------------------------------------
void GradPens::setColors(color_interval_t colors, size_t gradations)
{
    _colors = colors;
    _colorGradations = gradations;
    restoreGradient();
}

//------------------------------------------------------------------------------
HPEN GradPens::operator()(Robo::frames_t longz) const
{
    int i = 0;
    CDEBUG("longz=" << longz);
    if (longz > _maxLasts)
    {
        //throw std::runtime_error("");
        i = int(_gradientPens.size() - 1);
    }
    else
    {
        // (sz=15-1 - 0) * (input - 0) / (last=700 - 0) + 0; }
        Robo::frames_t t = Utils::interval_map(longz, { 0u, _maxLasts }, { 0u, _gradientPens.size() - 1u });
        if (t > INT_MAX)
            throw std::runtime_error("longz is too large");
        i = int(t);
    }
    return _gradientPens[i];
}
