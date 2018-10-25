#include "StdAfx.h"
#include "RoboMovesTarget.h"
#ifdef MY_WINDOW
#include "WindowDraw.h"
#endif // MY_WINDOW

void TargetI::save(tptree &root) const
{
    tptree node;
    root.add_child(_T("target"), node);
    //node.put(_T("center"), _center);
    //node.put(_T("min"), _min);
    //node.put(_T("max"), _max);
    //node.put(_T("_thickness"), _thickness);
    node.put(_T("_precision"), _precision);
}
void TargetI::load(tptree &root)
{
    auto &node = root.get_child_optional(_T("target")).get_value_or(root);
    //_center.load(node.get_child(_T("center")));
    //_min.load(node.get_child(_T("min")));
    //_max.load(node.get_child(_T("max")));
    //_thickness = node.get<double>(_T("thickness"));
    _precision = node.get<double>(_T("precision"));
}
//------------------------------------------------------------------------------
void  RecTarget::generate ()
{
  _coords.resize(_n_rows * _n_cols);
  double r_step = (_max.y - _min.y) / (_n_rows - 1U);
  double c_step = (_max.x - _min.x) / (_n_cols - 1U);

  int k = 0;
  for (auto i = 0u; i < _n_rows; ++i)
    for (auto j = 0u; j < _n_cols; ++j)
      _coords[k++] = Point{/* left   */ _min.x + j * c_step,
                           /* bottom */ _min.y + i * r_step};
  set_thickness(std::max(r_step, c_step));
}

void  RecTarget::draw(HDC hdc, HPEN hPen,
                      bool internalLines,
                      bool internalPoints,
                      double internalPointsRadius) const
{
#ifdef MY_WINDOW
    double lft = min().x - Utils::EPSILONT;
    double btm = min().y - Utils::EPSILONT;
    double rgh = max().x + Utils::EPSILONT;
    double top = max().y + Utils::EPSILONT;

    double r = (max().y - min().y) / (_n_rows - 1U);
    double c = (max().x - min().x) / (_n_cols - 1U);

    drawMyFigure(hdc, _center, (rgh - lft) + c, (top - btm) + r,
                 0./*angle*/, MyFigure::Rectangle, hPen);
    //-----------------------------------------------------------------
    if (internalLines)
    {
        for (auto i = 1U; i < _n_cols; ++i)
            drawLine(hdc, { lft + i * c - c / 2., top - r / 2. },
                          { lft + i * c - c / 2., btm + r / 2. }, hPen);

        for (auto i = 1U; i < _n_rows; ++i)
            drawLine(hdc, { lft - c / 2., btm + i * r - r / 2. },
                          { rgh + c / 2., btm + i * r - r / 2. }, hPen);
    }
    //-----------------------------------------------------------------
    if (internalPoints)
    {
        for (auto p : _coords)
            drawCircle(hdc, p, internalPointsRadius, hPen);
    }
#endif // MY_WINDOW
}

void RecTarget::save(tptree &root) const
{
    TargetI::save(root);
    auto &node = root.get_child(_T("target"));
    node.put(_T("type"), RecTarget::name());
    node.put(_T("hn_aims"), _n_cols);
    node.put(_T("vn_aims"), _n_rows);

    //node.put(_T("left"), target.left);
    //node.put(_T("right"), target.right);
    //node.put(_T("top"), target.top);
    //node.put(_T("bottom"), target.bottom);

    auto &prect = node.get_child(_T("rectangle"));
    vec_t rect{ min(),{ max().x, min().y }, max(),{ min().x, max().y }, min() };
    for (auto &p : rect)
    {
        tptree pt;
        p.save(pt);
        prect.push_back(std::make_pair(_T(""), pt));
    }
}
void RecTarget::load(tptree &root)
{
    TargetI::load(root);
    auto &node = root.get_child_optional(_T("target")).get_value_or(root);
    //auto type = node.get<tstring>(_T("type"));
    //if (type != RecTarget::name())
    //    CERROR("RecTarget: Invalid load");

    _n_cols = node.get<unsigned>(_T("hn_aims"));
    _n_rows = node.get<unsigned>(_T("vn_aims"));

    auto prect = node.get_child_optional(_T("rectangle"));
    if (prect.is_initialized())
    {
        vec_t rect;
        for (tptree::value_type &v : prect.get())
        {
            Point p;
            p.load(v.second);
            rect.push_back(p);
        }
        calc_internals(rect);
    }
    else
    {
        double left = node.get<double>(_T("left"));
        double right = node.get<double>(_T("right"));
        double top = node.get<double>(_T("top"));
        double bottom = node.get<double>(_T("bottom"));

        _min = { left, bottom };
        _max = { right , top };
        _center = (_min + _max) / 2;
    }
    generate();
}
//------------------------------------------------------------------------------
void PolyTarget::generate()
{
    _coords.reserve(_n_rows * _n_cols);
    double r_step = (_max.y - _min.y) / (_n_rows - 1U);
    double c_step = (_max.x - _min.x) / (_n_cols - 1U);

    for (auto i = 0U; i < _n_rows; ++i)
        for (auto j = 0U; j < _n_cols; ++j)
        {
            Point p{ (_min.x + j * c_step), (_min.y + i * r_step) };
            if (PolyTarget::contain(p))
                _coords.push_back(p);
        }
    set_thickness(std::max(r_step, c_step));
}

bool PolyTarget::contain(const Point &p) const
{ return bg::within(p, _polygon)/*insidePolygon(_polygon.outer(), p)*/; }

void PolyTarget::draw(HDC hdc, HPEN hPen,
                      bool internalLines,
                      bool internalPoints,
                      double internalPointsRadius) const
{
#ifdef MY_WINDOW
    double lft = min().x - Utils::EPSILONT;
    double btm = min().y - Utils::EPSILONT;
    double rgh = max().x + Utils::EPSILONT;
    double top = max().y + Utils::EPSILONT;

    double r = (max().y - min().y) / (_n_rows - 1U);
    double c = (max().x - min().x) / (_n_cols - 1U);

    drawMyFigure(hdc, _polygon.outer(), MyFigure::Polygon, hPen);
    //-----------------------------------------------------------------
    if (internalLines)
    {
        for (auto i = 1U; i < _n_cols; ++i)
            drawLine(hdc, { lft + i * c - c / 2., top - r / 2. },
                          { lft + i * c - c / 2., btm + r / 2. }, hPen);

        for (auto i = 1U; i < _n_rows; ++i)
            drawLine(hdc, { lft - c / 2., btm + i * r - r / 2. },
                          { rgh + c / 2., btm + i * r - r / 2. }, hPen);
    }
    //-----------------------------------------------------------------
    if (internalPoints)
    {
        for (auto p : _coords)
            drawCircle(hdc, p, internalPointsRadius, hPen);
    }
#endif // MY_WINDOW
}

void PolyTarget::save(tptree &root) const
{
    TargetI::save(root);
    auto &node = root.get_child(_T("target"));
    node.put(_T("type"), RecTarget::name());
    node.put(_T("hn_aims"), _n_cols);
    node.put(_T("vn_aims"), _n_rows);

    tptree polygon;
    node.add_child(_T("polygon"), polygon);
    for (auto &p : _polygon.outer())
    {
        tptree pt;
        p.save(pt);
        polygon.push_back(std::make_pair(_T(""), pt));
    }
}
void PolyTarget::load(tptree &root)
{
    TargetI::load(root);
    auto &node = root.get_child_optional(_T("target")).get_value_or(root);
    //auto type = node.get<tstring>(_T("type"));
    //if (type != RecTarget::name())
    //    CERROR("RecTarget: Invalid load");

    _n_cols = node.get<unsigned>(_T("hn_aims"));
    _n_rows = node.get<unsigned>(_T("vn_aims"));

    auto &polygon = node.get_child(_T("polygon"));
    for (tptree::value_type &v : node.get_child(_T("polygon")))
    {
        Point p;
        p.load(v.second);
        _polygon.outer().push_back(p);
    }
    calc_internals(_polygon.outer());
    generate();
}
//------------------------------------------------------------------------------
