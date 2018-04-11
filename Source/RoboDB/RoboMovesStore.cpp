#include "StdAfx.h"
#include "RoboMovesStore.h"


//------------------------------------------------------------------------------
size_t RoboMoves::Store::adjacencyByXYBorders(IN  const Point &aim, IN double side,
                                              OUT std::pair<Record, Record> &x_pair,
                                              OUT std::pair<Record, Record> &y_pair) const
{
    boost::lock_guard<boost::mutex> lock(store_mutex_);
    //-------------------------------------------------------
    size_t count = 0U;
    //-------------------------------------------------------
    {
        const MultiIndexMoves::index<ByX>::type &X_index = store_.get<ByX>();
        auto pair_x = X_index.range((aim.x - side) <= boost::lambda::_1, boost::lambda::_1 <= (aim.x + side));

        decltype(pair_x.first)  it_max_x = X_index.end();
        decltype(pair_x.first)  it_min_x = X_index.end();
        for (auto it = pair_x.first; it != pair_x.second; ++it)
        {
            if ((it->hit.x < aim.x) && (it_max_x == X_index.end() || it_max_x->hit.x < it->hit.x))
            { it_max_x = it; }

            if ((it->hit.x > aim.x) && (it_min_x == X_index.end() || it_min_x->hit.x > it->hit.x))
            { it_min_x = it; }
        }

        if (it_max_x != X_index.end() && it_min_x != X_index.end())
        { x_pair = std::make_pair(*it_max_x, *it_min_x); count += 2U; }
    }
    //-------------------------------------------------------
    {
        const MultiIndexMoves::index<ByY>::type  &Y_index = store_.get<ByY>();
        auto pair_y = Y_index.range((aim.y - side) <= boost::lambda::_1, boost::lambda::_1 <= (aim.y + side));

        decltype(pair_y.first)  it_max_y = Y_index.end();
        decltype(pair_y.first)  it_min_y = Y_index.end();
        for (auto it = pair_y.first; it != pair_y.second; ++it)
        {
            if ((it->hit.y < aim.y) && (it_max_y == Y_index.end() || it_max_y->hit.y < it->hit.y))
            { it_max_y = it; }

            if ((it->hit.y > aim.y) && (it_min_y == Y_index.end() || it_min_y->hit.y > it->hit.y))
            { it_min_y = it; }
        }

        if (it_max_y != Y_index.end() && it_min_y != Y_index.end())
        { y_pair = std::make_pair(*it_max_y, *it_min_y); count += 2U; }
    }
    //-------------------------------------------------------
    return count;
}
//------------------------------------------------------------------------------
void  RoboMoves::Store::draw(HDC hdc, HPEN hPen, double circleRadius) const
{
    HPEN hPen_old = (HPEN)SelectObject(hdc, hPen);
    // --------------------------------------------------------------
    std::unordered_map<Point, double, PointHasher>  map_points;
    // --------------------------------------------------------------
    {
        boost::lock_guard<boost::mutex>  lock(store_mutex_);
        for (auto &rec : store_)
        {
            /*  Если в одну точку попадают несколько движений -
             *  выбрать лучшее движение по элегантности */
            auto pRec = map_points.find(rec.hit);
            if (pRec != map_points.end())
            {
                double elegance = rec.eleganceMove();
                if (pRec->second < elegance)
                    map_points.insert(std::make_pair(rec.hit, elegance));
            }
            else
            { map_points.insert(std::make_pair(rec.hit, rec.eleganceMove())); }
        }
    }
    // --------------------------------------------------------------
    for (auto &pt : map_points)
        drawCircle(hdc, pt.first, circleRadius);
    // --------------------------------------------------------------
    SelectObject(hdc, hPen_old);
}
void  RoboMoves::Store::draw(HDC hdc, color_gradient_t gradient, double circleRadius) const
{
    std::vector<HPEN> hPens(gradient.size());
    for (auto i = 0U; i < gradient.size(); ++i)
    {
        // tcout << '(' << GetRValue (c) << ' '
        //                   << GetGValue (c) << ' '
        //                   << GetBValue (c) << ' '
        //            << ')' << ' '; // std::endl;
    
        HPEN hPen = CreatePen(PS_SOLID, 1, gradient[i]);
        // DrawCircle(hdc, Point(0.01 * i - 0.99, 0.9), 0.01, hPen);
        hPens[i] = hPen;
    }

    try
    {
        boost::lock_guard<boost::mutex> lock(store_mutex_);
        // --------------------------------------------------------------
        for (auto &rec : store_)
        {
            // double elegance = rec.eleganceMove ();
            // tcout << elegance << std::endl;
            // size_t index = static_cast<size_t> (elegance * (gradient.size () - 1));
            // // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            // index = index >= gradient.size () ? gradient.size () - 1 : index;

            // double longs = static_cast<double> (rec.longestMusclesControl () - minTimeLong);
            size_t longs = rec.longestMusclesControl();

            size_t index;
            if (longs > 500U)
                index = 2U;
            else if (longs > 200U)
                index = 1U;
            else
                index = 0U;

            // double step  = static_cast<double> (maxTimeLong - minTimeLong) / gradient.size ();
            // size_t index = static_cast<size_t> (longs / step);

            // size_t index = static_cast<size_t>
            //               (((longs       - minTimeLong) /
            //                 (maxTimeLong - minTimeLong)) * (gradient.size (); - 1));

            // COLORREF col = GetPixel (hdc, Tx (rec.hit.x), Ty (rec.hit.y));
            // if ( col >= RGB(255,0,0) && col <= RGB(0,0,255) && gradient[index] < col )

            if (circleRadius > 0.)
            { drawCircle(hdc, rec.hit, 0.01, hPens[index]); }
            else
            { SetPixel(hdc, Tx(rec.hit.x), Ty(rec.hit.y), gradient[index]); }

            boost::this_thread::interruption_point();
        }
    }
    catch (boost::thread_interrupted&)
    { /* tcout << _T("WorkingThread interrupted") << std::endl; */ }
    // --------------------------------------------------------------
    for (auto hPen : hPens) { DeleteObject(hPen); }
}
//------------------------------------------------------------------------------
void  RoboMoves::Store::save(tstring filename) const
{
    boost::this_thread::disable_interruption  no_interruption;
    boost::lock_guard<boost::mutex>  lock(store_mutex_);
    try
    {
        std::ofstream ofs(filename, std::ios_base::binary | std::ios_base::out);
        // boost::archive::text_oarchive  toa(ofs);
        boost::archive::binary_oarchive   boa(ofs);
        //boa & minTimeLong & maxTimeLong & store_;
    }
    catch (...)
    {
        tstring last_error = getLastErrorString();
        MessageBoxW(NULL, last_error.c_str(), _T("Error"), MB_OK);
    }
}
void  RoboMoves::Store::load(tstring filename)
{
    if (isFileExists(filename.c_str()))
    {
        boost::this_thread::disable_interruption  no_interruption;
        boost::lock_guard<boost::mutex>  lock(store_mutex_);

        std::ifstream ifs(filename, std::ios_base::binary | std::ios_base::in);
        //std::stringstream buffer(std::ios_base::binary | std::ios_base::in | std::ios_base::out);
        //buffer << ifs.rdbuf();
        //ifs.close();
      
        // boost::archive::text_iarchive tia(ifs);
        boost::archive::binary_iarchive  bia(ifs);
        //boost::archive::binary_iarchive     bia(buffer);
        //bia & minTimeLong & maxTimeLong & store_;
        //-------------------------------------------
    }
}
//------------------------------------------------------------------------------
void  RoboMoves::Store::insert(const Record &rec)
{
    // if (rec.aim in store)
    // {
    //   /* Вверх класть более удачные, отн. кол-ва движение и точность попадания */
    //  
    //   /* ??? Несколько вариантов одного и того же движения ??? */
    //   // массив траекторий для каждой точки
    //
    //   ?? store.insert (rec);
    // } else
    try
    {
        // tcout << tstring (rec) << std::endl; // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        boost::this_thread::disable_interruption no_interruption;
        boost::lock_guard<boost::mutex> lock(store_mutex_);

        store_.insert(rec);

        // auto result = store_.insert(rec);
        // if (result.second)
        // {
        //     Robo::frames_t longs = rec.longestMusclesControl();
        //     if (!maxTimeLong && !minTimeLong)
        //     {
        //         maxTimeLong = longs;
        //         minTimeLong = longs;
        //     }
        //     else if (longs > maxTimeLong)
        //     { maxTimeLong = longs; }
        //     else if (longs < minTimeLong)
        //     { minTimeLong = longs; }
        // }
    }
    catch (...)
    { MessageBox(NULL, getLastErrorString().c_str(), _T(" Error "), MB_OK); }
}
//------------------------------------------------------------------------------
void  RoboMoves::Store::insert(Robo::RoboI &robo, const Robo::Control &controls)
{
    Robo::Trajectory trajectory;

    robo.reset();
    Point pos_base = robo.position();

    robo.move(controls, trajectory);

    const Point& pos = robo.position();

    try
    {
        // boost::this_thread::disable_interruption no_interruption;
        // boost::lock_guard<boost::mutex> lock(store_mutex_);
        insert(Record(pos, pos_base, pos, controls, trajectory));
    }
    catch (...)
    { MessageBox(NULL, getLastErrorString().c_str(), _T(" Error "), MB_OK); }
}
//------------------------------------------------------------------------------
