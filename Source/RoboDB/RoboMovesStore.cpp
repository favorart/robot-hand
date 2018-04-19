#include "StdAfx.h"
#include "RoboMovesStore.h"

//------------------------------------------------------------------------------
size_t RoboMoves::Store::adjacencyByXYBorders(IN  const Point &aim, IN double side,
                                              OUT std::pair<Record, Record> &x_pair,
                                              OUT std::pair<Record, Record> &y_pair) const
{
    boost::lock_guard<boost::mutex> lock(_store_mutex);
    //-------------------------------------------------------
    size_t count = 0U;
    //-------------------------------------------------------
    {
        const MultiIndexMoves::index<ByX>::type &X_index = _store.get<ByX>();
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
        const MultiIndexMoves::index<ByY>::type  &Y_index = _store.get<ByY>();
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

// --------------------------------------------------------------
void RoboMoves::Store::draw(HDC hdc, double radius, std::function<HPEN(size_t)> getPen) const
{
    try
    {
        unsigned i = 0;
        for (const auto &rec : _store)
        {
            drawCircle(hdc, rec.hit, radius, getPen(rec.longestMusclesControl()));

            if (++i == 100)
            {
                boost::this_thread::interruption_point();
                i = 1;
            }
        }
    }
    catch (boost::thread_interrupted&)
    {
        CINFO("WorkingThread interrupted");
    }
}

//------------------------------------------------------------------------------
void RoboMoves::Store::insert(const Record &rec)
{
    try
    {
        /* массив траекторий для каждой точки
        *  Несколько вариантов одного и того же движения
        *  Вверх класть более удачные, отн. кол-ва движение и точность попадания
        */
        boost::this_thread::disable_interruption no_interruption;
        boost::lock_guard<boost::mutex> lock(_store_mutex);
        // ==============================
        auto status = _store.insert(rec);
        // ==============================
        if (status.second)
            for (const auto &p : rec.trajectory)
                _inverse.push_back({ &p, &(*status.first) });
    }
    catch (const std::exception &e)
    {
        CERROR(e.what());
    }
}

//------------------------------------------------------------------------------
void RoboMoves::Store::dump_off(const tstring &filename, bool text_else_bin) const
{
    try
    {
        boost::this_thread::disable_interruption no_interruption;
        boost::lock_guard<boost::mutex> lock(_store_mutex);

        if (text_else_bin)
        {
            std::ofstream ofs(filename, std::ios_base::out);
            boost::archive::text_oarchive toa(ofs);
            toa & *this;
        }
        else
        {
            std::ofstream ofs(filename, std::ios_base::out | std::ios_base::binary);
            boost::archive::binary_oarchive boa(ofs);
            boa & *this;
        }
        CINFO("saved '" << filename << "' store " << size() << " inverse " << _inverse.size());
    }
    catch (const std::exception &e)
    {
        CERROR(e.what());
    }
}

//------------------------------------------------------------------------------
void RoboMoves::Store::pick_up(const tstring &filename, bool text_else_bin)
{
    if (!fs::exists(filename))
        CERROR(_T("File '") << filename << _T("' does not exists."));

    clear();

    try
    {
        boost::this_thread::disable_interruption no_interruption;
        boost::lock_guard<boost::mutex> lock(_store_mutex);

        if (text_else_bin)
        {
            std::ifstream ifs(filename, std::ios_base::in);
            boost::archive::text_iarchive tia(ifs);
            tia & *this;
        }
        else
        {
            std::ifstream ifs(filename, std::ios_base::in | std::ios_base::binary);
            boost::archive::binary_iarchive bia(ifs);
            bia & *this;
        }

        for (const auto &rec : _store)
            for (const auto &p : rec.trajectory)
                _inverse.push_back({ &p, &rec });

        CINFO("loaded '" << filename << "' store " << size() << " inverse " << _inverse.size());
    }
    catch (const std::exception &e)
    {
        CERROR(e.what());
    }
}

//------------------------------------------------------------------------------
bool RoboMoves::Store::near_passed_build_index()
{
    if (_inverse_index_last == _inverse.size())
        return true;

    try
    {
        boost::lock_guard<boost::mutex> lock(_store_mutex);

        _inverse_kdtree.addPoints(_inverse_index_last, _inverse.size() - 1);
        CINFO("built " << _inverse.size() - _inverse_index_last);
        _inverse_index_last = _inverse.size();
        return true;
    }
    catch (const std::exception &e)
    {
        CERROR(e.what());
        return false;
    }
}

