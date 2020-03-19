#pragma once

// https://github.com/jlblancoc/nanoflann
//#include "nanoflann.hpp"
//#include "mtree.h"
//
//#include "RoboMovesRecord.h"
//#ifdef MY_WINDOW
//#include "WindowHeader.h"
//#include "WindowDraw.h"
//#endif // MY_WINDOW


//#include "Robo.h"
//#include "RoboControl.h"
namespace Robo {
using distance_t = double;
class RoboI;
class Control;
}

namespace RoboPos {
class Approx;
}

namespace RoboMoves
{
//------------------------------------------------------------------------------
class Record;
class ApproxFilter;
//------------------------------------------------------------------------------
class StoreI
{
public:
    virtual void  insert(const Record&) = 0;
    virtual void  clear() = 0;
    virtual bool  empty() const = 0;
    virtual size_t size() const = 0;

    //virtual auto  begin()       -> decltype(_store.begin()) = 0;
    //virtual auto  begin() const -> decltype(_store.begin()) = 0;
    //virtual auto  end()       -> decltype(_store.end()) = 0;
    //virtual auto  end() const -> decltype(_store.end()) = 0;

    enum class Format { NONE = 0, TXT = 1, BIN = (1 << 1) };
    virtual void dump_off(const tstring &filename, const Robo::RoboI&, Format) const = 0;
    virtual void pick_up(const tstring &filename, std::shared_ptr<Robo::RoboI>&, Format) = 0;

    /// returns the closest hit in a square adjacency of the aim point
    virtual std::pair<bool, Record> getClosestPoint(const Point &aim, Robo::distance_t side) const = 0;
    /// returns the closest hit in store
    virtual Record closestEndPoint(const Point &aim) const = 0;

    using Mod = std::function<void(Record&)>;
    /// заменить для данного управления Control значения полей в Record
    virtual void replace(const Robo::Control&, const Mod&) = 0;

    using range_t = std::vector<Record>;
    /// прямоугольная окрестность точки
    virtual size_t adjacencyRectPoints(range_t&, const Point &min, const Point &max) const = 0;
    /// круглая окрестность точки
    virtual size_t adjacencyPoints(range_t&, const Point &aim, Robo::distance_t radius) const = 0;
    /// найти все движения, длительность которых находится между min и max
    virtual size_t similDistances(range_t&, Robo::distance_t min, Robo::distance_t max) const = 0;

    /// найти ближайшие действия в окрестности размера side по Индексу P 
    virtual size_t adjacencyByPBorders(range_t&, const Point &aim, Robo::distance_t side) const = 0;
    /// найти ближайшие действия в окрестности размера side по Индексу X и по Индексу Y затем
    virtual size_t adjacencyByXYBorders(const Point &aim, double side,
                                        std::pair<Record, Record> &x_pair,
                                        std::pair<Record, Record> &y_pair) const = 0;

    /// найти точно Record по его Control
    virtual const Record* exactRecordByControl(const Robo::Control&) const = 0;
    /// найти точку оставки по цели aim
    virtual size_t findEndPoint(range_t&, const Point &aim) const = 0;

    //using MultiIndexMovesIxPcIter = MultiIndexMoves::index<ByP>::type::const_iterator;
    //using MultiIndexMovesSqPassing = std::pair<MultiIndexMovesIxPcIter, MultiIndexMovesIxPcIter>;
    ///// iterators-pair using: for (auto it=ret.first; it!=ret.second; ++it) {}
    ///// \return all points in a square adjacency for the aim point
    //virtual MultiIndexMovesSqPassing aim_sq_adjacency(IN const Point &aim, IN double side) const = 0;
    //virtual MultiIndexMovesSqPassing aim_sq_adjacency(IN const Point &aim, IN const Point &min, IN const Point &max) const = 0;

    /// получить из заданной окрестности точки р размера radius все Record-ы, которые проходили через точку p
    virtual size_t near_passed_points(range_t&, const Point &p, Robo::distance_t radius) const = 0;
    /// Construct Inverse Index of all passed points
    virtual void near_passed_build_index() = 0;

    /// постоить матрицу интерполяции по заданной store для предсказания точек остановки для новых Control
    virtual void construct_approx(size_t max_n_controls, RoboMoves::ApproxFilter&) = 0;
    /// получить объект интерполяции
    virtual RoboPos::Approx* approx() = 0;

#ifdef MY_WINDOW
    using GetHPen = std::function<HPEN(const RoboMoves::Record&)>;
    virtual void draw(HDC hdc, double radius) const = 0;
    virtual void draw(HDC hdc, double radius, HPEN hPen) const = 0;
    virtual void draw(HDC hdc, double radius, const GetHPen&) const = 0;
#endif
};
}

