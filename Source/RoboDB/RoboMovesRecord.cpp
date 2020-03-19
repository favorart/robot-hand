#include "RoboMovesRecord.h"
#include "RoboMovesStore.h"

using namespace Robo;
using namespace RoboMoves;
//---------------------------------------------------------
RoboMoves::Record::Record(IN const Point         &aim,
                          IN const Point         &move_begin,
                          IN const Point         &move_final,
                          IN const muscles_array &muscles,
                          IN const frames_array  &starts,
                          IN const frames_array  &lasts,
                          IN size_t               controls_count,
                          IN const Traj          &visited) :
    aim_(aim), move_begin_(move_begin), move_final_(move_final), visited_(visited),
    _strategy(Strategy::get(controls)),
    _lasts_step(0)
{
    if (!visited_.size())
        throw std::logic_error("Incorrect trajectory in constructor Record");

    for (auto i = 0U; i < controls_count; ++i)
        control_.append({ muscles[i], starts[i], lasts[i] });
    control_.validated(RoboI::musclesMaxCount);

    updateErrDistance(move_final);
}

RoboMoves::Record::Record(IN const Point   &aim,
                          IN const Point   &move_begin,
                          IN const Point   &move_final,
                          IN const Control &controls,
                          IN const Traj    &visited,
                          IN Robo::frames_t lasts_step) :
    aim_(aim), move_begin_(move_begin), move_final_(move_final),
    control_(controls), visited_(visited), _lasts_step(0),
    _strategy(Strategy::get(controls)),
    _update_time(std::time(NULL)), _update_traj(0)
{
    if (!visited_.size())
        throw std::logic_error("Record: Incorrect trajectory");
    _lasts_step = lasts_step;
    if (!_lasts_step)
        throw std::logic_error("Record: Incorrect lasts_step");
    //if (!control_.validateMusclesTimes())
    //    throw std::logic_error("Record: Invalid control");
    updateErrDistance(move_final_);
}
//---------------------------------------------------------
void RoboMoves::Record::clear()
{
    aim_ = { 0.,0. };
    move_begin_ = { 0.,0. };
    move_final_ = { 0.,0. };
    control_.clear();
    visited_.clear();

    _strategy = Strategy::getEmpty();
    _lasts_step = 0;
    _error_distance = 0.;
    _update_time = 0;
    _update_traj = 0;
}
//---------------------------------------------------------
distance_t RoboMoves::Record::eleganceMove() const
{
    distance_t res = 0;
    /* Количество движений */
    res += distance_t(1) / getNControls();
    /* Суммарное время работы двигателей */
    res += ratioSumOfWorkingTime();
    /* Количество задействованных мышц */
    res += ratioUsedMusclesCount();
    /* max.отклонение от оптимальной траектории */
    res += ratioTrajectoryDivirgence();
    /* Длина траектории по сравнениею с дистанцией */
    res += ratioDistanceByTrajectory();
    /* Переломы в движении */
    res += ratioTrajectoryBrakes();
    return res;
}
//---------------------------------------------------------
distance_t RoboMoves::Record::ratioDistanceByTrajectory() const
{
    distance_t visited_distance = 0;
    for (auto curr = visited_.begin(), next = std::next(curr); next != visited_.end(); curr = next, ++next)
        visited_distance += bg::distance(curr->spec(), next->spec());
    /* длина траектории по сравнениею с дистанцией */
    return (visited_distance > 0) ? (bg::distance(aim_, move_begin_) / visited_distance) : 1.;
}
distance_t RoboMoves::Record::ratioTrajectoryDivirgence() const
{
    using Line = boost::geometry::model::linestring<Point>;
    Line line{ move_begin_, aim };
    distance_t max_divirgence = 0.;
    /* max. отклонение от кратчайшего пути - прямой */
    br::for_each(visited_, [&line, &max_divirgence](const auto &state) {
        distance_t d = bg::distance(state.spec(), line);
        if (max_divirgence < d)
            max_divirgence = d;
    });
    return distance_t(1) / max_divirgence;
}
//---------------------------------------------------------
Robo::frames_t RoboMoves::Record::longestMusclesControl() const
{
    auto cmpAc = [](const Actuator &a, const Actuator &b) {
        return ((a.start + a.lasts) < (b.start + b.lasts));
    };
    const Actuator &longestControl = *boost::max_element(control_, cmpAc);
    return (longestControl.start + longestControl.lasts);
}
//---------------------------------------------------------
distance_t RoboMoves::Record::ratioSumOfWorkingTime() const
{
    frames_t sum = 0;
    br::for_each(control_, [&sum](const Actuator &a) { sum += a.lasts; });
    return distance_t(1) / sum;
}
distance_t RoboMoves::Record::ratioUsedMusclesCount() const
{
    muscle_t muscles = 0, max_m = 0;
    br::for_each(control_, [&muscles, &max_m](const Actuator &a) {
        max_m = std::max(max_m, a.muscle);
        muscles |= (1 << a.muscle);
    });
    /* Количество задействованных мышц */
    unsigned count = 0;
    for (muscle_t m = 0; m < max_m; ++m)
        count += (muscles & (1 << m)) ? 1 : 0;
    return distance_t(1) / count;
}
distance_t RoboMoves::Record::ratioTrajectoryBrakes() const
{
    unsigned count = 0;
    // резакая смена направления - ПЕРEЛОМЫ и остановки
    distance_t small = (2 * RoboI::minFrameMove);
    for (auto prev = visited_.begin(), curr = std::next(prev), next = std::next(curr);
         next != visited_.end();
         prev = curr, curr = next, ++next)
    {
        auto a = angle_degrees(prev->spec(), curr->spec(), next->spec());
        if (a < 120 || a > 240 || bg::distance(curr->spec(), next->spec()) < small)
            ++count;
    }
    return distance_t(1) / count;
}
//------------------------------------------------------------------------------
tostream& RoboMoves::operator<<(tostream &s, const RoboMoves::Record &rec)
{
    s << rec.aim_ << std::endl;
    s << rec.move_begin_ << std::endl;
    s << rec.move_final_ << std::endl;
    s << rec.control_ << std::endl;
    s << rec.visited_.size() << _T(' ');
    //for (auto p : rec.visited_)
    //    s << p << _T(' ');
    //s << (_update_time - std::time(NULL)) << _T(' ') << _update_traj  << std::endl;
    //s << _error_distance << std::endl;
    s << rec._lasts_step << std::endl;
    s << std::endl;
    return s;
}
tistream& RoboMoves::operator>>(tistream &s, RoboMoves::Record &rec)
{
    s >> rec.aim_;
    s >> rec.move_begin_;
    s >> rec.move_final_;
    s >> rec.control_;
    size_t sz;
    s >> sz;
    //for (auto i : boost::irange<size_t>(0, sz))
    //{
    //    Point p;
    //    s >> p;
    //    rec.visited_.push_back(p);
    //}
    //s >> _update_time >> _update_traj;
    //_update_time += std::time(NULL);
    //s >> _error_distance;
    s >> rec._lasts_step;
    rec.updateErrDistance(rec.move_final_);
    return s;
}
//------------------------------------------------------------------------------
Strategy RoboMoves::Strategy::get(const Robo::Control &controls)
{
    // int(std::ceil(std::log2(RoboI::musclesMaxCount))); //=3=log2(8)
    //if (controls.size() > NPOS / n_muscles_bits) // 64/8 ~ 8
    //    CERROR("Too long control=" << controls.size() << " >" << NPOS / n_muscles_bits);
    return Strategy::get(controls, RoboI::musclesMaxCount); // !!! INCORRECT depends on n_muscles
}
//------------------------------------------------------------------------------
Strategy RoboMoves::Strategy::get(const Robo::Control &controls, muscle_t n_muscles)
{
    if (controls.size() > NPOS / n_muscles)
        CERROR("Too long control=" << controls.size() << " >" << NPOS / n_muscles);

    int pos = 0;
    Strategy strategy(n_muscles);
    for (auto &a : controls)
        strategy._number |= (Strategy::Value(a.muscle) << (n_muscles * pos++));
    return strategy;
}

//------------------------------------------------------------------------------
RoboMoves::Strategy::Value RoboMoves::Strategy::chunk(size_t pos) const
{
    Value _chunk = ((1 << _nmuscles) - 1) << (pos * _nmuscles);
    return (_chunk & _number) >> (pos * _nmuscles);
}

//------------------------------------------------------------------------------
bool RoboMoves::Strategy::operator==(const RoboMoves::Strategy &s) const
{
    if (_nmuscles != s._nmuscles)
        CERROR("Invalid compare n_muscles=" << _nmuscles << " > s.n_muscles" << s._nmuscles);

    if (!_number || !s._number) // empty equals all
        return true;
    //for (size_t pos = 0; pos < nchunks(); ++pos)
    //    if (chunk(pos) != s.chunk(pos))
    //        return false;
    //return true;
    return (_number == s._number);
}

//------------------------------------------------------------------------------
bool RoboMoves::Strategy::almost_eq(const RoboMoves::Strategy &s) const
{
    if (_nmuscles != s._nmuscles)
        CERROR("Invalid compare n_muscles=" << _nmuscles << " > s.n_muscles" << s._nmuscles);

    if (!_number || !s._number) // empty equals all
        return true;

    Value chunk1, chunk2;
    for (size_t lpos = 0, rpos = 0; lpos < nchunks() || rpos < s.nchunks(); ++lpos, ++rpos)
    {
        while ((chunk1 =   chunk(lpos)) == 0 /*skip empties*/ && lpos <   nchunks()) ++lpos;
        while ((chunk2 = s.chunk(rpos)) == 0 /*skip empties*/ && rpos < s.nchunks()) ++rpos;
        //if (chunk1 == 0 || chunk2 == 0) break;
        if (chunk1 != chunk2)
            return false;
    }
    return true;
}

