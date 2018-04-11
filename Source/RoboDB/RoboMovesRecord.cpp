#include "StdAfx.h"
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
                          IN const Trajectory    &visited) :
    aim_(aim), move_begin_(move_begin), move_final_(move_final), visited_(visited)
{
    if (!visited_.size())
        throw std::exception("Incorrect trajectory in constructor Record");

    for (auto i = 0U; i < controls_count; ++i)
        control_.append({ muscles[i], starts[i], lasts[i] });

    if (!control_.validateMusclesTimes())
        throw std::exception("Invalid muscles constructor Record parameter");
}

RoboMoves::Record::Record(IN const Point         &aim,
                          IN const Point         &hand_begin,
                          IN const Point         &hand_final,
                          IN const Robo::Control &controls,
                          IN const Trajectory    &visited) :
    aim_(aim), move_begin_(hand_begin), move_final_(hand_final), control_(controls), visited_(visited)
{
    if (!visited_.size())
        throw std::exception("Incorrect trajectory in constructor Record");

    if (!control_.validateMusclesTimes())
        throw std::exception("Invalid muscles constructor Record parameter");
}
//---------------------------------------------------------

//---------------------------------------------------------
double  RoboMoves::Record::eleganceMove() const
{
    /* Суммарное время работы двигателей */
    double sum_time_muscles = 0.;
    for (size_t i = 0U; i < control_.size(); ++i)
    { sum_time_muscles += control_[i].last; }

    /* Количество движений */
    double controls_count_ratio = 1. / n_controls;
    /* Количество задействованных мышц */
    double muscles_count_ratio = ratioUsedMusclesCount();
    /* max.отклонение от оптимальной траектории */
    double max_divirgence = ratioTrajectoryDivirgence();
    /* Длина траектории по сравнениею с дистанцией */
    double distance_ratio = ratioDistanceByTrajectory();
    /* Переломы в движении */
    double trajectory_brakes = ratioTrajectoryBrakes();

    return  distance_ratio + max_divirgence
        + trajectory_brakes + sum_time_muscles
        + muscles_count_ratio + controls_count_ratio;

    // double total_time = longestMusclesControl ();
    // return total_time;
}
//---------------------------------------------------------
double  RoboMoves::Record::ratioDistanceByTrajectory() const
{
    double  distance_ratio = 1.;
    double visited_disance = 0.;

    if (visited_.size())
    {
        auto curr = move_begin_;
        /* Длина траектории по сравнениею с дистанцией */
        for (auto next = visited_.begin() /*, next = std::next (curr)*/; next != visited_.end(); ++next)
        {
            visited_disance += boost_distance(curr, *next);
            curr = *next;
        }
        visited_disance += boost_distance(aim_, visited_.back());

        distance_ratio = (visited_disance) ? (boost_distance(aim_, move_begin_) / visited_disance) : 1.;
    }
    // else distance_ratio = 1.;
    return  distance_ratio;
}
double  RoboMoves::Record::ratioTrajectoryDivirgence() const
{
    /* max.отклонение */
    typedef boost::geometry::model::d2::point_xy<double> bpt;
    boost::geometry::model::linestring<bpt>  line;

    auto start = visited_.front();
    line.push_back(bpt(start.x, start.y));
    line.push_back(bpt(aim.x, aim.y));

    double max_divirgence = 0.;
    for (auto &pt : visited_)
    {
        double dist = boost::geometry::distance(bpt(pt.x, pt.y), line);
        if (dist > max_divirgence)
            max_divirgence = dist;
    }

    return max_divirgence;
}
//---------------------------------------------------------
Robo::frames_t RoboMoves::Record::longestMusclesControl() const
{
    auto cmpAc = [](const Actuator &a, const Actuator &b) {
            return ((a.start + a.last) < (b.start + b.last));
        };
    const Actuator &longestControl = *boost::max_element(control_, cmpAc);
    return  (longestControl.start + longestControl.last);
}
//---------------------------------------------------------
double RoboMoves::Record::ratioUsedMusclesCount() const
{
    /* Количество задействованных мышц */
    double muscles_count = 0.;
    for (muscle_t m = 0; m < RoboI::musclesMaxCount; ++m)
        if (br::find(control_, m) != control_.end())
            ++muscles_count;
    return  (1. / muscles_count);
}
double RoboMoves::Record::ratioTrajectoryBrakes() const
{
    // TODO: ПЕРEЛОМЫ = остановки

    // for ( auto m : mus )
    // if ( Opn and Cls in controls )
    // or start > start + last
    // (1. / controlsCount) * /* Количество движений != ПЕРЕЛОМ */
    return 0.;
}

//---------------------------------------------------------
