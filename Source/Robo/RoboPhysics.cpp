#include "Robo.h"

using namespace Robo;
//--------------------------------------------------------------------------------
void RoboI::step(muscle_t muscle, IN frames_t lasts)
{
    /* Исключить незадействованные двигатели */
    if (muscle == Robo::MInvalid && lasts == 0)
        throw std::runtime_error("Controls invalid!");

    if (muscleStatus(muscle) <= _frame)
        muscleMove(_frame, muscle, lasts);

    /* !!! NO for, NO realMove, need step(frame, traj) !!! */
}

//--------------------------------------------------------------------------------
void RoboI::step()
{
    for (muscle_t m = 0; m < musclesCount(); ++m)
        while (muscleStatus(m) > 0 &&
               muscleStatus(m) <= _frame)
            muscleMove(_frame, m, 0);

    realMove(); // step() needs !!! FOR REAL MOVE !!!

    auto rarity = getVisitedRarity();
    if (_trajectory_show && !(_frame % rarity) && (_frame / rarity) >= _trajectory.size())
        _trajectory.push_back(position());
    _frame++;
}
void RoboI::step(IN const bitset_t &muscles, IN frames_t lasts)
{
    if (!muscles.any() /*|| muscles.to_ullong() >= frames_t(std::pow(2, musclesCount())*/ || lasts == 0)
        throw std::runtime_error("Controls invalid!");

    for (muscle_t m = 0; m < musclesCount(); ++m)
        while (muscles[m] && muscleStatus(m) <= _frame)
            muscleMove(_frame, m, lasts);

    step();
}
void RoboI::step(IN const Control &control)
{
    control.validated(musclesCount());

    frames_t start = control[0].start /*, lasts = control[0].lasts*/;
    for (auto &c : control)
    {
        if (/*lasts != c.lasts ||*/ start != c.start)
            throw std::runtime_error("Controls invalid!");
        step(c.muscle, c.lasts);
    }

    step();
}

//--------------------------------------------------------------------------------
frames_t RoboI::move(IN frames_t max_frames)
{
    if (!somethingMoving())
        throw std::runtime_error("Nothing moving");

    while (_frame < max_frames && !moveEnd()) // just moving
    {
        step();
        //----------------------------------------------
        boost::this_thread::interruption_point();
    }

    if (_trajectory_show)
        _trajectory.push_back(position()); // put last

    return _frame;
}
//--------------------------------------------------------------------------------
frames_t RoboI::move(IN const bitset_t &muscles, IN frames_t lasts, IN frames_t max_frames)
{
    /* Что-то должно двигаться, иначе беск.цикл */
    if (!muscles.any() || muscles.to_ullong() >= frames_t(std::pow(2, musclesCount())) || lasts == 0)
        throw std::runtime_error("Controls invalid!");

    step(muscles, lasts);

    return move(max_frames);
}
frames_t RoboI::move(IN const bitset_t &muscles, IN frames_t lasts)
{
    return move(muscles, lasts, LastInfinity);
}
//--------------------------------------------------------------------------------
frames_t RoboI::move(IN const Control &controls)
{
    return move(controls, LastInfinity);
}
frames_t RoboI::move(IN const Control &controls, IN frames_t max_frames)
{
    controls.validated(musclesCount());

    for (auto &c : controls) // starts all moves
    {
        while (c.start > _frame)
            step();
        step(c.muscle, c.lasts);
    }

    return move(max_frames);
}

//--------------------------------------------------------------------------------

