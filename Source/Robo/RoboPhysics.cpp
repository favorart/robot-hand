#include "RoboPhysics.h"

using namespace Robo;
//--------------------------------------------------------------------------------
bool RoboPhysics::somethingMoving() const
{
    return !(ba::all_of_equal(status.musclesMove, 0));
}
//--------------------------------------------------------------------------------
void RoboPhysics::muscleDriveStop(muscle_t muscle)
{
    status.musclesMove[muscle] = 0;
    status.prevFrame[muscle] = 0;
    status.shifts[muscle] = 0;

    status.lastsMove[muscle] = 0;
    status.lastsStop[muscle] = 0;
    status.lasts[muscle] = 0;
}
bool RoboPhysics::muscleDriveFrame(muscle_t muscle)
{
    joint_t joint = jointByMuscle(muscle);
    double Frame = 0.;
    //------------------------------------------------
    if (status.lastsMove[muscle] > 0)
    {
        auto last = status.lastsMove[muscle] - 1;
        const auto &frames = physics.framesMove[joint];

        do
        {
            Frame = (last >= frames.size()) ? frames.back() : frames[last];
            if (last >= physics.framesMove[joint].size())
                throw std::runtime_error("DriveMove: Invalid lasts " + std::to_string(joint));
        } while (status.prevFrame[muscle] >= Frame && ++last < frames.size());
    }
    else if (status.lastsStop[muscle] > 0)
    {
        auto last = status.lastsStop[muscle] - 1;
        const auto &frames = physics.framesStop[joint];

        do
        {
            Frame = (last >= frames.size()) ? frames.back() : frames[last];
            if (last >= physics.framesStop[joint].size())
                throw std::runtime_error("DriveStop: Invalid lasts " + std::to_string(joint));
        } while (status.prevFrame[muscle] < Frame && ++last < frames.size());
    }
    else throw std::logic_error("!lastsMove & !lastsStop");
    //------------------------------------------------
    status.prevFrame[muscle] = Frame;
    // ??? hs.velosity_  -- momentum velosity

    // --- WIND ----------------------------------
    if (env.windy && status.lastsMove[muscle] == 1)
        Frame = Utils::random(RoboI::minFrameMove, *boost::max_element(physics.framesMove[joint]));
    // -------------------------------------------
    status.shifts[muscle] = Frame;

    if (std::isnan(status.shifts[muscle]) || std::isinf(status.shifts[muscle]))
    {
        CERROR("shift NAN");
        getchar();
        exit(1);
    }
    // -------------------------------------------
    return (fabs(Frame) >= RoboI::minFrameMove);
}
void RoboPhysics::muscleDriveMove(frames_t frame, muscle_t muscle, frames_t lasts)
{
    /* если не производится никакого движения и нет сигнала о начале нового */
    if (!lasts && !status.lastsMove[muscle] && !status.lastsStop[muscle])
        return;
    //-------------------------------------------------------
    status.moveEnd = false;
    //-------------------------------------------------------
    if (lasts > 0)
    {
        if (!status.lastsMove[muscle])
        {
            /* движение не было - начало нового движения */
            status.lastsStop[muscle] = 0;
            status.lastsMove[muscle] = 1;
            status.lasts[muscle] = lasts;

            if (!status.lastsStop[muscle])
                status.musclesMove[muscle] = frame;
        }
        else
        {
            /* движение было - остановка по сигналу */
            status.lastsStop[muscle] = 1;
            status.lastsMove[muscle] = 0;
            status.lasts[muscle] = 0;
        }
    }
    //-------------------------------------------------------
    if (status.lastsMove[muscle])
    {
        if (status.lasts[muscle] < status.lastsMove[muscle]
            /* продолжение движения, если остался на месте (блокировка противоположным мускулом) */
            || !muscleDriveFrame(muscle))
        {
            /* остановка основного движения по истечении заявленной длительности */
            status.lastsStop[muscle] = 1;
            status.lastsMove[muscle] = 0;
            status.lasts[muscle] = 0;
        }
        else
        {
            /* Time is moving forward! */
            ++status.lastsMove[muscle];
            ++status.musclesMove[muscle];
        }
    }
    //-------------------------------------------------------
    else if (status.lastsStop[muscle])
    {
        /* движение по инерции */
        if (!muscleDriveFrame(muscle))
        {
            muscleDriveStop(muscle);
            /* проверяем, что остальные двигатели уже остановились - полная остановка */
            if (ba::all_of_equal(status.musclesMove, 0))
                status.moveEnd = true;
        }
        else
        {
            /* Time is moving forward! */
            ++status.lastsStop[muscle];
            ++status.musclesMove[muscle];
        }
    }
}
//--------------------------------------------------------------------------------
void RoboPhysics::step(const bitwise &muscles)
{
    if (!muscles.any() /*|| muscles.to_ullong() >= frames_t(std::pow(2, musclesCount())*/)
        throw std::runtime_error("Controls invalid!");

    for (muscle_t m = 0; m < musclesCount(); ++m)
    {
        if (muscleStatus(m) > 0 && muscleStatus(m) < _frame)
            throw std::runtime_error("Controls invalid frame!");
        /// TODO: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        if ((lastsStatusT(m) == 'm' && !muscles[m]) /*stop*/ ||
            (lastsStatusT(m) != 'm' &&  muscles[m]) /* go */)
            muscleDriveMove(_frame, m, 1);
    }
    step();
}

//--------------------------------------------------------------------------------
void RoboPhysics::step(muscle_t muscle, IN frames_t lasts)
{
    /* Исключить незадействованные двигатели */
    if (muscle == MInvalid && lasts == 0)
        throw std::runtime_error("Controls invalid!");

    if (muscleStatus(muscle) <= _frame)
        muscleDriveMove(_frame, muscle, lasts);

    /* !!! NO for, NO realMove, need step(frame, traj) !!! */
}

//--------------------------------------------------------------------------------
void RoboPhysics::step()
{
    for (muscle_t m = 0; m < musclesCount(); ++m)
        while (muscleStatus(m) > 0 &&
               muscleStatus(m) <= _frame)
            muscleDriveMove(_frame, m, 0);

    realMove(); // step() needs !!! FOR REAL MOVE !!!

    auto rarity = getVisitedRarity();
    if (_trajectory_show && !(_frame % rarity) && (_frame / rarity) >= _trajectory.size())
        _trajectory.push_back(position());
    _frame++;
}
void RoboPhysics::step(IN const bitset_t &muscles, IN frames_t lasts)
{
    if (!muscles.any() /*|| muscles.to_ullong() >= frames_t(std::pow(2, musclesCount())*/ || lasts == 0)
        throw std::runtime_error("Controls invalid!");

    for (muscle_t m = 0; m < musclesCount(); ++m)
    {
        while (muscleStatus(m) > 0 &&
               muscleStatus(m) < _frame)
            muscleDriveMove(_frame, m, 0);
        if (muscles[m]/* && muscleStatus(m) == _frame*/)
            muscleDriveMove(_frame, m, lasts);
    }

    step();
}
void RoboPhysics::step(IN const Control &control)
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
void RoboPhysics::step(IN const Control &control, OUT size_t &control_curr)
{
    control.validated(musclesCount());
    for (size_t i = control_curr; (i < control.size()) && (control[i].start == _frame); ++i)
    {
        step(control[i].muscle, control[i].lasts);
        control_curr = i + 1;
    }
    step();
}

//--------------------------------------------------------------------------------
frames_t RoboPhysics::move(IN frames_t max_frames)
{
    if (!somethingMoving())
        //throw std::runtime_error("Nothing moving");
        return _frame;

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
frames_t RoboPhysics::move(IN const bitset_t &muscles, IN frames_t lasts, IN frames_t max_frames)
{
    /* Что-то должно двигаться, иначе беск.цикл */
    if (!muscles.any() || muscles.to_ullong() >= frames_t(std::pow(2, musclesCount())) || lasts == 0)
        throw std::runtime_error("Controls invalid!");

    step(muscles, lasts);

    return move(max_frames);
}
frames_t RoboPhysics::move(IN const bitset_t &muscles, IN frames_t lasts)
{
    return move(muscles, lasts, LastsInfinity);
}
//--------------------------------------------------------------------------------
frames_t RoboPhysics::move(IN const Control &controls)
{
    return move(controls, LastsInfinity);
}
frames_t RoboPhysics::move(IN const Control &controls, IN frames_t max_frames)
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
RoboPhysics::RoboPhysics(const Point &base,
                               const JointsPInputs &joint_inputs,
                               const std::shared_ptr<EnvEdges> &eiges) :
    physics(base, joint_inputs), status(joint_inputs), feedback(), env(eiges)
{}
//--------------------------------------------------------------------------------
RoboPhysics::Physics::Physics(const Point &base, const JointsPInputs &joint_inputs) :
    dynamics(true), oppHandle(true)
{
    auto joints_count = joint_inputs.size();
    assert(joints_count > 0);

    auto base_center = joints_count;
    jointsBases[base_center] = base;

    for (const auto &j_in : joint_inputs)
    {
        if (!j_in->show)
            continue;

        auto joint = j_in->type;
        auto law = j_in->frames;
        
        auto nMoveFrames = j_in->nMoveFrames;
        auto nStopFrames = static_cast<frames_t>(nMoveFrames * law.stopDistanceRatio) + 2;
        
        framesMove[joint].resize(nMoveFrames);
        framesStop[joint].resize(nStopFrames);

        law.moveLaw->generate(framesMove[joint].begin(), nMoveFrames,
                              RoboI::minFrameMove, j_in->maxMoveFrame);

        double maxVelosity = *boost::max_element(framesMove[joint]);
        law.stopLaw->generate(framesStop[joint].begin(), nStopFrames - 1,
                              RoboI::minFrameMove, j_in->maxMoveFrame * law.stopDistanceRatio,
                              maxVelosity);
        /* last frame must be 0 to deadend */
        framesStop[joint][nStopFrames - 1] = 0.;
        jointsBases[joint] = j_in->base;
    }
}
//--------------------------------------------------------------------------------
