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
            //if (last > physics.framesMove[joint].size())
            //    throw std::runtime_error("DriveMove: Invalid lasts " + std::to_string(joint));
        } while (status.prevFrame[muscle] > Frame && ++last < frames.size());
    }
    else if (status.lastsStop[muscle] > 0)
    {
        auto last = status.lastsStop[muscle] - 1;
        const auto &frames = physics.framesStop[joint];

        do
        {
            Frame = (last >= frames.size()) ? frames.back() : frames[last];
            //if (last > physics.framesStop[joint].size())
            //    throw std::runtime_error("DriveStop: Invalid lasts " + std::to_string(joint));
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
        if (status.lastsMove[muscle] == 0)
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
    if (status.lastsMove[muscle] > 0)
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
    else if (status.lastsStop[muscle] > 0)
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
        if (status.musclesMove[m] > 0) // уже движется
        {
            if (status.musclesMove[m] < _frame || status.lasts[m] != (status.lastsMove[m] - 1))
                throw std::runtime_error("Controls invalid frame!");
            if (muscles[m])
                status.lasts[m]++;
        }
        else
            muscleDriveMove(_frame, m, 1);
    }
    step();
}

//--------------------------------------------------------------------------------
void RoboPhysics::step(muscle_t m, IN frames_t lasts)
{
    if (status.musclesMove[m] != (0) &&
        status.musclesMove[m] != (_frame) &&
        status.musclesMove[m] != (_frame + 1))
        throw std::runtime_error("Controls invalid!");
    if (status.musclesMove[m] == 0 || status.musclesMove[m] == _frame)
        muscleDriveMove(_frame, m, lasts);
    // !!! NO for, thus NO realMove - need step()!
}

//--------------------------------------------------------------------------------
void RoboPhysics::step()
{
    for (muscle_t m = 0; m < musclesCount(); ++m)
        step(m, 0);

    realMove(); // step() needs !!! FOR REAL MOVE !!!

    auto rarity = getVisitedRarity();
    if (_trajectory_save && !(_frame % rarity) && (_frame / rarity) >= _trajectory.size())
        _trajectory.push_back(position());
    _frame++;
}
void RoboPhysics::step(IN const bitset_t &muscles, IN frames_t lasts)
{
    for (muscle_t m = 0; m < musclesCount(); ++m)
        if (muscles[m])
            step(m, lasts);

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
    for (size_t i = control_curr; i < control.size(); ++i)
    {
        if (control[i].start < _frame)
            throw std::runtime_error("Controls invalid!");
        if (control[i].start > _frame)
            break;
        step(control[i].muscle, control[i].lasts);
        control_curr = i + 1;
    }
    step();
}

//--------------------------------------------------------------------------------
frames_t RoboPhysics::move(IN const std::vector<muscle_t> &ctrls)
{
    for (frames_t i = 0; (i < ctrls.size()) && (!moveEnd()); ++i)
    {
        step({ ctrls[i] });
        boost::this_thread::interruption_point();
    }
    if (_trajectory_save)
        _trajectory.push_back(position());
    return _frame;
}
//--------------------------------------------------------------------------------
frames_t RoboPhysics::move(IN frames_t max_frames)
{
    /* Исключить незадействованные двигатели */
    if (!somethingMoving())
        //throw std::runtime_error("Nothing moving");
        return _frame;

    while (_frame < max_frames && !moveEnd()) // just moving
    {
        if (!somethingMoving())
            return _frame;
        //----------------------------------------------
        step();
        //----------------------------------------------
        boost::this_thread::interruption_point();
    }

    if (_trajectory_save)
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
void RoboPhysics::reset()
{
    _reset();
    //-----------------------------------------------------
    for (muscle_t m = 0; m < musclesCount(); ++m)
    {
        muscleDriveStop(m);
        //status.acceleration[m] = 0.;
        //status.velosity[m] = 0.;
    }
    //-----------------------------------------------------
    status.moveEnd = false;
    //-----------------------------------------------------
    for (joint_t j = 0; j < jointsCount(); ++j)
        resetJoint(j);
}
//--------------------------------------------------------------------------------
RoboPhysics::RoboPhysics(const Point &base,
                         const JointsInputsPtrs &joint_inputs,
                         const std::shared_ptr<EnvEdges> &eiges) :
    RoboI(), physics(base, joint_inputs), status(joint_inputs), feedback(), env(eiges)
{
    if (!ba::is_sorted(joint_inputs))
        throw std::runtime_error("Joint Inputs are not sorted!");
}
//--------------------------------------------------------------------------------
RoboPhysics::Status::Status(const JointsInputsPtrs &joint_inputs)
{
    jointsCount = 0;
    for (auto &j_in : joint_inputs)
        if (j_in->show)
            curPos[jointsCount++] = j_in->base;
    musclesCount = RoboI::musclesPerJoint * jointsCount;
}
//--------------------------------------------------------------------------------
RoboPhysics::Physics::Physics(const Point &base, const JointsInputsPtrs &joint_inputs) :
    dynamics(true), oppHandle(true)
{
    joint_t j = 0;
    for (const auto &j_in : joint_inputs)
    {
        if (!j_in->show)
            continue;

        auto law = j_in->frames;
        
        auto nMoveFrames = j_in->nMoveFrames;
        auto nStopFrames = static_cast<frames_t>(nMoveFrames * law.stopDistanceRatio) + 2;
        
        framesMove[j].resize(nMoveFrames);
        framesStop[j].resize(nStopFrames);

        law.moveLaw->generate(framesMove[j].begin(), nMoveFrames,
                              RoboI::minFrameMove, j_in->maxMoveFrame);

        double maxVelosity = *boost::max_element(framesMove[j]);
        law.stopLaw->generate(framesStop[j].begin(), nStopFrames - 1,
                              RoboI::minFrameMove, j_in->maxMoveFrame * law.stopDistanceRatio,
                              maxVelosity);
        /* last frame must be 0 to deadend */
        framesStop[j][nStopFrames - 1] = 0.;
        jointsBases[j] = j_in->base;
        ++j;
    }

    if (!j)
        throw std::runtime_error("No active joints!");

    jointsBases[j/*base_center*/] = base;
}
//--------------------------------------------------------------------------------
