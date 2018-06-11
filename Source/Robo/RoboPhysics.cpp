#include "RoboPhysics.h"

using namespace Robo;

//--------------------------------------------------------------------------------
bool Robo::RoboPhysics::muscleFrame(muscle_t muscle)
{
    joint_t joint = jointByMuscle(muscle);
    double Frame = 0.;
    //------------------------------------------------
    if (status.lastsMove[muscle] > 0)
    {
        assert(status.lastsStop[muscle] == 0);

        auto last = status.lastsMove[muscle] - 1;
        const auto &frames = physics.framesMove[joint];

        do
        {
            Frame = (last >= frames.size()) ? frames.back() : frames[last];
        } while (status.prevFrame[muscle] >= Frame && ++last < frames.size());
    }
    else if (status.lastsStop[muscle] > 0)
    {
        assert(status.lastsMove[muscle] == 0);

        auto last = status.lastsStop[muscle] - 1;
        const auto &frames = physics.framesStop[joint];

        do
        {
            Frame = (last >= frames.size()) ? frames.back() : frames[last];
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
    // status.curPos !!! wait for realMove
    //------------------------------------------------
    //CDEBUG("j=" << joint << ", m=" << muscle << " Frame=" << status.shifts[muscle]
    //       << ((atStop) ? " stop " : " move ")
    //       << ((atStop) ? status.lastsStop[muscle] : status.lastsMove[muscle]));
    // -------------------------------------------
    return (fabs(Frame) >= RoboI::minFrameMove);
}
void Robo::RoboPhysics::muscleMove(frames_t frame, muscle_t muscle, frames_t last)
{
    /* если не производится никакого движения и нет сигнала о начале нового */
    if (!last && !status.lastsMove[muscle] && !status.lastsStop[muscle])
        return;
    //-------------------------------------------------------
    status.moveEnd = false;
    //-------------------------------------------------------
    if (last > 0)
    {
        /* control given */
        if (!status.lastsMove[muscle])
        {
            /* начало нового движения */
            status.lastsStop[muscle] = 0;
            status.lastsMove[muscle] = 1;
            status.lasts[muscle] = last;

            if (!status.lastsStop[muscle])
                status.musclesMove[muscle] = frame;
        }
        else
        {
            /* остановка основного движения - по сигналу */
            status.lastsStop[muscle] = 1;
            status.lastsMove[muscle] = 0;
            status.lasts[muscle] = 0;
        }
    }
    //-------------------------------------------------------
    if (status.lastsMove[muscle])
    {
        /* двигатель работает */
        if (/* по истечении заданной длительности */
            status.lasts[muscle] < status.lastsMove[muscle]
            /* продолжение основного движения - остался на месте - блокировка противоположным мускулом */
            || !muscleFrame(muscle))
        {
            /* остановка основного движения */
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
    if (status.lastsStop[muscle])
    {
        /* движение по инерции */
        if (!muscleFrame(muscle))
        {
            /* если движения нет и торможение трением завершилось */
            assert(status.lastsMove[muscle] == 0);
            //-------------------------------------------------------
            /* полная остановка */
            status.lastsMove[muscle] = 0;
            status.lastsStop[muscle] = 0;
            status.lasts[muscle] = 0;
            //-----------------------------
            /* исключаем остановленный двигатель */
            status.musclesMove[muscle] = 0;
            //-----------------------------
            /* проверяем, что остальные двигатели уже остановились */
            if (ba::all_of_equal(status.musclesMove, 0))
            {
                /* Полная остановка руки */
                status.moveEnd = true;
            }
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
Robo::RoboPhysics::RoboPhysics(const Point &base,
                               const JointsPInputs &joint_inputs,
                               const std::shared_ptr<EnvEdges> &eiges) :
    physics(base, joint_inputs), status(joint_inputs), feedback(), env(eiges)
{}

//--------------------------------------------------------------------------------
Robo::RoboPhysics::Physics::Physics(const Point &base, const JointsPInputs &joint_inputs) :
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
void Robo::RoboPhysics::step(frames_t frame, muscle_t muscle, frames_t last)
{
    /* Исключить незадействованные двигатели */
    if (muscle < musclesCount() && last > 0)
    {
        /// TODO: oppo_allow
        if (status.musclesMove[muscle] <= frame)
            muscleMove(frame, muscle, last);
    }
    else if (muscle == Robo::MInvalid && last == 0)
    {
        for (muscle_t m = 0; m < musclesCount(); ++m)
            while (status.musclesMove[m] > 0 && status.musclesMove[m] <= frame)
                muscleMove(frame, m, last);
        realMove(); // step() needs REAL MOVE !!! 
    }
    else throw std::exception("Controls invalid!");
}
//--------------------------------------------------------------------------------
frames_t Robo::RoboPhysics::move(muscle_t muscle, frames_t last)
{
    frames_t frame = 0;
    if (muscle < musclesCount() && last > 0)
    {   
        step(frame, muscle, last);

        while (!moveEnd())
            step(frame++);
    }
    return frame;
}
frames_t Robo::RoboPhysics::move(muscle_t muscle, frames_t last, Trajectory &visited)
{
    frames_t frame = 0;
    if (muscle < musclesCount() && last > 0)
    {
        step(frame, muscle, last);

        while (!moveEnd())
        {
            step(frame);
            if (!(frame % getVisitedRarity()))
                visited.push_back({ position() });
            ++frame;
        }
        visited.push_back({ position() });
    }
    return frame;
}
//--------------------------------------------------------------------------------
frames_t Robo::RoboPhysics::move(IN const Control &controls, OUT Trajectory &visited)
{
    frames_t frame = 0U;
    if (controls.size())
    {
        for (auto &c : controls)
        {
            while (c.start > frame)
            {
                step(frame);
                if (!(frame % getVisitedRarity()))
                    visited.push_back(position());
                ++frame;
            }
            step(frame, c.muscle, c.lasts);
        }

        while (!moveEnd())
        {
            step(frame);
            if (!(frame % getVisitedRarity()))
                visited.push_back(position());
            ++frame;
        }
        visited.push_back(position());
    }
    return frame;
}

//--------------------------------------------------------------------------------
void Robo::RoboPhysics::step(frames_t frame, const Learn::Act &act)
{
    for (muscle_t m = 0; m < musclesCount(); ++m)
    {
        if (act.getAct(m) && status.lastsMove[m] == 0)
        {
            status.lastsStop[m] = 0;
            status.lastsMove[m] = 1;
            status.lasts[m] = UINT_MAX;
        }
        else if (!act.getAct(m) && status.lastsMove[m] != 0)
        {
            status.lastsStop[m] = 1;
            status.lastsMove[m] = 0;
            status.lasts[m] = 0;
        }

        while (status.musclesMove[m] > 0 && status.musclesMove[m] <= frame)
            muscleMove(frame, m, 0);
    }
    realMove(); // step() needs !!! REAL MOVE !!! 
}
