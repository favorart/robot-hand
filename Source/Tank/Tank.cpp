#ifdef MY_WINDOW
#include "WindowHeader.h"
#include "WindowDraw.h"
#endif // MY_WINDOW
#include "RoboMuscles.h"
#include "RoboEdges.h"
#include "Tank.h"

using namespace Robo;
using namespace Mobile;
//--------------------------------------------------------------------------------
frames_t Tank::muscleMaxLast(const Control &control) const
{
    //frames_t last = 0U;
    //for (auto &c : control)
    //{
    //    joint_t joint = jointByMuscle(c.muscle);
    //    auto sz = (physics.framesMove[joint].size() - 1);
    //    last = last ? std::min(last, sz) : sz;
    //}
    //return last;
    return Robo::LastInfinity;
}
frames_t Tank::muscleMaxLast(muscle_t muscle) const
{
    //return (physics.framesMove[jointByMuscle(muscle)].size() - 1);
    return Robo::LastInfinity;
}
//--------------------------------------------------------------------------------
bool Tank::muscleFrame (Muscle muscle, bool atStop)
{
    Joint joint = JofM(muscle);
    double Frame = 0.;
    //------------------------------------------------
    if (!atStop)
    {
        auto last = status.lastsMove[muscle] - 1;
        const auto &frames = physics.framesMove[joint];

        do
        {
            Frame = (last >= frames.size()) ? frames.back() : frames[last];
        } while (status.prevFrame[muscle] >= Frame && ++last < frames.size());
    }
    else
    {
        auto last = status.lastsStop[muscle] - 1;
        const auto &frames = physics.framesStop[joint];

        do
        {
            Frame = (last >= frames.size()) ? frames.back() : frames[last];
        } while (status.prevFrame[muscle] < Frame && ++last < frames.size());
    }
    //------------------------------------------------
    status.prevFrame[muscle] = Frame;
    // ??? hs.velosity_;
    
    // --- WIND ----------------------------------
    if (status.windy && status.lastsMove[muscle] == 1)
        Frame = Utils::random(Tank::minFrameMove, *boost::max_element(physics.framesMove[joint]));
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
    return (fabs(Frame) >= Tank::minFrameMove);
}
void Tank::muscleMove  (frames_t frame, Muscle muscle, frames_t last)
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
            || !muscleFrame(muscle, false))
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
        if (!muscleFrame(muscle, true))
        {
            /* полная остановка */
            status.lastsMove[muscle] = 0;
            status.lastsStop[muscle] = 0;
            status.lasts[muscle] = 0;
            //-----------------------------
            /* исключаем остановленный двигатель */
            status.musclesMove[muscle] = 0;
            //-----------------------------
            /* проверяем, что остальные двигатели уже остановились */
            if (boost::algorithm::none_of(status.musclesMove, [](const auto &v) { return (v != 0); }))
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
Tank::Tank(IN const Point &baseCenter, IN const std::list<JointInput> &joints) :
    params(joints),
    physics(baseCenter, joints),
    status(joints)
{
    if (!joints.size() || joints.size() > JointsMaxCount)
        throw std::exception("Incorrect joints count");
    reset();
}
//--------------------------------------------------------------------------------
Tank::Tank(IN const Point &baseCenter, IN const std::list<std::shared_ptr<Robo::JointInput>> &joints) :
    Tank(baseCenter, JInputs<Tank::JointInput>(joints)) {}
//--------------------------------------------------------------------------------
Tank::Params::Params(IN const std::list<Tank::JointInput> &jointInputs):
    musclesUsedCount(0),
    jointsUsedCount(0),
    dynamics(false),
    oppHandle(false),
    trackWidth(0.015),
    trackHeight(0.035),
    bodyHeight(0.025),
    centerRadius(0.005)
{
    muscle_t m = 0;
    joint_t j = 0;
    for (auto& jInput : jointInputs)
    {
        if (!jInput.show)
            continue;

        musclesUsedCount += 2;
        jointsUsedCount++;
        Joint joint = jInput.type;

        //defOpen[joint] = jInput.defaultPose;
        jointsUsed[j++] = joint;

        musclesUsed[m++] = MofJ(joint, true);
        musclesUsed[m++] = MofJ(joint, false);
    }

    for (; m < MusclesMaxCount; ++m)
        musclesUsed[m] = MInvalid;
    for (; j < JointsMaxCount; ++j)
        jointsUsed[j] = JInvalid;

    std::sort(std::begin(musclesUsed), std::end(musclesUsed));
    std::sort(std::begin(jointsUsed), std::end(jointsUsed));
}
//--------------------------------------------------------------------------------
Tank::Status::Status(IN const std::list<Tank::JointInput> &jointInputs) :
    visitedRarity{ 1 },
    windy{ false },
    edges{ std::make_shared<EnvEdgesTank>() }
{
    for (auto &input : jointInputs)
        if (input.show)
            curPos[input.type] = input.base;
}
//--------------------------------------------------------------------------------
Tank::Physics::Physics(IN const Point &baseCenter, IN const std::list<Tank::JointInput> &jointInputs)
{
    assert(jointInputs.size());
    maxMoveFrame = jointInputs.front().maxMoveFrame;
    assert(maxMoveFrame > 0.);

    frames_t nMoveFrames = jointInputs.front().nMoveFrames;
    assert(nMoveFrames > 0);
    frames_t nStopFrames = static_cast<frames_t>(nMoveFrames * jointInputs.front().frames.stopDistanceRatio + 2);
    assert(nStopFrames > 0.);

    jointsBases[Joint::JCount] = baseCenter;
    for (const auto &jInput : jointInputs)
    {
        if (!jInput.show)
            continue;

        Joint joint = jInput.type;
        auto law = jInput.frames;

        jointsBases[joint] = jInput.base;
        assert(jointsBases[joint] < Point(1.,1.));
        assert(maxMoveFrame == jInput.maxMoveFrame);
        assert(nMoveFrames == jInput.nMoveFrames);
        assert(nStopFrames == static_cast<frames_t>(nMoveFrames * law.stopDistanceRatio) + 2);

        framesMove[joint].resize(nMoveFrames);
        framesStop[joint].resize(nStopFrames);

        law.moveLaw->generate(framesMove[joint].begin(), nMoveFrames,
                              Tank::minFrameMove, maxMoveFrame);

        double maxVelosity = *boost::max_element(framesMove[joint]);
        law.stopLaw->generate(framesStop[joint].begin(), nStopFrames-1,
                              Tank::minFrameMove, maxMoveFrame * law.stopDistanceRatio,
                              maxVelosity);
        /* last frame must be 0 to deadend */
        framesStop[joint][nStopFrames - 1] = 0.;
    }

    //double angle = maxMoveFrame_ * M_PI / 180.;
    //double radius = boost_distance(jointsBases[Joint::LTrack], jointsBases[Joint::RTrack]) / 2.;
    //maxMoveFrame = angle * radius;
}
//--------------------------------------------------------------------------------
void Tank::realMove()
{
#ifdef TANK_DEBUG
    r1_ = r2_ = 0;
    center_ = { 0.,0. };
#endif // TANK_DEBUG

    Point &cpL{ status.curPos[Joint::LTrack] };
    Point &cpR{ status.curPos[Joint::RTrack] };

    const double between = boost_distance(cpL, cpR);

    const double shiftL = (status.shifts[Muscle::LTrackFrw] - status.shifts[Muscle::LTrackBck]);
    const double shiftR = (status.shifts[Muscle::RTrackFrw] - status.shifts[Muscle::RTrackBck]);

    if (std::isnan(shiftL) || std::isinf(shiftL) ||
        std::isnan(shiftR) || std::isinf(shiftR))
    {
        CERROR("shift NAN");
        std::getchar();
        std::exit(1);
    }

    if (fabs(shiftL) < Tank::minFrameMove &&
        fabs(shiftR) < Tank::minFrameMove)
    {
        for (auto &muscle : params.musclesUsed)
        {
            status.lastsMove[muscle] = 0;
            status.lastsStop[muscle] = 0;
            status.lasts[muscle] = 0;
            status.musclesMove[muscle] = 0;
            status.shifts[muscle] = 0;
        }
        status.moveEnd = true;
        return;
    }

    const Point bodyCenterOld = { (cpL.x + cpR.x) / 2., (cpL.y + cpR.y) / 2. };
    Point center{}, normal{};
    double tan_angle = 0., radius = 0.;

    bool strait = false;
    if (fabs(fabs(shiftL) - fabs(shiftR)) < Tank::minFrameMove)
    {
        if (boost::math::sign(shiftL) != boost::math::sign(shiftR))
        {
            cpL.rotate_radians(bodyCenterOld, std::atan(shiftL / between));
            cpR.rotate_radians(bodyCenterOld, std::atan(shiftL / between));
        }
        else
        {
            const auto shift = (shiftL + shiftR) / 2;
            // нормаль
            normal = cpR - cpL;
            normal = { -normal.y, normal.x };
            normal /= normal.norm2();
            normal *= shift;
            //CDEBUG(normal);
            strait = true;
        }
    }
    else if (fabs(shiftL) > fabs(shiftR))
    {
        tan_angle = (shiftL - shiftR) / between;
        if (fabs(tan_angle) > 0.)
        {
            radius = fabs(shiftR / tan_angle);
            center = alongLineAtDistance(cpR, cpL, radius);

#ifdef TANK_DEBUG
            r1_ = radius;
            r2_ = (shiftL / tan_angle);
            center_ = center;
#endif // TANK_DEBUG
        }
        else
        {
            // нормаль
            normal = cpR - cpL;
            normal = { -normal.y, normal.x };
            normal /= normal.norm2();
            assert(shiftL == shiftR);
            normal *= shiftL;
        }
    }
    else if (fabs(shiftL) < fabs(shiftR))
    {
        tan_angle = (shiftR - shiftL) / between;
        if (fabs(tan_angle) > 0.)
        {
            radius = fabs(shiftL / tan_angle);
            center = alongLineAtDistance(cpL, cpR, radius);

#ifdef TANK_DEBUG
            r1_ = radius;
            r2_ = (shiftR / tan_angle);
            center_ = center;
#endif // TANK_DEBUG
        }
        else
        {
            // нормаль
            normal = cpL - cpR;
            normal = { -normal.y, normal.x };
            normal /= normal.norm2();
            assert(shiftL == shiftR);
            normal *= shiftL;
        }
    }
    else
    {
        CERROR("Tank ELSE");
        std::getchar();
        std::exit(1);
    }
    
    CINFO("cpL=" << cpL << " cpR=" << cpR);
    if (fabs(tan_angle) > 0)
    {
        cpL.rotate_radians(center, +std::atan(tan_angle));
        cpR.rotate_radians(center, -std::atan(tan_angle));
    }
    else
    {
        cpL += normal;
        cpR += normal;
    }
    status.curPos[Joint::JCount] = { (cpL.x + cpR.x) / 2., (cpL.y + cpR.y) / 2. };
    CINFO("cpL=" << cpL << " cpR=" << cpR);
    CINFO("curPosBase=" << status.curPos[Joint::JCount] << " old=" << bodyCenterOld);

    Point bodyVelosity = status.curPos[Joint::JCount] - bodyCenterOld;
    if (!status.edges->interaction(*this, bodyVelosity))
    {
        /// TODO:
    }
    //Point LEdge{ std::min(cpL.x, cpR.x) - params.trackHeight,
    //             std::min(cpL.y, cpR.y) - params.trackWidth / 2 };
    //Point REdge{ std::max(cpL.x, cpR.x) + params.trackHeight,
    //             std::max(cpL.y, cpR.y) + params.trackWidth / 2 };

    //const Point LBorder{ (-1. + Tank::minFrameMove), (-1. + Tank::minFrameMove) };
    //const Point RBorder{ (+1. - Tank::minFrameMove), (+1. - Tank::minFrameMove) };

    if (boost::algorithm::none_of(status.shifts, [](const auto &c) {
        return (fabs(c) >= Tank::minFrameMove);
    }))
    {
        for (auto &muscle : params.musclesUsed)
        {
            status.lastsMove[muscle] = 0;
            status.lastsStop[muscle] = 0;
            status.lasts[muscle] = 0;
            status.musclesMove[muscle] = 0;
            //status.shifts[muscle] = 0;
        }
        status.moveEnd = true;
    }

    for (auto &muscle : params.musclesUsed)
        status.shifts[muscle] = 0.;
}
//--------------------------------------------------------------------------------
void Tank::step(IN frames_t frame, muscle_t muscle, IN frames_t last)
{
    /* Исключить незадействованные двигатели */
    if (muscle < musclesCount() && last > 0)
    {
        /// TODO: Controls check !!!
        if (status.musclesMove[M(muscle)] <= frame)
            muscleMove(frame, M(muscle), last);
    }
    else if (muscle == Robo::MInvalid && last == 0)
    {
        for (muscle_t m = 0; m < musclesCount(); ++m)
            while (status.musclesMove[M(m)] > 0 && status.musclesMove[M(m)] <= frame)
                muscleMove(frame, M(m), last);

        // step() needs !!! FOR REAL MOVE !!! 
        realMove();
    }
    else throw std::exception("Controls invalid!");
}
//--------------------------------------------------------------------------------
frames_t Tank::move(muscle_t muscle, IN frames_t last)
{
    frames_t frame = 0;
    if (muscle < musclesCount() && last > 0)
    {
        /* Что-то должно двигаться, иначе беск.цикл */
        step(frame, muscle, last);

        while (!moveEnd())
            step(frame++);
    }
    return frame;
}
frames_t Tank::move(muscle_t muscle, IN frames_t last, OUT Trajectory &visited)
{
    frames_t frame = 0;
    if (muscle < musclesCount() && last > 0)
    {
        /* start movement */
        step(frame, muscle, last);

        while (!moveEnd())
        {
            /* just moving */
            step(frame);
            if ( !(frame % getVisitedRarity()) )
                visited.push_back(position());
            ++frame;
        }
        visited.push_back(position());
    }
    return frame;
}
//--------------------------------------------------------------------------------
frames_t Tank::move(IN const Control &controls, OUT Trajectory &visited)
{
    frames_t frame = 0U;
    if (controls.size())
    {
        for (auto &c : controls)
        {
            while (c.start > frame)
            {
                step(frame);
                if ( !(frame % getVisitedRarity()) )
                    visited.push_back(position());
                ++frame;
            }
            /* start movement */
            step(frame, c.muscle, c.lasts);
        }

        while (!moveEnd())
        {
            /* just moving */
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
void Tank::draw(IN HDC hdc, IN HPEN hPen, IN HBRUSH hBrush) const
{
#ifdef MY_WINDOW
    const Point &L = status.curPos[Joint::LTrack];
    const Point &R = status.curPos[Joint::RTrack];
    
    Point centerBody{ (L.x + R.x) / 2, (L.y + R.y) / 2 };
    double phy = L.angle(R);

    // draw Body
    double bodyWidth = boost_distance(centerBody, L) + params.trackWidth;
    drawMyFigure(hdc, centerBody, bodyWidth, params.bodyHeight, phy, MyFigure::Rectangle, hPen);
    //------------------------------------------------------------------
    for (joint_t j = 0; j < jointsCount(); ++j)
    {
        Joint joint = J(j);
        const Point& pos = status.curPos[joint];
        // draw Track
        drawMyFigure(hdc, pos, params.trackWidth, params.trackHeight, phy, MyFigure::Rectangle, hPen);
    }
    //------------------------------------------------------------------
    // draw center
    drawCircle(hdc, centerBody, params.centerRadius);

#ifdef TANK_DEBUG
    if (r1_ || r2_)
    {
        drawCircle(hdc, center_, r1_);
        drawCircle(hdc, center_, r2_);
    }
#endif // TANK_DEBUG
#endif // MY_WINDOW
}
//--------------------------------------------------------------------------------
void Tank::getWorkSpace(OUT Trajectory &workSpace)
{
    reset();
    // TODO: drawWorkSpace BUG
    //----------------
    //Control controls{ { Muscle::RTrackFrw, 0, 5 } };
    //controls.append({Muscle::LTrackBck,0,5});
    //move(controls, workSpace);  // 90 degrees
    ////----------------
    //controls[1].muscle = Muscle::RTrackFrw;     move(controls, workSpace);  // (-1.0, +1.0) on begin Tracks
    //controls[1].muscle = Muscle::RTrackBck;     move(controls, workSpace);  // 90 degrees
    //controls[1].muscle = Muscle::RTrackFrw;     move(controls, workSpace);  // (-1.0, -1.0) on begin Tracks
    //controls[1].muscle = Muscle::RTrackBck;     move(controls, workSpace);  // 90 degrees
    //controls[1].muscle = Muscle::RTrackFrw;     move(controls, workSpace);  // (+1.0, -1.0) on begin Tracks
    //controls[1].muscle = Muscle::RTrackBck;     move(controls, workSpace);  // 90 degrees
    //controls[1].muscle = Muscle::RTrackFrw;     move(controls, workSpace);  // (+1.0, +1.0) on begin Tracks
    //controls[1].muscle = Muscle::RTrackBck;     move(controls, workSpace);  // 90 degrees
    //controls[1].muscle = Muscle::RTrackFrw;     move(controls, workSpace);  // (start, start) on center
    //controls[1].muscle = Muscle::RTrackBck;     move(controls, workSpace);  // 90 degrees
    // end!
    workSpace.push_back({ +0.97, +0.97 });
    workSpace.push_back({ -0.97, +0.97 });
    workSpace.push_back({ -0.97, -0.97 });
    workSpace.push_back({ +0.97, -0.97 });
    workSpace.push_back({ +0.97, +0.97 });
}
//--------------------------------------------------------------------------------
void Tank::reset()
{
    /* drop status */
    for (muscle_t m = 0; m < musclesCount(); ++m)
    {
        Muscle muscle = M(m);

        status.lastsMove[muscle] = 0;
        status.lastsStop[muscle] = 0;
        status.lasts[muscle] = 0;

        status.musclesMove[muscle] = 0;
        status.prevFrame[muscle] = 0;
        //status.acceleration[muscle] = 0.;
        //status.velosity[muscle] = 0.;
    }
    //-----------------------------------------------------
    status.moveEnd = false;
    //-----------------------------------------------------
    for (joint_t joint = 0; joint < jointsCount(); ++joint)
    {
        /* make it at resetJoint(): status[ curPo angles ] */
        resetJoint(joint);
    }
    //status.curPos[Joint::JCount] = physics.jointsBases[Joint::JCount]; // base Center;
}
//--------------------------------------------------------------------------------
void Tank::resetJoint(IN joint_t joint)
{
    /* reset joint to default */
    setJoints({ { joint, 0. } });
}
void Tank::setJoints(IN const JointsOpenPercent &percents)
{
    for (const auto &jr : percents)
    {
        Joint joint = J(jr.first);
        status.curPos[joint] = physics.jointsBases[joint];
        /* =2.8 ~distance from up-left to down-right canvas-corner */
        status.shifts[joint] = (2.8 * jr.second / 100.);
    }
    status.curPos[Joint::JCount] = (status.curPos[Joint::LTrack] + status.curPos[Joint::RTrack]) / 2;
    //realMove();
}
//--------------------------------------------------------------------------------
void Tank::controlsValidate(const Control&) const
{
    /// TODO: controlsValidate
}
//--------------------------------------------------------------------------------

