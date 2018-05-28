
#ifdef MY_WINDOW
#include "WindowHeader.h"
#include "WindowDraw.h"
#endif // MY_WINDOW
#include "RoboMuscles.h"
#include "RoboEdges.h"
#include "Tank.h"

#define DEBUG_PRINT

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
    // status.curPos !!! wait for realMove
    //------------------------------------------------
#ifdef DEBUG_PRINT
    tcout << "j=" << joint << ", m=" << muscle << " Frame=" << status.shifts[muscle]
          << ((atStop) ? " stop " : " move ")
          << ((atStop) ? status.lastsStop[muscle] : status.lastsMove[muscle])
          << std::endl;
#endif
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
    r1_ = r2_ = 0;
    center_ = { 0.,0. };

    Point &cpL{ status.curPos[Joint::LTrack] };
    Point &cpR{ status.curPos[Joint::RTrack] };

    const double between = boost_distance(cpL, cpR);

    const double shiftL = (status.shifts[Muscle::LTrackFrw] - status.shifts[Muscle::LTrackBck]);
    const double shiftR = (status.shifts[Muscle::RTrackFrw] - status.shifts[Muscle::RTrackBck]);

    if (fabs(shiftL) < Tank::minFrameMove && fabs(shiftR) < Tank::minFrameMove)
    {
        for (auto &muscle : params.musclesUsed)
        {
            status.lastsMove[muscle] = 0;
            status.lastsStop[muscle] = 0;
            status.lasts[muscle] = 0;
            status.musclesMove[muscle] = 0;
        }
        status.moveEnd = true;
        return;
    }

    /// TODO: !!!!!!!!!!!! realMove BUG !!!!!!!!!!!!!!!!!!!!!!!
    // ROTATE
    Point center, normal;
    double angle, radius;
    if (fabs(shiftL) > fabs(shiftR))
    {
        angle = (shiftL - shiftR) / between;
        if (fabs(angle) > 0.)
        {
            radius = (shiftR / angle);
            center = alongLineAtDistance(cpR, cpL, radius);

            r1_ = radius;
            r2_ = (shiftL / angle);
            center_ = center;
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
    else
    {
        angle = (shiftR - shiftL) / between;
        if (fabs(angle) > 0)
        {
            radius = (shiftL / angle);
            center = alongLineAtDistance(cpL, cpR, radius);

            r1_ = radius;
            r2_ = (shiftR / angle);
            center_ = center;
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

    const Point bodyCenterOld{ (cpL.x + cpR.x) / 2., (cpL.y + cpR.y) / 2. };
    const Point bodyCenterNew = (fabs(angle) > 0. ? rotate(bodyCenterOld, center, angle) : bodyCenterOld + normal);
    status.curPos[Joint::JCount] = bodyCenterNew;

    if (bodyCenterOld.x > 1.1 || bodyCenterOld.y > 1.1 || bodyCenterNew.x > 1.1 || bodyCenterNew.y > 1.1)
        tcout << "FF" << std::endl;

    Point bodyVelosity = bodyCenterNew - bodyCenterOld;
    if (!status.edges->interaction(*this, bodyVelosity))
    {
#ifdef DEBUG_PRINT
        tcout << "------------------" << std::endl;
        tcout << "shiftL=" << shiftL << " shiftR=" << shiftR << std::endl;
        tcout << "L=" << cpL << " R=" << cpR << std::endl;
        tcout << (fabs(angle) > 0 ? "C=" : "n=") << (fabs(angle) > 0 ? center : normal);
        tcout << " a=" << angle << std::endl;
#endif

        if (fabs(angle) > 0)
        {
            auto cl = cpL, cr = cpR;

            cpL.rotate_radians(center, +angle);
            cpR.rotate_radians(center, -angle);
            
            double a = angle_radians(cpL, center, cpR);

            if (shiftL != a * radius && shiftR != a * radius)
                tcout << shiftL << ' ' << a * radius << ' ' << shiftR << " a=" << a << std::endl;
            //if (fabs(between - boost_distance(cpL, cpR)) < 10e-5)
            //{
            //    if (center != cpR)
            //        cpR = alongLineAtDistance(center, cpR, between / 2);
            //    if (center != cpL)
            //        cpL = alongLineAtDistance(center, cpL, between / 2);
            //}
        }
        else
        {
            cpL += normal;
            cpR += normal;
        }

#ifdef DEBUG_PRINT
        tcout << "------------------" << std::endl;
        tcout << "L=" << cpL << " R=" << cpR << std::endl;
        tcout << "------------------" << std::endl;
#endif
    }

    //Point LEdge{ std::min(cpL.x, cpR.x) - params.trackHeight,
    //             std::min(cpL.y, cpR.y) - params.trackWidth / 2 };
    //Point REdge{ std::max(cpL.x, cpR.x) + params.trackHeight,
    //             std::max(cpL.y, cpR.y) + params.trackWidth / 2 };
    //
    //const Point LBorder{ (-1. + Tank::minFrameMove), (-1. + Tank::minFrameMove) };
    //const Point RBorder{ (+1. - Tank::minFrameMove), (+1. - Tank::minFrameMove) };
    //
    // if (LEdge < LBorder || REdge > RBorder)
    if (boost::algorithm::none_of(status.shifts, [](const auto &c) { return (fabs(c) >= Tank::minFrameMove); }))
    {
        for (auto &muscle : params.musclesUsed)
        {
            status.lastsMove[muscle] = 0;
            status.lastsStop[muscle] = 0;
            status.lasts[muscle] = 0;
            status.musclesMove[muscle] = 0;
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
    /// TODO: drawWorkSpace BUG
    //----------------
    Control controls{ { Muscle::RTrackFrw, 0, 5 } };
    controls.append({Muscle::LTrackBck,0,5});
    move(controls, workSpace);  // 90 degrees
    //----------------
    controls[1].muscle = Muscle::RTrackFrw;     move(controls, workSpace);  // (-1.0, +1.0) on begin Tracks
    controls[1].muscle = Muscle::RTrackBck;     move(controls, workSpace);  // 90 degrees
    controls[1].muscle = Muscle::RTrackFrw;     move(controls, workSpace);  // (-1.0, -1.0) on begin Tracks
    controls[1].muscle = Muscle::RTrackBck;     move(controls, workSpace);  // 90 degrees
    controls[1].muscle = Muscle::RTrackFrw;     move(controls, workSpace);  // (+1.0, -1.0) on begin Tracks
    controls[1].muscle = Muscle::RTrackBck;     move(controls, workSpace);  // 90 degrees
    controls[1].muscle = Muscle::RTrackFrw;     move(controls, workSpace);  // (+1.0, +1.0) on begin Tracks
    controls[1].muscle = Muscle::RTrackBck;     move(controls, workSpace);  // 90 degrees
    controls[1].muscle = Muscle::RTrackFrw;     move(controls, workSpace);  // (start, start) on center
    controls[1].muscle = Muscle::RTrackBck;     move(controls, workSpace);  // 90 degrees
    // end!
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
    realMove();
}
//--------------------------------------------------------------------------------
void Tank::controlsValidate(const Control&) const
{
    /// TODO: controlsValidate
}
//--------------------------------------------------------------------------------


//void Tank::realMove()
//{
//    Point &cpL{ status.curPos[Joint::LTrack] };
//    Point &cpR{ status.curPos[Joint::RTrack] };
//    //========================================================
//    Point L{ Lp.x, Lp.y + status.shifts[Joint::LTrack] };
//    Point R{ Rp.x, Rp.y + status.shifts[Joint::RTrack] };
//    
//    Point i = intersection_point(Lp, Rp, L, R);
//    
//    double phy = angle(boost_distance(L,i), boost_distance(L,Lp));
//    status.curPos[Joint::LTrack] = rotate(L, i, phy);
//    status.curPos[Joint::RTrack] = rotate(R, i, phy);
//    //========================================================
//    double shiftL = status.shifts[Muscle::LTrackFrw] - status.shifts[Muscle::LTrackBck];
//    double shiftR = status.shifts[Muscle::RTrackFrw] - status.shifts[Muscle::RTrackBck];
//
//    if (fabs(shiftL) < Tank::minFrameMove && fabs(shiftR) < Tank::minFrameMove)
//        return;
//
//    const double between = boost_distance(cpL, cpR);
//    Point C;
//    double angle;
//    //========================================================
//    double A = (Lp.y - Rp.y);
//    double B = (Rp.x - Lp.x);
//    double C = (Lp.x * Rp.y - Rp.x * Lp.y);
//    A * C.x + B * C.y + C == 0;
//    boost_distance(Lp, C) == radius;
//    
//    //========================================================
//    auto n = Point{ -cpL.y - cpR.y, cpL.x - cpR.x } / between;
//    
//    if (shiftL > 0 && shiftR > 0)
//    {
//        // move forward
//        cpL += n * std::min(shiftL, shiftR);
//        cpR += n * std::min(shiftL, shiftR);
//    
//        shiftL -= std::min(shiftL, shiftR);
//        shiftR -= std::min(shiftL, shiftR);
//    }
//    else if (shiftL < 0 && shiftR < 0)
//    {
//        // move backward
//        cpL += n * std::max(shiftL, shiftR);
//        cpR += n * std::max(shiftL, shiftR);
//    
//        shiftL -= std::max(shiftL, shiftR);
//        shiftR -= std::max(shiftL, shiftR);
//    }
//    else
//    {
//        // turning
//        double turn = std::min(fabs(shiftL), fabs(shiftR));
//        turn = (shiftL > 0) ? +turn : -turn;
//    
//        Point C{ cpL + cpR / 2. };
//        double phy = turn / (between / 2);
//    
//        cpL.rotate(C, +phy);
//        cpR.rotate(C, -phy);
//    
//        shiftL -= turn;
//        shiftR += turn;
//    }
//}
