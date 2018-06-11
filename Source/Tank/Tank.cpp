#ifdef MY_WINDOW
#include "WindowHeader.h"
#include "WindowDraw.h"
#endif // MY_WINDOW
#include "RoboMuscles.h"
#include "RoboInputs.h"
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
    return Robo::LastsInfinity;
}
frames_t Tank::muscleMaxLast(muscle_t muscle) const
{
    //return (physics.framesMove[jointByMuscle(muscle)].size() - 1);
    return Robo::LastsInfinity;
}
//--------------------------------------------------------------------------------
void Tank::realMove()
{
#ifdef TANK_DEBUG
    r1_ = r2_ = 0;
    center_ = { 0.,0. };
#endif // TANK_DEBUG

    const joint_t jcenter = jointsCount();
    const joint_t l_track = (params.jointsUsed[0] == Joint::LTrack) ? 0 : 1;
    const joint_t r_track = (params.jointsUsed[0] == Joint::RTrack) ? 0 : 1;

    const muscle_t mlf = muscleByJoint(l_track, true);
    const muscle_t mlb = muscleByJoint(l_track, false);
    const muscle_t mrf = muscleByJoint(r_track, true);
    const muscle_t mrb = muscleByJoint(r_track, false);

    const double shiftL = (status.shifts[mlf] - status.shifts[mlb]);
    const double shiftR = (status.shifts[mrf] - status.shifts[mrb]);

    Point &cpL{ status.curPos[l_track] };
    Point &cpR{ status.curPos[r_track] };
    const double between = boost_distance(cpL, cpR);

    if (std::isnan(shiftL) || std::isinf(shiftL) ||
        std::isnan(shiftR) || std::isinf(shiftR))
    {
        CERROR("shift NAN");
        std::getchar();
        std::exit(1);
    }

    if (fabs(shiftL) < RoboI::minFrameMove &&
        fabs(shiftR) < RoboI::minFrameMove)
    {
        for (muscle_t muscle = 0; muscle < musclesCount(); ++muscle)
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

    if (fabs(fabs(shiftL) - fabs(shiftR)) < RoboI::minFrameMove)
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

    //CINFO("cpL=" << cpL << " cpR=" << cpR);
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

    status.curPos[jcenter] = { (cpL.x + cpR.x) / 2., (cpL.y + cpR.y) / 2. };
    //CINFO("cpL=" << cpL << " cpR=" << cpR);
    //CINFO("curPosBase=" << status.curPos[jcenter] << " old=" << bodyCenterOld);

    Point bodyVelosity = status.curPos[jcenter] - bodyCenterOld;
    if (!env.edges->interaction(*this, bodyVelosity))
    {
        /// TODO:
    }
    //Point LEdge{ std::min(cpL.x, cpR.x) - params.trackHeight,
    //             std::min(cpL.y, cpR.y) - params.trackWidth / 2 };
    //Point REdge{ std::max(cpL.x, cpR.x) + params.trackHeight,
    //             std::max(cpL.y, cpR.y) + params.trackWidth / 2 };

    //const Point LBorder{ (-1. + Tank::minFrameMove), (-1. + Tank::minFrameMove) };
    //const Point RBorder{ (+1. - Tank::minFrameMove), (+1. - Tank::minFrameMove) };

    if (ba::none_of(status.shifts, [](const auto &shift) { return (fabs(shift) >= RoboI::minFrameMove); }))
    {
        for (muscle_t muscle = 0; muscle < musclesCount(); ++muscle)
        {
            status.lastsMove[muscle] = 0;
            status.lastsStop[muscle] = 0;
            status.lasts[muscle] = 0;
            status.musclesMove[muscle] = 0;
            //status.shifts[muscle] = 0;
        }
        status.moveEnd = true;
    }

    for (muscle_t muscle = 0; muscle < musclesCount(); ++muscle)
        status.shifts[muscle] = 0.;
}

//--------------------------------------------------------------------------------
Tank::Tank(const Point &base, const JointsPInputs &joints) :
    RoboPhysics(base, joints, std::make_shared<EnvEdgesTank>()),
    params(joints, *this)
{
    if (!joints.size() || joints.size() > Tank::JCount)
        throw std::logic_error("Incorrect joints count");
    reset();
}

//--------------------------------------------------------------------------------
Tank::Params::Params(const JointsPInputs &joint_inputs, const Tank &tank):
    trackWidth(0.015),
    trackHeight(0.035),
    bodyHeight(0.025),
    centerRadius(0.005)
{
    const auto &front = *joint_inputs.front();
    auto maxMoveFrame = front.maxMoveFrame;
    assert(maxMoveFrame > RoboI::minFrameMove);

    frames_t nMoveFrames = front.nMoveFrames;
    assert(nMoveFrames > 0);

    frames_t nStopFrames = static_cast<frames_t>(nMoveFrames * front.frames.stopDistanceRatio + 2);
    assert(nStopFrames > 0);

    muscle_t m = 0;
    joint_t j = 0;
    for (const auto &j_in : joint_inputs)
    {
        if (!j_in->show)
            continue;

        //assert(jointsBases[joint] < Point(1., 1.));
        assert(maxMoveFrame == j_in->maxMoveFrame);
        assert(nMoveFrames == j_in->nMoveFrames);
        assert(nStopFrames == static_cast<frames_t>(nMoveFrames * j_in->frames.stopDistanceRatio) + 2);
        
        auto pTJIn = dynamic_cast<const Tank::JointInput*>(j_in.get());
        
        jointsUsed[j++] = pTJIn->joint;
        
        musclesUsed[m++] = tank.MofJ(pTJIn->joint, true);
        musclesUsed[m++] = tank.MofJ(pTJIn->joint, false);
    }
    for (; m < Tank::MCount; ++m)
        musclesUsed[m] = Tank::MInvalid;
    for (; j < Tank::JCount; ++j)
        jointsUsed[j] = Tank::JInvalid;
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
void Tank::resetJoint(joint_t joint)
{
    /* reset joint to default */
    setJoints({ { joint, 0. } });
}
//--------------------------------------------------------------------------------
void Tank::setJoints(const JointsOpenPercent &percents)
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
    workSpace.push_back({Point{ +0.97, +0.97 }});
    workSpace.push_back({Point{ -0.97, +0.97 }});
    workSpace.push_back({Point{ -0.97, -0.97 }});
    workSpace.push_back({Point{ +0.97, -0.97 }});
    workSpace.push_back({Point{ +0.97, +0.97 }});
}
//--------------------------------------------------------------------------------

