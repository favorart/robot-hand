﻿#ifdef MY_WINDOW
#include "WindowHeader.h"
#include "WindowDraw.h"
#endif // MY_WINDOW
#include "RoboMuscles.h"
#include "RoboInputs.h"
#include "RoboEdges.h"
#include "RoboPhysicsStatus.h"
#include "Tank.h"

using namespace Robo;
using namespace Mobile;
double betw;
//--------------------------------------------------------------------------------
frames_t Tank::muscleMaxLasts(const Control &control) const
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
frames_t Tank::muscleMaxLasts(muscle_t muscle) const
{
    //return (physics.framesMove[jointByMuscle(muscle)].size() - 1);
    return Robo::LastsInfinity;
}
//--------------------------------------------------------------------------------
const Point& Tank::position() const
{ return status->currPos[Center]; }
//--------------------------------------------------------------------------------
Tank::Tank(const Point &base, const JointsInputsPtrs &joints) :
    RoboPhysics(base, joints, std::make_shared<EnvEdgesTank>(*this, 10/*%*/, 1.5)),
    params(joints, *this)
{
    if (params.jointsUsed[0] != Joint::LTrack || params.jointsUsed[1] != Joint::RTrack)
        throw std::logic_error("Invalid tracks numeration");
    if (!joints.size() || joints.size() > jointsCount())
        throw std::logic_error("Incorrect joints count");
    //reset();
    betw = boost_distance(currPos(LTrack), currPos(RTrack));
}
//--------------------------------------------------------------------------------
Tank::Params::Params(const JointsInputsPtrs &joint_inputs, const Tank &tank) :
    trackWidth(0.02),
    trackHeight(0.05),
    bodyHeight(0.04),
    centerRadius(0.005)
{
    musclesUsed.fill(Tank::Muscle::MInvalid);
    jointsUsed.fill(Tank::Joint::JInvalid);

    const auto &front = *joint_inputs.front();
    auto dMoveDistance = front.frames.dMoveDistance;
    assert(dMoveDistance > RoboI::minFrameMove);

    auto nMoveFrames = front.frames.nMoveFrames;
    assert(nMoveFrames > 0);

    frames_t nStopFrames = static_cast<frames_t>(front.frames.nMoveFrames * front.frames.dInertiaRatio + 2);
    assert(nStopFrames > 0);

    muscle_t m = 0;
    joint_t j = 0;
    for (const auto &j_in : joint_inputs)
    {
        if (!j_in->show)
            continue;
        //assert(jointsBases[joint] < Point(1., 1.));
        assert(dMoveDistance == j_in->frames.dMoveDistance);
        assert(nMoveFrames == j_in->frames.nMoveFrames);
        assert(nStopFrames == static_cast<frames_t>(j_in->frames.nMoveFrames * j_in->frames.dInertiaRatio) + 2);

        auto pTJIn = dynamic_cast<const Tank::JointInput*>(j_in.get());
        Tank::Joint joint = pTJIn->Joint();

        jointsUsed[j++] = joint;
        musclesUsed[m++] = tank.MofJ(joint, true);
        musclesUsed[m++] = tank.MofJ(joint, false);
    }
}
//--------------------------------------------------------------------------------
void Tank::realMove()
{
#ifdef TANK_DEBUG
    r1_ = r2_ = 0;
    center_ = { 0.,0. };
#endif // TANK_DEBUG
    const distance_t shiftL = (status->shifts[LTrackFrw] - status->shifts[LTrackBck]) - Imoment(RTrack);
    const distance_t shiftR = (status->shifts[RTrackFrw] - status->shifts[RTrackBck]) - Imoment(LTrack);
    const distance_t between = boost_distance(currPos(LTrack), currPos(RTrack));
    if (betw != between)
        CDEBUG("betw1" << std::setprecision(6) << betw << " " << std::setprecision(6) << between);

    if (std::isnan(shiftL) || std::isinf(shiftL) ||
        std::isnan(shiftR) || std::isinf(shiftR))
        CERROR("shift NAN");

    Point center{}, normal{}; // tmp vars
    distance_t tan_angle = 0., radius = 0.;

    if (fabs(fabs(shiftL) - fabs(shiftR)) < RoboI::minFrameMove)
    {
        // если гусеницы имеют одинаковое смещение по модулю
        if (boost::math::sign(shiftL) != boost::math::sign(shiftR))
        {
            Point bodyCenterOld = currPos(Center);
            // если гусеницы крутятся в разные стороны - разворот на месте
            currPos(LTrack).rotate_radians(bodyCenterOld, std::atan(shiftL / between));
            currPos(RTrack).rotate_radians(bodyCenterOld, std::atan(shiftL / between));
            CDEBUG("1");
        }
        else
        {
            const auto shift = (shiftL + shiftR) / 2;
            // гусеницы крутятся в одну сторону - движение по прямой
            normal = currPos(LTrack) - currPos(RTrack); // нормаль
            normal = { -normal.y, normal.x };
            normal /= normal.norm2(); // единичная нормаль
            normal *= shift; // движение по прямой, длина
            //CDEBUG(normal);
            CDEBUG("2");
        }
    }
    else if (fabs(shiftL) > fabs(shiftR))
    {
        tan_angle = (shiftL - shiftR) / between;
        if (fabs(tan_angle) > 0./*eps?*/)
        {
            radius = fabs(shiftR / tan_angle);
            center = alongLineAtDistance(currPos(RTrack), currPos(LTrack), radius);
            CDEBUG("3");
#ifdef TANK_DEBUG
            r1_ = radius;
            r2_ = (shiftL / tan_angle);
            center_ = center;
#endif // TANK_DEBUG
        }
        else
        {
            // нормаль
            normal = currPos(LTrack) - currPos(RTrack);
            normal = { -normal.y, normal.x };
            normal /= normal.norm2();
            assert(shiftL == shiftR);
            normal *= shiftL;
            CDEBUG("4");
        }
    }
    else if (fabs(shiftL) < fabs(shiftR))
    {
        tan_angle = (shiftR - shiftL) / between;
        if (fabs(tan_angle) > 0./*eps?*/)
        {
            radius = fabs(shiftL / tan_angle);
            center = alongLineAtDistance(currPos(LTrack), currPos(RTrack), radius);
            CDEBUG("5");
#ifdef TANK_DEBUG
            r1_ = radius;
            r2_ = (shiftR / tan_angle);
            center_ = center;
#endif // TANK_DEBUG
        }
        else
        {
            // нормаль
            normal = currPos(LTrack) - currPos(RTrack);
            normal = { -normal.y, normal.x };
            normal /= normal.norm2();
            assert(shiftL == shiftR);
            normal *= shiftL;
            CDEBUG("6");
        }
    }
    else CERROR("Tank ELSE");

    if (fabs(tan_angle) > 0./*eps?*/)
    {
        currPos(LTrack).rotate_radians(center, +std::atan(tan_angle));
        currPos(RTrack).rotate_radians(center, -std::atan(tan_angle));
    }
    else
    {
        currPos(LTrack) += normal;
        currPos(RTrack) += normal;
    }
    if (betw != boost_distance(currPos(LTrack), currPos(RTrack)))
        CDEBUG("betw2" << std::setprecision(6) << betw << " " << std::setprecision(6) << boost_distance(currPos(LTrack), currPos(RTrack)));
    // =================
    currPos(Center) = (currPos(LTrack) + currPos(RTrack)) / 2.;
    // =================
    env->edges->interaction(getEnvCond() & EDGES, center, normal, tan_angle);
    // =================
    if (ba::all_of_equal(status->shifts, 0.))
    {
        for (muscle_t m = 0; m < musclesCount(); ++m)
            muscleDriveStop(m);
        status->moveEnd = true;
    }
    for (muscle_t muscle = 0; muscle < musclesCount(); ++muscle)
        status->shifts[muscle] = 0.;
}
//--------------------------------------------------------------------------------
void Tank::draw(IN HDC hdc, IN HPEN hPen, IN HBRUSH hBrush) const
{
#ifdef MY_WINDOW
    const Point &L = status->currPos[LTrack];
    const Point &R = status->currPos[RTrack];
    const Point &centerBody = status->currPos[Center];
    //------------------------------------------------------------------
    double phy = L.angle(R);
    // Body
    double bodyWidth = (boost_distance(R, L) - params.trackWidth);
    drawMyFigure(hdc, centerBody, bodyWidth, params.bodyHeight, phy, MyFigure::Rectangle, hPen);
    // Tracks
    for (joint_t j = 0; j < jointsCount(); ++j)
        drawMyFigure(hdc, status->currPos[j], params.trackWidth, params.trackHeight, phy, MyFigure::Rectangle, hPen);
    // Center
    drawCircle(hdc, centerBody, params.centerRadius);
    // Front
    drawMyFigure(hdc, centerBody, bodyWidth, params.bodyHeight, phy, MyFigure::Triangle, hPen);
    //------------------------------------------------------------------
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
    workSpace.push_back({ Point{ +0.97, +0.97 } });
    workSpace.push_back({ Point{ -0.97, +0.97 } });
    workSpace.push_back({ Point{ -0.97, -0.97 } });
    workSpace.push_back({ Point{ +0.97, -0.97 } });
    workSpace.push_back({ Point{ +0.97, +0.97 } });
}
//--------------------------------------------------------------------------------
void Tank::resetJoint(IN joint_t joint)
{
    muscleDriveStop(muscleByJoint(joint, true));
    muscleDriveStop(muscleByJoint(joint, false));
    setJoints({ { joint, 0. } });
}
void Tank::setJoints(IN const JointsOpenPercent &percents)
{
    for (const auto &jr : percents)
    {
        joint_t joint = jr.first;
        currPos(joint) = basePos(joint);
        /* =2.8 ~distance from up-left to down-right canvas-corner */
        status->shifts[joint] = (2.8 * jr.second / 100./*%*/);
    }
    currPos(Center) = (currPos(LTrack) + currPos(RTrack)) / 2;
    //realMove();
}
//--------------------------------------------------------------------------------
std::shared_ptr<RoboI> Tank::make(const tstring &type, tptree &node)
{
    if (type != Tank::name())
        return std::shared_ptr<RoboI>(nullptr);

    Point robo_base;
    JointsInputsPtrs robo_joints;
    robo_base.load(node.get_child(_T("base")));
    for (tptree::value_type &v : node.get_child(_T("joints")))
    {
        std::shared_ptr<Robo::JointInput> ji;
        ji = std::make_shared<Robo::Mobile::Tank::JointInput>();
        ji->load(v.second);
        robo_joints.push_back(ji);
    }
    robo_joints.sort([](const auto &a, const auto &b) { return (*a < *b); });
    auto r = std::make_shared<Tank>(robo_base, robo_joints);

    auto enviroment = node.get_optional<short>(_T("enviroment")).get_value_or(0);
    r->setEnvCond(static_cast<Enviroment>(enviroment));
    return r;
}

