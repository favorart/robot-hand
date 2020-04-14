#ifdef MY_WINDOW
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
//--------------------------------------------------------------------------------
frames_t Tank::muscleMaxLasts(const Control &/*control*/) const
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
frames_t Tank::muscleMaxLasts(muscle_t /*muscle*/) const
{
    //return (physics.framesMove[jointByMuscle(muscle)].size() - 1);
    return Robo::LastsInfinity;
}
//--------------------------------------------------------------------------------
Tank::Tank(const Point &base, const JointsInputsPtrs &joint_inputs) :
    RoboPhysics(base, joint_inputs, std::make_shared<EnvEdgesTank>(*this, 10/*%*/, 1.5)),
    params(joint_inputs, *this)
{
    if (params.jointsUsed[0] != Joint::LTrack || params.jointsUsed[1] != Joint::RTrack)
        throw std::logic_error("Invalid tracks numeration");
    if (!joint_inputs.size() || joint_inputs.size() > jointsCount())
        throw std::logic_error("Incorrect joint_inputs count");
    //reset();
#ifdef TANK_DEBUG
    betw = boost_distance(currPos(LTrack), currPos(RTrack));
#endif // TANK_DEBUG
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

#ifdef _DEBUG
    const auto &front = *joint_inputs.front();
    const double dMoveDistance = front.frames.dMoveDistance;
    const frames_t nMoveFrames = front.frames.nMoveFrames;
    const frames_t nStopFrames = static_cast<frames_t>(front.frames.nMoveFrames * front.frames.dInertiaRatio + 2);
#endif

    assert(dMoveDistance > RoboI::minFrameMove);
    assert(nMoveFrames > 0);
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
bool Tank::realMove()
{
    const distance_t shiftL = jointShift(LTrack);
    const distance_t shiftR = jointShift(RTrack);
    const distance_t between = boost_distance(currPos(LTrack), currPos(RTrack));

#ifdef TANK_DEBUG
    r1_ = r2_ = 0;
    center_ = { 0.,0. };
    if (betw != between)
        CDEBUG("betw1" << std::setprecision(6) << betw << " " << std::setprecision(6) << between);
#endif // TANK_DEBUG

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
#ifdef TANK_DEBUG
    if (betw != boost_distance(currPos(LTrack), currPos(RTrack)))
        CDEBUG("betw2" << std::setprecision(6) << betw << " " << std::setprecision(6) << boost_distance(currPos(LTrack), currPos(RTrack)));
#endif // TANK_DEBUG

    currPos(Center) = (currPos(LTrack) + currPos(RTrack)) / 2.;
    return (shiftL == 0. && shiftR == 0.);
}
//--------------------------------------------------------------------------------
void Tank::draw(IN HDC hdc, IN HPEN hPen, IN HBRUSH hBrush) const
{
#ifdef MY_WINDOW
    const Point &L = jointPos(LTrack);
    const Point &R = jointPos(RTrack);
    const Point &centerBody = curPos(Center);
    //------------------------------------------------------------------
    double phy = L.angle(R);
    // Body
    double bodyWidth = (boost_distance(R, L) - params.trackWidth);
    drawMyFigure(hdc, centerBody, bodyWidth, params.bodyHeight, phy, MyFigure::Rectangle, hPen);
    // Tracks
    for (joint_t j = 0; j < jointsCount(); ++j)
        drawMyFigure(hdc, jointPos(j), params.trackWidth, params.trackHeight, phy, MyFigure::Rectangle, hPen);
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
void Tank::reset()
{
    RoboPhysics::reset();
    //for (joint_t joint = 0; joint <= jointsCount(); ++joint)
    //    currPos(joint) = basePos(joint);
}
void Tank::resetJoint(IN joint_t joint)
{
    RoboPhysics::resetJoint(joint);
    setJoints({ { joint, 0. } });
}
void Tank::setJoints(IN const JointsOpenPercent &percents)
{
    for (const auto &jr : percents)
    {
        joint_t joint = jr.first;
        currPos(joint) = basePos(joint);
        /* =2.8 ~distance from up-left to down-right canvas-corner */
        //status->shifts[joint] = (2.8 * jr.second / 100./*%*/);       //TODO: !!!
    }
    //currPos(Center) = basePos(Center);
    currPos(Center) = (currPos(LTrack) + currPos(RTrack)) / 2;
    //realMove(); //TODO: !!!
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

    tstring s = node.get_optional<tstring>(_T("enviroment")).get_value_or(_T(""));
    r->setEnvCond(scanEnviroment(s));
    return r;
}

