#ifdef MY_WINDOW
#include "WindowHeader.h"
#include "WindowDraw.h"
#endif // MY_WINDOW
#include "RoboMuscles.h"
#include "RoboInputs.h"
#include "RoboEdges.h"
#include "RoboPhysicsStatus.h"
#include "Hand.h"

using namespace Robo;
using namespace NewHand;
//--------------------------------------------------------------------------------
frames_t Hand::muscleMaxLasts(const Control &control) const
{
    frames_t lasts = 0;
    for (const auto &c : control)
    {
        if (c.muscle >= musclesCount())
            throw std::logic_error("Invalid control");
        joint_t joint = jointByMuscle(c.muscle);
        lasts = lasts ? std::min(lasts, params.nMoveFrames[joint]) : params.nMoveFrames[joint];
    }
    return lasts;
}
frames_t Hand::muscleMaxLasts(muscle_t muscle) const
{ 
    if (muscle >= musclesCount())
        throw std::logic_error("Invalid control");
    return params.nMoveFrames[jointByMuscle(muscle)];
}
//--------------------------------------------------------------------------------
void Hand::jointMove(joint_t joint_move, double offset)
{
    if (fabs(offset) < RoboI::minFrameMove)
        return;
    const Point &prev = (joint_move + 1 == jointsCount()) ? basePos(jointsCount()) : currPos(joint_move + 1);
    for (joint_t j = joint_move + 1; j > 0; --j)
    {
        if (J(joint_move) == Joint::Clvcl)
            currPos(j - 1).x -= offset;
        else
            currPos(j - 1).rotate_radians(prev, offset);
    }
    angles[joint_move] += offset;
}
//--------------------------------------------------------------------------------
void Hand::realMove()
{
    bool move = false;
    // =================
    for (joint_t j = 0; j < jointsCount(); ++j)
    {
        const auto mo = muscleByJoint(j, true);
        const auto mc = muscleByJoint(j, false);
        // =================
        double offset = -(status->shifts[mc] - status->shifts[mo]) + Imoment(j);
        if (fabs(offset) < RoboI::minFrameMove)
            continue;
        // =================
        const double mAn = maxJointAngle(j);
        // =================
        offset = (0.0 > (angles[j] + offset)) ? (0.0 - angles[j]) : offset;
        offset = (mAn < (angles[j] + offset)) ? (mAn - angles[j]) : offset;
        // =================
        jointMove(j, offset);
        // =================
        if (fabs(offset) >= RoboI::minFrameMove)
            move = true;
    }
    // =================
    env->edges->interaction(getEnvCond() & EDGES);
    // =================
    if (!move)
    {
        for (muscle_t m = 0; m < musclesCount(); ++m)
            muscleDriveStop(m);
        status->moveEnd = true;
    }
    // =================
    status->shifts.fill(0);
}
//--------------------------------------------------------------------------------
Hand::Hand(const Point &base, const JointsInputsPtrs &joints) :
    RoboPhysics(base, joints, std::make_shared<Robo::EnvEdgesHand>(*this, 20, 2)),
    params(joints, *this)
{
    if (!joints.size() || joints.size() > Hand::joints)
        throw std::logic_error("Hand: Incorrect joints inputs size");
}
//--------------------------------------------------------------------------------
Hand::Params::Params(const JointsInputsPtrs &joints, const Hand &hand) :
    drawPalm(false),
    palmRadius(0.05),
    jointRadius(0.03),
    sectionWidth(0.01)
{
    jointsUsed.fill(Hand::Joint::JInvalid);
    musclesUsed.fill(Hand::Muscle::MInvalid);

    muscle_t m = 0;
    joint_t j = 0;
    for (auto& j_in : joints)
    {
        if (!j_in->show)
            continue;

        auto pHJIn = dynamic_cast<const Hand::JointInput*>(j_in.get());
        Hand::Joint joint = pHJIn->Joint();

        maxAngles[j] = j_in->frames.dMoveDistance;
        nStopFrames[j] = static_cast<frames_t>(j_in->frames.nMoveFrames * j_in->frames.dInertiaRatio);
        nMoveFrames[j] = j_in->frames.nMoveFrames;
        defOpen[j] = pHJIn->defaultPose;

        jointsUsed[j++] = joint;
        musclesUsed[m++] = hand.MofJ(joint, true);
        musclesUsed[m++] = hand.MofJ(joint, false);

        if (joint == Hand::Joint::Wrist)
            drawPalm = true;
    }
}
//--------------------------------------------------------------------------------
void Hand::resetJoint(joint_t joint)
{
    muscleDriveStop(muscleByJoint(joint, true));
    muscleDriveStop(muscleByJoint(joint, false));
    setJoints({ {joint, params.defOpen[joint]} });
}
void Hand::setJoints(const JointsOpenPercent &percents)
{
    for (const auto &jr : percents)
    {
        joint_t joint = jr.first;
        double ratio = jr.second / 100.; //%
        
        if (ratio > 1. || ratio < 0.)
            throw std::logic_error("Invalid joint set: must be 0 >= percent >= 100%");

        double angle = ratio * maxJointAngle(joint);
        jointMove(joint, (angle - angles[joint]));
    }
}
//--------------------------------------------------------------------------------
void Hand::draw(IN HDC hdc, IN HPEN hPen, IN HBRUSH hBrush) const
{
#ifdef MY_WINDOW
    HPEN   hPen_old = (HPEN)SelectObject(hdc, hPen);
    HBRUSH hBrush_old = (HBRUSH)SelectObject(hdc, hBrush);
    //------------------------------------------------------------------
    const Point &base = basePos(jointsCount());

    MoveToEx (hdc, Tx(1.00),   Ty(base.y + params.sectionWidth), NULL);
    LineTo   (hdc, Tx(base.x), Ty(base.y + params.sectionWidth));

    MoveToEx (hdc, Tx(1.00),   Ty(base.y - params.sectionWidth), NULL);
    LineTo   (hdc, Tx(base.x), Ty(base.y - params.sectionWidth));
    //------------------------------------------------------------------
    for (joint_t joint = 0; joint < jointsCount(); ++joint)
    {
        const Point &B = status->currPos[joint];
        const Point &A = ((joint + 1) == jointsCount()) ? base : status->currPos[joint + 1];

        double sw = params.sectionWidth / boost_distance(A, B);

        Point up_c{ rotate(B + (A - B) * sw, B, -90.) }, up_n{ rotate(A + (B - A) * sw, A, +90.) };
        Point dn_c{ rotate(B + (A - B) * sw, B, +90.) }, dn_n{ rotate(A + (B - A) * sw, A, -90.) };
        
        MoveToEx (hdc, Tx(up_c.x), Ty(up_c.y), NULL);
        LineTo   (hdc, Tx(up_n.x), Ty(up_n.y));

        MoveToEx (hdc, Tx(dn_c.x), Ty(dn_c.y), NULL);
        LineTo   (hdc, Tx(dn_n.x), Ty(dn_n.y));

        drawCircle(hdc, B, (J(joint) == Joint::Wrist) ? params.palmRadius : params.jointRadius);
    }
    drawCircle(hdc, base, params.jointRadius);
    //------------------------------------------------------------------
    // отменяем ручку
    SelectObject(hdc, hPen_old);
    SelectObject(hdc, hBrush_old);
#endif // MY_WINDOW
}
//--------------------------------------------------------------------------------
void Hand::getWorkSpace(OUT Trajectory &workSpace)
{
    for (joint_t joint = 0; joint < jointsCount(); ++joint)
        setJoints({ { joint , 0. } });
    Muscle sequence[] = {
        Muscle::ClvclCls, Muscle::ShldrCls, Muscle::ClvclOpn,
        Muscle::ElbowCls, Muscle::WristCls, Muscle::ShldrOpn,
        Muscle::ElbowOpn, Muscle::WristOpn
    };
    Control controls;
    frames_t prev = 0;
    for (Muscle muscle : sequence)
    {
        auto &used = params.musclesUsed;
        auto it = std::find(used.begin(), used.end(), muscle);
        if (it != used.end())
        {
            auto m = static_cast<muscle_t>(it - used.begin());
            controls.append({ m, prev, muscleMaxLasts(m) });
            prev += muscleMaxLasts(m);
        }
    }
    CINFO("Workspace:" << controls);
    move(controls, LastsInfinity);
    std::transform(trajectory().begin(), trajectory().end(),
                   std::back_inserter(workSpace),
                   [](const auto &state) { return state.spec(); });
    reset();
}
//--------------------------------------------------------------------------------
std::shared_ptr<RoboI> Hand::make(const tstring &type, tptree &node)
{
    if (type != Hand::name())
        return std::shared_ptr<RoboI>(nullptr);

    Point robo_base;
    JointsInputsPtrs robo_joints;
    robo_base.load(node.get_child(_T("base")));
    for (tptree::value_type &v : node.get_child(_T("joints")))
    {
        std::shared_ptr<Robo::JointInput> ji;
        ji = std::make_shared<Robo::NewHand::Hand::JointInput>();
        ji->load(v.second);
        robo_joints.push_back(ji);
    }
    robo_joints.sort([](const auto &a, const auto &b) { return (*a < *b); });
    auto r = std::make_shared<Hand>(robo_base, robo_joints);

    auto enviroment = node.get_optional<short>(_T("enviroment")).get_value_or(0);
    r->setEnvCond(static_cast<Enviroment>(enviroment)); // <KZLM!
    return r;
}
