﻿#ifdef MY_WINDOW
#include "WindowHeader.h"
#include "WindowDraw.h"
#endif // MY_WINDOW
#include "RoboMuscles.h"
#include "RoboInputs.h"
#include "RoboEdges.h"
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
    if (!offset) return;
    const Point &base = physics.jointsBases[jointsCount()]; // jointsOpenCoords
    const Point &prev = (joint_move + 1 == jointsCount()) ? base : status.curPos[joint_move + 1];
    for (joint_t j = joint_move + 1; j > 0; --j)
    {
        if (J(joint_move) == Joint::Clvcl)
            status.curPos[j - 1].x -= offset;
        else
            status.curPos[j - 1].rotate_radians(prev, offset);
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
        const double Frame = (status.shifts[mc] - status.shifts[mo]);
        if (!Frame) continue;
        // =================
        const double mAn = maxJointAngle(j);
        // =================
        double offset = -Frame;
        offset = (0.0 > (angles[j] + offset)) ? (0.0 - angles[j]) : offset;
        offset = (mAn < (angles[j] + offset)) ? (mAn - angles[j]) : offset;
        // =================
        jointMove(j, offset);
        // =================
        if (!offset && status.shifts[mc] && status.shifts[mo])
        {
            // ??? turn off mutual BLOCKing
        }
        if (offset) move = true;
    }
    // =================
    feedback.currVelAcc(jointsCount(), status.curPos);
    // =================
    env.edges->interaction();
    // =================
    if (!move)
    {
        for (muscle_t m = 0; m < musclesCount(); ++m)
            muscleDriveStop(m);
        status.moveEnd = true;
    }
    // =================
    status.shifts.fill(0);
}
//--------------------------------------------------------------------------------
Hand::Hand(const Point &base, const JointsInputsPtrs &joints, bool edges) :
    RoboPhysics(base, joints, std::make_shared<Robo::EnvEdgesHand>(*this, edges)),
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

        maxAngles[j] = j_in->maxMoveFrame;
        nStopFrames[j] = static_cast<frames_t>(j_in->nMoveFrames * j_in->frames.stopDistanceRatio);
        nMoveFrames[j] = j_in->nMoveFrames;
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
        double ratio = jr.second / 100.;
        
        if (ratio > 1. || ratio < 0.)
            throw std::logic_error("Invalid joint set: must be 0 >= percent >= 100");

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
    Point base = physics.jointsBases[jointsCount()];

    MoveToEx (hdc, Tx(1.00),   Ty(base.y + params.sectionWidth), NULL);
    LineTo   (hdc, Tx(base.x), Ty(base.y + params.sectionWidth));

    MoveToEx (hdc, Tx(1.00),   Ty(base.y - params.sectionWidth), NULL);
    LineTo   (hdc, Tx(base.x), Ty(base.y - params.sectionWidth));

    //------------------------------------------------------------------
    for (joint_t joint = 0; joint < jointsCount(); ++joint)
    {
        const Point &B = status.curPos[joint];
        const Point &A = ((joint + 1) == jointsCount()) ? base : status.curPos[joint + 1];

        double sw = params.sectionWidth / boost_distance(A, B);

        Point up_c{ rotate(B + (A - B) * sw, B, -90.) }, up_n{ rotate(A + (B - A) * sw, A, +90.) };
        Point dn_c{ rotate(B + (A - B) * sw, B, +90.) }, dn_n{ rotate(A + (B - A) * sw, A, -90.) };
        
        MoveToEx (hdc, Tx(up_c.x), Ty(up_c.y), NULL);
        LineTo   (hdc, Tx(up_n.x), Ty(up_n.y));

        MoveToEx (hdc, Tx(dn_c.x), Ty(dn_c.y), NULL);
        LineTo   (hdc, Tx(dn_n.x), Ty(dn_n.y));

        drawCircle(hdc, status.curPos[joint], (J(joint) == Joint::Wrist) ? params.palmRadius : params.jointRadius);
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

    // TODO: for (muscle_t muscle = 0; muscle < musclesCount(); ++muscle) {}
    Muscle sequence[] = {
        Muscle::ClvclOpn, Muscle::ShldrCls, Muscle::ClvclCls,
        Muscle::ElbowCls, Muscle::WristCls, Muscle::ShldrOpn,
        Muscle::ElbowOpn, Muscle::WristOpn
    };
    for (Muscle muscle : sequence)
    {
        auto &used = params.musclesUsed;
        auto it = std::find(used.begin(), used.end(), muscle);
        if (it != used.end())
        {
            muscle_t m = (it - used.begin());
            Control control{ { m, 0, muscleMaxLasts(m) } };
            //CINFO(control);
            move(control);
            workSpace = trajectory();
        }
    }
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
    bool edges = node.get_optional<bool>(_T("edges")).get_value_or(true);
    return std::make_shared<Hand>(robo_base, robo_joints, edges);
}

