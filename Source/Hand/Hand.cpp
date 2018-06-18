#ifdef MY_WINDOW
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
        Joint joint = J(j - 1);
        if (J(joint_move) == Joint::Clvcl)
            status.curPos[j].x -= offset;
        else
            status.curPos[j].rotate(prev, offset);
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
        const double Frame = (status.shifts[mc] - status.shifts[mo]) * 180. / M_PI;
        if (!Frame) continue;
        // =================
        const double mOf = maxJointOffset(j);
        // =================
        double offset = -Frame;
        offset = (0.0 > (angles[j] + offset)) ? (0.0 - angles[j]) : offset;
        offset = (mOf < (angles[j] + offset)) ? (mOf - angles[j]) : offset;
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
Hand::Hand(const Point &base, const JointsPInputs &joints) :
    RoboPhysics(base, joints, std::make_shared<Robo::EnvEdgesHand>()),
    params(joints, *this)
{
    if (!joints.size() || joints.size() > Hand::joints)
        throw std::logic_error("Hand: Incorrect joints inputs size");
    reset();
}
//--------------------------------------------------------------------------------
Hand::Params::Params(const JointsPInputs &joints, const Hand &hand) :
    drawPalm(false),
    palmRadius(0.05),
    jointRadius(0.03),
    sectionWidth(0.01)
{
    muscle_t m = 0;
    joint_t j = 0;
    for (auto& j_in : joints)
    {
        if (!j_in->show)
            continue;

        joint_t joint = j_in->type;

        maxAngles[joint] = j_in->maxMoveFrame;
        nStopFrames[joint] = static_cast<frames_t>(j_in->nMoveFrames * j_in->frames.stopDistanceRatio);
        nMoveFrames[joint] = j_in->nMoveFrames;

        auto pHJIn = dynamic_cast<const Hand::JointInput*>(j_in.get());
        
        defOpen[joint] = pHJIn->defaultPose;
        jointsUsed[j++] = pHJIn->joint;

        musclesUsed[m++] = hand.MofJ(pHJIn->joint, true);
        musclesUsed[m++] = hand.MofJ(pHJIn->joint, false);

        if (hand.J(joint) == Hand::Joint::Wrist)
            drawPalm = true;
    }

    for (; m < Hand::muscles; ++m)
        musclesUsed[m] = Hand::Muscle::MInvalid;
    for (; j < Hand::joints; ++j)
        jointsUsed[j] = Hand::Joint::JInvalid;
}
//--------------------------------------------------------------------------------
void Hand::reset()
{
    _reset();
    /* drop status */
    for (muscle_t m = 0; m < musclesCount(); ++m)
    {
        muscleDriveStop(m);
        //status.acceleration[m] = 0.;
        //status.velosity[m] = 0.;
    }
    //-----------------------------------------------------
    status.moveEnd = false;
    //-----------------------------------------------------
    for (joint_t joint = 0; joint < jointsCount(); ++joint)
    {
        /* make it at resetJoint(): status[ curPos, angles ] */
        resetJoint(joint);
    }
}
//--------------------------------------------------------------------------------
void Hand::resetJoint(joint_t joint)
{
    // TODO: ??? drop muscle status
    /* reset joint to default */
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

        double angle = ratio * maxJointOffset(joint);
        if (angles[joint] != angle)
            jointMove(joint, (angle - angles[joint]));
    }
}
//--------------------------------------------------------------------------------
void Hand::draw(IN HDC hdc, IN HPEN hPen, IN HBRUSH hBrush) const
{
#ifdef MY_WINDOW
    HPEN   hPen_old = (HPEN)SelectObject(hdc, hPen);
    HBRUSH hBrush_old = (HBRUSH)SelectObject(hdc, hBrush);

    Point base = physics.jointsBases[jointsCount()];

    MoveToEx (hdc, Tx(1.00),   Ty(base.y + params.sectionWidth), NULL);
    LineTo   (hdc, Tx(base.x), Ty(base.y + params.sectionWidth));

    MoveToEx (hdc, Tx(1.00),   Ty(base.y - params.sectionWidth), NULL);
    LineTo   (hdc, Tx(base.x), Ty(base.y - params.sectionWidth));

    for (joint_t joint = 0; joint < jointsCount(); ++joint)
    {
        const Point &B = status.curPos[joint];
        const Point &A = (J(joint + 1) == Joint::JBase) ? base : status.curPos[joint + 1];

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

