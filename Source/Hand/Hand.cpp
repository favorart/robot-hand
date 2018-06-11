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
frames_t Hand::muscleMaxLast(const Control &control) const
{
    frames_t last = 0;
    for (const auto &c : control)
    {
        if (c.muscle >= musclesCount())
            throw std::logic_error("Invalid control");
        joint_t joint = jointByMuscle(c.muscle);
        last = last ? std::min(last, params.nMoveFrames[joint]) : params.nMoveFrames[joint];
    }
    return last;
}
frames_t Hand::muscleMaxLast(muscle_t muscle) const
{ 
    if (muscle >= musclesCount())
        throw std::logic_error("Invalid control");
    return params.nMoveFrames[jointByMuscle(muscle)];
}
//--------------------------------------------------------------------------------
void Hand::jointMove(joint_t joint, double offset)
{
    const Point &base = physics.jointsBases[jointsCount()]; // jointsOpenCoords
    const Point &prev = (J(joint + 1) == Joint::JBase) ? base : status.curPos[joint + 1];
    for (joint_t j = joint + 1; j > 0; --j)
    {
        if (J(j - 1) == Joint::Clvcl)
            status.curPos[j].x -= offset;
        else
            status.curPos[j].rotate(prev, offset);
    }
    angles[joint] += offset;
}
//--------------------------------------------------------------------------------
void Hand::realMove()
{
    for (joint_t joint = 0; joint < jointsCount(); ++joint)
    {
        auto mo = muscleByJoint(joint, true);
        auto mc = muscleByJoint(joint, false);
        double Frame = (status.shifts[mo] - status.shifts[mc]) * 180. / M_PI;
        
        double maxOffset = static_cast<double>(params.maxAngles[joint]) / ((J(joint) == Joint::Clvcl) ? 100. : 1.);

        double offset = 0.;
        switch (J(joint))
        {
        case Joint::Clvcl: offset = ((angles[joint] + Frame < 0. || angles[joint] + Frame > maxOffset) ? 0. : +Frame); break;
        case Joint::Shldr: offset = ((angles[joint] - Frame < 1. || angles[joint] - Frame > maxOffset) ? 0. : -Frame); break;
        case Joint::Elbow: offset = ((angles[joint] - Frame < 1. || angles[joint] - Frame > maxOffset) ? 0. : -Frame); break;
        case Joint::Wrist: offset = ((angles[joint] - Frame < 1. || angles[joint] - Frame > maxOffset) ? 0. : -Frame); break;
        default: std::logic_error("Invalid joint");
        }
        //if (fabs(offset) < Hand::minFrameMove) continue;
        jointMove(joint, offset);
    }
}

//--------------------------------------------------------------------------------
Hand::Hand(const Point &base, const JointsPInputs &joints) :
    RoboPhysics(base, joints, std::make_shared<Robo::EnvEdgesHand>()),
    params(joints, *this)
{
    if (!joints.size() || joints.size() > Hand::Joint::JCount)
        throw std::logic_error("Incorrect joints count");
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

        auto joint = j_in->type;

        maxAngles[joint] = j_in->maxMoveFrame;
        nStopFrames[joint] = static_cast<frames_t>(j_in->nMoveFrames * j_in->frames.stopDistanceRatio);
        nMoveFrames[joint] = j_in->nMoveFrames;

        auto pHJIn = dynamic_cast<const Hand::JointInput*>(j_in.get());
        
        defOpen[joint] = pHJIn->defaultPose;
        jointsUsed[j++] = pHJIn->joint;

        musclesUsed[m++] = hand.MofJ(pHJIn->joint, true);
        musclesUsed[m++] = hand.MofJ(pHJIn->joint, false);

        if (joint == Hand::Joint::Wrist)
            drawPalm = true;
    }

    for (; m < Hand::MCount; ++m)
        musclesUsed[m] = Hand::MInvalid;
    for (; j < Hand::JCount; ++j)
        jointsUsed[j] = Hand::JInvalid;
}

//--------------------------------------------------------------------------------
void Hand::reset()
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
        /* make it at resetJoint(): status[ curPos, angles ] */
        resetJoint(joint);
    }
}
//--------------------------------------------------------------------------------
void Hand::resetJoint(joint_t joint)
{
    // TODO: ??? drop muscle status
    /* reset joint to default */
    setJoints({ {joint, params.defOpen[J(joint)]} });
}
//--------------------------------------------------------------------------------
void Hand::setJoints(const JointsOpenPercent &percents)
{
    for (const auto &jr : percents)
    {
        joint_t joint = jr.first;
        double ratio = jr.second / 100.;
        
        if (ratio > 1. || ratio < 0.)
            throw std::logic_error("Invalid joint set: must be 0 >= percent >= 100");

        double maxAngle = static_cast<double>(params.maxAngles[J(joint)]) / ((joint == Hand::Joint::Clvcl) ? 100. : 1.);

        double angle = ratio * maxAngle;
        if (angles[joint] != angle)
        {
            /* status[ curPos, angles ] */
            jointMove(joint, (angle - angles[joint]));
        }
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
            Control control{ { m, 0, muscleMaxLast(m) } };
            //CINFO(control);
            move(control, workSpace);
        }
    }
    reset();
}
//--------------------------------------------------------------------------------
