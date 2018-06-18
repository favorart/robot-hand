#ifdef MY_WINDOW
#include "WindowHeader.h"
#include "WindowDraw.h"
#endif // MY_WINDOW
#include "RoboMuscles.h"
#include "RoboEdges.h"
#include "Hand.h"

using namespace Robo;
using namespace NewHand;
//--------------------------------------------------------------------------------
bool Hand::somethingMoving()
{
    return ba::any_of(status.musclesMove, [](const auto &m) { return (m > 0); });
}
//--------------------------------------------------------------------------------
frames_t Hand::muscleMaxLasts(const Control &control) const
{
    frames_t last = 0;
    for (const auto &c : control)
    {
        if (c.muscle >= musclesCount())
            throw std::logic_error("Invalid control");
        Joint joint = JofM(M(c.muscle));
        last = last ? std::min(last, physics.maxMoveFrames[joint]) : physics.maxMoveFrames[joint];
    }
    return last;
}
frames_t Hand::muscleMaxLasts(muscle_t muscle) const
{ 
    if (muscle >= musclesCount())
        throw std::logic_error("Invalid control");
    return physics.maxMoveFrames[JofM(M(muscle))];
}
//--------------------------------------------------------------------------------
void Hand::jointMove(joint_t joint_move, double offset)
{
    if (!offset) return;
    const Point &base = physics.jointsOpenCoords[Joint::JCount];
    const Point &prev = (joint_move + 1 == jointsCount()) ? base : status.curPos[J(joint_move + 1)];
    for (joint_t j = joint_move + 1; j > 0; --j)
    {
        Joint joint = J(j - 1);
        if (J(joint_move) == Joint::Clvcl)
            status.curPos[joint].x -= offset;
        else
            status.curPos[joint].rotate(prev, offset);
    }
    status.angles[J(joint_move)] += offset;
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
        const double Frame = (status.shifts[M(mc)] - status.shifts[M(mo)]) * 180. / M_PI;
        if (!Frame) continue;
        // =================
        const Joint joint = J(j);
        const double mOf = maxJointOffset(joint);
        // =================
        double offset = -Frame;
        offset = (0.0 > (status.angles[joint] + offset)) ? (0.0 - status.angles[joint]) : offset;
        offset = (mOf < (status.angles[joint] + offset)) ? (mOf - status.angles[joint]) : offset;
        // =================
        jointMove(j, offset);
        // =================
        if (!offset && status.shifts[M(mc)] && status.shifts[M(mo)])
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
void Hand::muscleDriveStop(muscle_t m)
{
    Muscle muscle = M(m);

    status.musclesMove[muscle] = 0;
    status.prevFrame[muscle] = 0;
    status.shifts[muscle] = 0;

    status.lastsMove[muscle] = 0;
    status.lastsStop[muscle] = 0;
    status.lasts[muscle] = 0;
}
bool Hand::muscleDriveFrame(muscle_t m)
{
    Muscle muscle = M(m);
    Joint joint = JofM(muscle);
    double Frame = 0.;
    //------------------------------------------------
    if (status.lastsMove[muscle] > 0)
    {
        auto last = status.lastsMove[muscle] - 1;
        const auto &frames = physics.framesMove[joint];

        do
        {
            Frame = (last >= frames.size()) ? frames.back() : frames[last];
        } while (status.prevFrame[muscle] >= Frame && ++last < frames.size());
    }
    else if (status.lastsStop[muscle] > 0)
    {
        auto last = status.lastsStop[muscle] - 1;
        const auto &frames = physics.framesStop[joint];

        do
        {
            Frame = (last >= frames.size()) ? frames.back() : frames[last];
        } while (status.prevFrame[muscle] < Frame && ++last < frames.size());
    }
    else throw std::logic_error("!lastsMove & !lastsStop");
    //------------------------------------------------
    status.shifts[muscle] = status.prevFrame[muscle] = Frame;
    // ??? status.velosity  -- momentum velosity
    if (status.windy && status.lastsMove[muscle] == 1)
        Frame = Utils::random(RoboI::minFrameMove, *boost::max_element(physics.framesMove[joint]));
    // -------------------------------------------
    if (std::isnan(status.shifts[muscle]) || std::isinf(status.shifts[muscle]))
    {
        CERROR("shift NAN");
        getchar();
        exit(1);
    }
    // -------------------------------------------
    return (fabs(Frame) > 0.);// RoboI::minFrameMove);
}
void Hand::muscleDriveMove(frames_t frame, muscle_t m, frames_t lasts)
{
    Muscle muscle = M(m);
    /* если не производится никакого движения и нет сигнала о начале нового */
    if (!lasts && !status.lastsMove[muscle] && !status.lastsStop[muscle])
        return;
    //-------------------------------------------------------
    status.moveEnd = false;
    //-------------------------------------------------------
    if (lasts)
    {
        if (status.lastsMove[muscle] == 0)
        {
            /* движение не было - начало нового движения */
            status.lastsStop[muscle] = 0;
            status.lastsMove[muscle] = 1;
            status.lasts[muscle] = lasts;

            if (!status.lastsStop[muscle])
                status.musclesMove[muscle] = frame;
        }
        else // if (status.lastsMove[muscle])
        {
            /* движение было - остановка по сигналу */
            status.lastsStop[muscle] = 1;
            status.lastsMove[muscle] = 0;
            status.lasts[muscle] = 0;
        }
    }
    //-------------------------------------------------------
    if (status.lastsMove[muscle] > 0)
    {
        if (/* остановка по истечении заявленной длительности */
            status.lasts[muscle] < status.lastsMove[muscle]
            /* продолжение движения, если остался на месте (блокировка противоположным мускулом) */
            || !muscleDriveFrame(m))
        {
            /* остановка основного движения - по истечении заданной длительности */
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
    else if (status.lastsStop[muscle] > 0)
    {
        /* Движение по инерции */
        if (!muscleDriveFrame(m))
        {
            muscleDriveStop(m);
            /* проверяем, что остальные двигатели уже остановились - полная остановка */
            if (ba::none_of(status.musclesMove, [](const auto &v) { return (v != 0); }))
                status.moveEnd = true;
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
Hand::Hand(IN const Point &baseClavicle, IN const std::list<JointInput> &joints) :
    params(joints),
    physics(baseClavicle, joints),
    status(joints)
{ 
    if (!joints.size() || joints.size() > JointsMaxCount)
        throw std::exception("Incorrect joints count");
    reset();
}

Hand::Hand(IN const Point &baseClavicle, IN const std::list<std::shared_ptr<Robo::JointInput>> &joints) :
    Hand(baseClavicle, JInputs<Hand::JointInput>(joints))
{}

Hand::Params::Params(IN const std::list<JointInput> &joints) :
    musclesUsedCount(0),
    jointsUsedCount(0),
    dynamics(false),
    oppHandle(false),
    drawPalm(false),
    palmRadius(0.05),
    jointRadius(0.03),
    sectionWidth(0.01)
{
    muscle_t m = 0;
    joint_t j = 0;
    for (auto& jInput : joints)
    {
        if (!jInput.show)
            continue;

        musclesUsedCount += 2;
        jointsUsedCount++;
        Joint joint = jInput.type;

        defOpen[joint] = jInput.defaultPose;
        jointsUsed[j++] = joint;

        musclesUsed[m++] = MofJ(joint, true);
        musclesUsed[m++] = MofJ(joint, false);

        if (joint == Joint::Wrist)
            drawPalm = true;
    }

    for (; m < MusclesMaxCount; ++m)
        musclesUsed[m] = MInvalid;
    for (; j < JointsMaxCount; ++j)
        jointsUsed[j] = JInvalid;

    std::sort(std::begin(musclesUsed), std::end(musclesUsed));
    std::sort(std::begin(jointsUsed), std::end(jointsUsed));
}

Hand::Physics::Physics(IN const Point &baseClavicle, IN const std::list<JointInput> &joints)
{
    jointsOpenCoords[JCount] = baseClavicle;
    for (const auto &jInput : joints)
    {
        if (!jInput.show)
            continue;

        Joint joint = jInput.type;
        auto law = jInput.frames;

        jointsOpenCoords[joint] = jInput.openCoords;
        jointsMaxAngles[joint] = jInput.maxAngle;
        maxMoveFrames[joint] = jInput.maxMoveFrames;
        minStopFrames[joint] = static_cast<frames_t>(maxMoveFrames[joint] * law.stopDistanceRatio);

        framesMove[joint].resize(maxMoveFrames[joint]);
        framesStop[joint].resize(minStopFrames[joint]);

        law.moveLaw->generate(framesMove[joint].begin(),
                              maxMoveFrames[joint],
                              Hand::minFrameMove,
                              jointsMaxAngles[joint]);

        double  maxVelosity = *boost::max_element(framesMove[joint]);
        law.stopLaw->generate(framesStop[joint].begin(),
                              minStopFrames[joint] - 1,
                              Hand::minFrameMove,
                              jointsMaxAngles[joint] * law.stopDistanceRatio,
                              maxVelosity);
        /* last frame must be 0 to deadend */
        framesStop[joint][minStopFrames[joint] - 1] = 0;
    }
}
//--------------------------------------------------------------------------------
void  Hand::reset()
{
    _reset();
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
void  Hand::resetJoint(IN joint_t joint)
{
    // TODO: ??? drop muscle status
    /* reset joint to default */
    setJoints({ {joint, params.defOpen[J(joint)]} });
}
//--------------------------------------------------------------------------------
void  Hand::setJoints(IN const JointsOpenPercent &percents)
{
    for (const auto &jr : percents)
    {
        Joint joint = J(jr.first);
        double ratio = jr.second / 100.;
        
        if (ratio > 1. || ratio < 0.)
            throw std::logic_error("Invalid joint set: must be 0 >= percent >= 100");

        double angle = ratio * maxJointOffset(joint);
        if (status.angles[joint] != angle)
            jointMove(jr.first, (angle - status.angles[joint]));
    }
}
//--------------------------------------------------------------------------------
void  Hand::draw(IN HDC hdc, IN HPEN hPen, IN HBRUSH hBrush) const
{
#ifdef MY_WINDOW
    HPEN   hPen_old = (HPEN)SelectObject(hdc, hPen);
    HBRUSH hBrush_old = (HBRUSH)SelectObject(hdc, hBrush);
    //------------------------------------------------------------------
    Point base = physics.jointsOpenCoords[Joint::Clvcl + 1];

    MoveToEx (hdc, Tx(1.00),   Ty(base.y + params.sectionWidth), NULL);
    LineTo   (hdc, Tx(base.x), Ty(base.y + params.sectionWidth));

    MoveToEx (hdc, Tx(1.00),   Ty(base.y - params.sectionWidth), NULL);
    LineTo   (hdc, Tx(base.x), Ty(base.y - params.sectionWidth));

    for (joint_t j = 0; j < jointsCount(); ++j)
    {
        Joint joint = J(j);

        const Point &B = status.curPos[joint];
        const Point &A = (j + 1 == jointsCount()) ? base : status.curPos[J(j + 1)];

        double sw = params.sectionWidth / boost_distance(A, B);

        Point up_c{ rotate(B + (A - B) * sw, B, -90.) }, up_n{ rotate(A + (B - A) * sw, A, +90.) };
        Point dn_c{ rotate(B + (A - B) * sw, B, +90.) }, dn_n{ rotate(A + (B - A) * sw, A, -90.) };
        
        MoveToEx (hdc, Tx(up_c.x), Ty(up_c.y), NULL);
        LineTo   (hdc, Tx(up_n.x), Ty(up_n.y));

        MoveToEx (hdc, Tx(dn_c.x), Ty(dn_c.y), NULL);
        LineTo   (hdc, Tx(dn_n.x), Ty(dn_n.y));

        drawCircle(hdc, status.curPos[joint], (joint == Joint::Wrist) ? params.palmRadius : params.jointRadius);
    }
    drawCircle(hdc, base, params.jointRadius);
    //------------------------------------------------------------------
    // отменяем ручку
    SelectObject(hdc, hPen_old);
    SelectObject(hdc, hBrush_old);
#endif // MY_WINDOW
}
//--------------------------------------------------------------------------------
void  Hand::getWorkSpace(OUT Trajectory &workSpace)
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

