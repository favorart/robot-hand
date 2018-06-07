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
frames_t  Hand::muscleMaxLast(const Control &control) const
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
frames_t  Hand::muscleMaxLast(muscle_t muscle) const
{ 
    if (muscle >= musclesCount())
        throw std::logic_error("Invalid control");
    return physics.maxMoveFrames[JofM(M(muscle))];
}
//--------------------------------------------------------------------------------
void    Hand::jointMove   (joint_t joint_move, double offset)
{
    //Joint joint = J(joint_move);
    //switch (joint)
    //{
    //case Joint::Clvcl:
    //{
    //    status.curPos[Joint::Clvcl].x -= offset; // curPosShldr_
    //    status.curPos[Joint::Shldr].x -= offset; // curPosArm_
    //    status.curPos[Joint::Elbow].x -= offset; // curPosHand_
    //    status.curPos[Joint::Wrist].x -= offset; // curPosPalm_
    //    status.angles[Joint::Clvcl] += offset;
    //    break;
    //}
    //case Joint::Shldr:
    //{
    //    status.curPos[Joint::Shldr].rotate(status.curPos[Joint::Clvcl], offset);
    //    status.curPos[Joint::Elbow].rotate(status.curPos[Joint::Clvcl], offset);
    //    status.curPos[Joint::Wrist].rotate(status.curPos[Joint::Clvcl], offset);
    //    status.angles[Joint::Shldr] += offset;
    //    break;
    //}
    //case Joint::Elbow:
    //{
    //    status.curPos[Joint::Elbow].rotate(status.curPos[Joint::Shldr], offset);
    //    status.curPos[Joint::Wrist].rotate(status.curPos[Joint::Shldr], offset);
    //    status.angles[Joint::Elbow] += offset;
    //    break;
    //}
    //case Joint::Wrist:
    //{
    //    status.curPos[Joint::Wrist].rotate(status.curPos[Joint::Elbow], offset);
    //    status.angles[Joint::Wrist] += offset;
    //    break;
    //}
    //}

    const Point &base = physics.jointsOpenCoords[Joint::JCount];
    const Point &prev = (joint_move + 1 == jointsCount()) ? base : status.curPos[J(joint_move + 1)];
    for (joint_t j = joint_move + 1; j > 0; --j)
    {
        Joint joint = J(j-1);
        if (joint == Joint::Clvcl)
            status.curPos[joint].x -= offset;
        else
            status.curPos[joint].rotate(prev, offset);
    }
    status.angles[J(joint_move)] += offset;
}
//--------------------------------------------------------------------------------
bool    Hand::muscleFrame (muscle_t m)
{
    Muscle muscle = M(m);
    Joint joint = JofM(muscle);
    double Frame = 0.;
    //------------------------------------------------
    if (status.lastsMove[muscle] > 0)
    {
        assert(status.lastsStop[muscle] == 0);

        auto last = status.lastsMove[muscle] - 1;
        const auto &frames = physics.framesMove[joint];

        do
        {
            Frame = (last >= frames.size()) ? frames.back() : frames[last];
        } while (status.prevFrame[muscle] >= Frame && ++last < frames.size());
    }
    else if (status.lastsStop[muscle] > 0)
    {
        assert(status.lastsMove[muscle] == 0);

        auto last = status.lastsStop[muscle] - 1;
        const auto &frames = physics.framesStop[joint];

        do
        {
            Frame = (last >= frames.size()) ? frames.back() : frames[last];
        } while (status.prevFrame[muscle] < Frame && ++last < frames.size());
    }
    else throw std::logic_error("!lastsMove & !lastsStop");
    // /* next frame with stop speed smooth */
    //do
    //{
    //    // TODO: удлинить массив доступных фрэймов движения? - незачем, т.к. 
    //    //       до конца прохода по массиву произойдёт удар по ограничению
    //    if (status.lastsStop[muscle] < physics.minStopFrames[joint])
    //    {
    //        Frame = 0.;
    //        break;
    //    }
    //    Frame = ((atStop) ? (physics.framesStop[joint][status.lastsStop[muscle]++]) :
    //        (physics.framesMove[joint][status.lastsMove[muscle]++]));
    //
    //} while (atStop && status.prevFrame[muscle] < Frame);
    //------------------------------------------------
    status.prevFrame[muscle] = Frame;
    // ??? status.velosity  -- momentum velosity

    // --- WIND ---
    if (status.windy && status.lastsMove[muscle] == 1)
        Frame = Utils::random(Hand::minFrameMove * 180. / M_PI, *boost::max_element(physics.framesMove[joint]));
    // -------------------------------------------
    // maxOffset
    double mOf = (joint == Joint::Clvcl) ?
        (static_cast<double>(physics.jointsMaxAngles[Joint::Clvcl]) / 100.) :
                             physics.jointsMaxAngles[joint];

    double offset = 0.;
    switch (muscle)
    {
    case Muscle::ClvclOpn: offset = (status.angles[Joint::Clvcl] + Frame > mOf) ? 0. /*(mOf - status.angles[Joint::Clvcl])*/ : +Frame; break;
    case Muscle::ClvclCls: offset = (status.angles[Joint::Clvcl] - Frame < 0.0) ? 0. /*(0.0 - status.angles[Joint::Clvcl])*/ : -Frame; break;
    case Muscle::ShldrOpn: offset = (status.angles[Joint::Shldr] - Frame < 1.0) ? 0. /*(1.0 - status.angles[Joint::Shldr])*/ : -Frame; break;
    case Muscle::ShldrCls: offset = (status.angles[Joint::Shldr] + Frame > mOf) ? 0. /*(mOf - status.angles[Joint::Shldr])*/ : +Frame; break;
    case Muscle::ElbowOpn: offset = (status.angles[Joint::Elbow] - Frame < 1.0) ? 0. /*(1.0 - status.angles[Joint::Elbow])*/ : -Frame; break;
    case Muscle::ElbowCls: offset = (status.angles[Joint::Elbow] + Frame > mOf) ? 0. /*(mOf - status.angles[Joint::Elbow])*/ : +Frame; break;
    case Muscle::WristOpn: offset = (status.angles[Joint::Wrist] - Frame < 1.0) ? 0. /*(1.0 - status.angles[Joint::Wrist])*/ : -Frame; break;
    case Muscle::WristCls: offset = (status.angles[Joint::Wrist] + Frame > mOf) ? 0. /*(mOf - status.angles[Joint::Wrist])*/ : +Frame; break;
    }

    jointMove(jointByMuscle(m), offset);
    return (fabs(offset) >= Hand::minFrameMove);
}
//--------------------------------------------------------------------------------
void    Hand::muscleMove  (frames_t frame, muscle_t m, frames_t last)
{
    Muscle muscle = M(m);
    /* если не производится никакого движения и нет сигнала о начале нового */
    if (!last && !status.lastsMove[muscle] && !status.lastsStop[muscle])
        return;
    //-------------------------------------------------------
    status.moveEnd = false;
    //-------------------------------------------------------
    if (last)
    {
        if (status.lastsMove[muscle] == 0)
        {
            /* движение не было - начало нового движения */
            status.lastsStop[muscle] = 0;
            status.lastsMove[muscle] = 1;
            status.lasts[muscle] = last;

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
        if (/* остановка движения - по истечении доступных фреймов */
            //status.lasts[muscle] >= physics.maxMoveFrames[joint]
            /* остановка по истечении заявленной длительности */
            status.lasts[muscle] <= status.lastsMove[muscle]
            /* продолжение движения, если остался на месте (блокировка противоположным мускулом) */
            || !muscleFrame(m))
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
    if (status.lastsStop[muscle] > 0)
    {
        /* Движение по инерции */
        if (!muscleFrame(m))
        {
            /* если движения нет и торможение трением завершилось */
            assert(status.lastsMove[muscle] == 0);
            //-------------------------------------------------------
            status.lastsMove[muscle] = 0;
            status.lastsStop[muscle] = 0;
            status.lasts[muscle] = 0;
            //-------------------------------------------------------
            /* исключаем остановленный двигатель */
            status.musclesMove[muscle] = 0;
            /* проверяем, что остальные двигатели уже остановились */
            if (ba::none_of(status.musclesMove, [](const auto &v) { return (v != 0); }))
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
    Hand(baseClavicle, JInputs<Hand::JointInput>(joints)) {}

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
                              Hand::minFrameMove * 180. / M_PI,
                              jointsMaxAngles[joint]);

        double  maxVelosity = *boost::max_element(framesMove[joint]);
        law.stopLaw->generate(framesStop[joint].begin(),
                              minStopFrames[joint] - 1,
                              Hand::minFrameMove * 180. / M_PI,
                              jointsMaxAngles[joint] * law.stopDistanceRatio,
                              maxVelosity);
        /* last frame must be 0 to deadend */
        framesStop[joint][minStopFrames[joint] - 1] = 0;
    }
}
//--------------------------------------------------------------------------------
void      Hand::step(IN frames_t frame, IN muscle_t muscle, IN frames_t last)
{
    /* Исключить незадействованные двигатели */
    if (muscle < musclesCount() && last > 0)
    {
        ///if (muscle != Robo::MInvalid)
        ///{
        ///    Control control;
        ///    for (muscle_t m = 0; m < musclesCount(); ++m)
        ///        if (status.musclesMove[M(m)] > 0)
        ///        {
        ///            Hand::Muscle m_ = M(m);
        ///            frames_t start = (frame - (status.lastsMove[m_] + status.lastsStop[m_]));
        ///            control.append({ m, start, status.lasts[m_] });
        ///        }
        ///        else if (m == muscle) control.append({ m, frame, last });
        ///
        ///    
        ///    controlValidate(control);
        ///    ///if (timeValidToStartOppositeMuscle(muscle))
        ///}

        if (status.musclesMove[M(muscle)] <= frame)
            muscleMove(frame, muscle, last);
    }
    else if (muscle == Robo::MInvalid && last == 0)
    {
        for (muscle_t m = 0; m < musclesCount(); ++m)
            while (status.musclesMove[M(m)] > 0 && status.musclesMove[M(m)] <= frame)
                muscleMove(frame, m, last);
    }
    else throw std::exception("Controls invalid!");
}
//--------------------------------------------------------------------------------
frames_t  Hand::move(IN Muscle muscle, IN frames_t last)
{
    frames_t frame = 0;
    if (muscle < musclesCount() && last > 0)
    {
        step(frame, muscle, last);
        /* Что-то должно двигаться, иначе беск.цикл */
        while (!moveEnd())
            step(frame++);
    }
    return frame;
}
frames_t  Hand::move(IN Muscle muscle, IN frames_t last, OUT Trajectory &visited)
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
frames_t  Hand::move(IN const Control& controls, OUT Trajectory& visited)
{
    controlsValidate(controls);

    frames_t frame = 0;
    /* Запускаем двигатели */
    for (const auto &c : controls)
    {
        if (frame < c.start)
        {
            step(frame);
            if (!(frame % getVisitedRarity()))
                visited.push_back(position());
            ++frame;
        }
        else if (frame == c.start)
        {
            step(frame, c.muscle, c.lasts);
            /* (no ++frame) могут стартовать одновременно
             * несколько мускулов, разной продолжительности
             */
        }
    } // end for

    while (!moveEnd())
    {
        step(frame);
        if (!(frame % getVisitedRarity()))
            visited.push_back(position());
        ++frame;
    }
    /* final position */
    visited.push_back(position());
    return frame;
}
//--------------------------------------------------------------------------------
void  Hand::reset()
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

        double maxAngle = (joint == Joint::Clvcl) ?
            static_cast<double>(physics.jointsMaxAngles[Joint::Clvcl]) / 100. :
                                physics.jointsMaxAngles[joint];

        double angle = ratio * maxAngle;
        if (status.angles[joint] != angle)
        {
            /* status[ curPos, angles ] */
            jointMove(jr.first, (angle - status.angles[joint]));
        }
    }
}
//--------------------------------------------------------------------------------
void  Hand::draw(IN HDC hdc, IN HPEN hPen, IN HBRUSH hBrush) const
{
#ifdef MY_WINDOW
    HPEN   hPen_old = (HPEN)SelectObject(hdc, hPen);
    HBRUSH hBrush_old = (HBRUSH)SelectObject(hdc, hBrush);
    //-----------------------------------------------------------------

    // // c-clavicle, s-shoulder, a-arm, h-hand, p-palm
    //Point c(physics.jointsOpenCoords[Joint::Clvcl + 1]),
    //      s(status.curPos[Joint::Clvcl]), a(status.curPos[Joint::Shldr]),
    //      h(status.curPos[Joint::Elbow]), p(status.curPos[Joint::Wrist]);
    //
    // // u-up, d-down // звенья - верхняя и нижняя палки
    //Point su(physics.jointsOpenCoords[Joint::Clvcl].x + params.sectionWidth - status.angles[Joint::Clvcl],
    //         physics.jointsOpenCoords[Joint::Clvcl].y + params.sectionWidth),
    //      sd(physics.jointsOpenCoords[Joint::Clvcl].x - params.sectionWidth - status.angles[Joint::Clvcl],
    //         physics.jointsOpenCoords[Joint::Clvcl].y - params.sectionWidth),
    //      au(physics.jointsOpenCoords[Joint::Shldr].x + params.sectionWidth - status.angles[Joint::Clvcl],
    //         physics.jointsOpenCoords[Joint::Shldr].y + params.sectionWidth),
    //      ad(physics.jointsOpenCoords[Joint::Shldr].x - params.sectionWidth - status.angles[Joint::Clvcl],
    //         physics.jointsOpenCoords[Joint::Shldr].y - params.sectionWidth),
    //      hu(physics.jointsOpenCoords[Joint::Elbow].x + params.sectionWidth - status.angles[Joint::Clvcl],
    //         physics.jointsOpenCoords[Joint::Elbow].y + params.sectionWidth),
    //      hd(physics.jointsOpenCoords[Joint::Elbow].x - params.sectionWidth - status.angles[Joint::Clvcl],
    //         physics.jointsOpenCoords[Joint::Elbow].y - params.sectionWidth);
    //
    //Point wu, wd;
    //if (params.drawPalm)
    //{
    //    wu = Point(physics.jointsOpenCoords[Joint::Wrist].x + params.sectionWidth - status.angles[Joint::Clvcl],
    //               physics.jointsOpenCoords[Joint::Wrist].y + params.sectionWidth);
    //    wd = Point(physics.jointsOpenCoords[Joint::Wrist].x - params.sectionWidth - status.angles[Joint::Clvcl],
    //               physics.jointsOpenCoords[Joint::Wrist].y - params.sectionWidth);
    //}
    // //-----------------------------------------------------------------
    //su.rotate(s, status.angles[Joint::Shldr]);
    //sd.rotate(s, status.angles[Joint::Shldr]);
    //
    //au.rotate(s, status.angles[Joint::Shldr]);
    //ad.rotate(s, status.angles[Joint::Shldr]);
    //
    //hu.rotate(s, status.angles[Joint::Shldr]);
    //hd.rotate(s, status.angles[Joint::Shldr]);
    //hu.rotate(a, status.angles[Joint::Elbow]);
    //hd.rotate(a, status.angles[Joint::Elbow]);
    //
    //if (params.drawPalm)
    //{
    //    wu.rotate(s, status.angles[Joint::Shldr]);
    //    wd.rotate(s, status.angles[Joint::Shldr]);
    //    wu.rotate(a, status.angles[Joint::Elbow]);
    //    wd.rotate(a, status.angles[Joint::Elbow]);
    //    wu.rotate(h, status.angles[Joint::Wrist]);
    //    wd.rotate(h, status.angles[Joint::Wrist]);
    //}
    // //-----------------------------------------------------------------
    //MoveToEx (hdc, Tx(1.00), Ty(su.y), NULL);
    //LineTo   (hdc, Tx(su.x), Ty(su.y));
    //LineTo   (hdc, Tx(au.x), Ty(au.y));
    //
    //MoveToEx (hdc, Tx(1.00), Ty(sd.y), NULL);
    //LineTo   (hdc, Tx(sd.x), Ty(sd.y));
    //LineTo   (hdc, Tx(ad.x), Ty(ad.y));
    //
    //au.rotate(a, status.angles[Joint::Elbow]);
    //ad.rotate(a, status.angles[Joint::Elbow]);
    //
    //MoveToEx (hdc, Tx(au.x), Ty(au.y), NULL);
    //LineTo   (hdc, Tx(hu.x), Ty(hu.y));
    //
    //MoveToEx (hdc, Tx(ad.x), Ty(ad.y), NULL);
    //LineTo   (hdc, Tx(hd.x), Ty(hd.y));
    //
    //if (params.drawPalm)
    //{
    //    //---palm-------------------- --------------------------------------
    //    DrawCircle(hdc, p, params.palmRadius);
    //
    //    //------------------------------------------------------------------
    //    hu.rotate(h, status.angles[Wrist]);
    //    hd.rotate(h, status.angles[Wrist]);
    //
    //    LineTo   (hdc, Tx(wd.x), Ty(wd.y));
    //    MoveToEx (hdc, Tx(hu.x), Ty(hu.y), NULL);
    //    LineTo   (hdc, Tx(wu.x), Ty(wu.y));
    //}
    // //---clavicle-------------------------------------------------------
    //DrawCircle(hdc, c, params.jointRadius);
    //
    // //---shoulder-------------------------------------------------------
    //DrawCircle(hdc, s, params.jointRadius);
    //
    // //---arm--------------------- --------------------------------------
    //DrawCircle(hdc, a, params.jointRadius);
    //
    // //---hand-------------------- --------------------------------------
    //DrawCircle(hdc, h, params.jointRadius);

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
            Control control{ { m, 0, muscleMaxLast(m) } };
            //CINFO(control);
            move(control, workSpace);
        }
    }
    reset();
}
//--------------------------------------------------------------------------------
void  Hand::controlsValidate(const Control &controls) const
{
    /* Что-то должно двигаться, иначе беск.цикл */
    if (!controls.size())
        throw std::logic_error("Controls are empty!");
    /* Управление должно быть отсортировано по времени запуска двигателя */
    if (controls[0].start != 0 || !br::is_sorted(controls))
        throw std::logic_error("Controls are not sorted!");
    /* Исключить незадействованные двигатели */
    auto ctrlInvalid = [muscles = musclesCount()](const auto &a) {
        //CINFO(a);
        return (a.lasts == 0) || (a.muscle >= muscles);
    };
    if (ba::any_of(controls, ctrlInvalid))
        throw std::logic_error("Controls have UNUSED or INVALID muscles!");

    /// TODO: Opposite simultaneosly
    /* Совместимо ли новое с тем, что уже сейчас движется? */
}

//--------------------------------------------------------------------------------
//Hand::Muscle Hand::selectControl(IN Muscle muscle) const
//{
//    if (!muscle)
//    { return controls[random(controls.size())]; }
//    else
//    {
//        for (auto m : controls)
//            if (!(m & muscle) && musclesValidUnion(m | muscle))
//                return m;
//        // auto m = controls[random (controls.size ())];
//        // while ( (m & muscle) || !musclesValidUnion (m | muscle) )
//        //   m = controls[random (controls.size ())];
//    }
//    return  EmptyMov;
//}
//--------------------------------------------------------------------------------
