﻿#include "StdAfx.h"
#include "RoboMuscles.h"
#include "RoboMotionLaws.h"

//--------------------------------------------------------------------------------
tstring Robo::NewHand::muscleHelp(Hand::Muscle muscle)
{
    tstring HHelps[] =
    {
        _T("сомкнуть запястье"), _T("раскрыть запястье"),
        _T("сомкнуть локоть"), _T("раскрыть локоть"),
        _T("сомкнуть плечо"), _T("раскрыть плечо"),
        _T("двинуть ключицей влево"), _T("двинуть ключицей вправо")
    };
    if (muscle >= Hand::Muscle::MCount)
        throw std::runtime_error("Help of INVALID Hand::Muscle");
    return HHelps[int(muscle)];
}
tstring Robo::NewHand::muscleName(Hand::Muscle muscle)
{
    switch (muscle)
    {
    case Hand::Muscle::ClvclOpn: return _T(" M:Clvcl-Open  "); break;
    case Hand::Muscle::ClvclCls: return _T(" M:Clvcl-Close "); break;
    case Hand::Muscle::ShldrOpn: return _T(" M:Shldr-Open  "); break;
    case Hand::Muscle::ShldrCls: return _T(" M:Shldr-Close "); break;
    case Hand::Muscle::ElbowOpn: return _T(" M:Elbow-Open  "); break;
    case Hand::Muscle::ElbowCls: return _T(" M:Elbow-Close "); break;
    case Hand::Muscle::WristOpn: return _T(" M:Wrist-Open  "); break;
    case Hand::Muscle::WristCls: return _T(" M:Wrist-Close "); break;
    }
    return _T(" M:Invalid     ");
}
tstring Robo::NewHand::jointName(Hand::Joint joint)
{
    switch (joint)
    {
    case Hand::Joint::Clvcl: return _T(" J:Clavicle "); break;
    case Hand::Joint::Shldr: return _T(" J:Shoulder "); break;
    case Hand::Joint::Elbow: return _T(" J:Elbow    "); break;
    case Hand::Joint::Wrist: return _T(" J:Wrist    "); break;
    }
    return _T(" J:Invalid  ");
}

//--------------------------------------------------------------------------------
tstring Robo::Mobile::muscleHelp(Tank::Muscle muscle)
{
    tstring THelps[] =
    {
        _T("левая гусеница назад"), _T("левая гусеница вперёд"),
        _T("правая гусеница назад"), _T("правая гусеница вперёд")
    };
    if (muscle >= Tank::Muscle::MCount)
        throw std::runtime_error("Help of INVALID Tank::Muscle");
    return THelps[int(muscle)];
}
tstring Robo::Mobile::muscleName(Tank::Muscle muscle)
{
    switch (muscle)
    {
    case Tank::Muscle::LTrackFrw: return _T(" M:LTrack-Forward  "); break;
    case Tank::Muscle::LTrackBck: return _T(" M:LTrack-Backward "); break;
    case Tank::Muscle::RTrackFrw: return _T(" M:RTrack-Forward  "); break;
    case Tank::Muscle::RTrackBck: return _T(" M:RTrack-Backward "); break;
    }
    return _T(" M:Invalid     ");
}
tstring Robo::Mobile::jointName(Tank::Joint joint)
{
    switch (joint)
    {
    case Tank::Joint::LTrack: return _T(" J:LTrack  "); break;
    case Tank::Joint::RTrack: return _T(" J:RTrack  "); break;
    }
    return _T(" J:Invalid ");
}

//--------------------------------------------------------------------------------
tstring Robo::getJointName(const Robo::RoboI& robo, Robo::joint_t joint)
{
    if (typeid(robo) == typeid(Robo::NewHand::Hand))
    {
        auto &hand = dynamic_cast<const Robo::NewHand::Hand&>(robo);
        return jointName(hand.J(joint));
    }
    else if (typeid(robo) == typeid(Robo::Mobile::Tank))
    {
        auto &tank = dynamic_cast<const Robo::Mobile::Tank&>(robo);
        return jointName(tank.J(joint));
    }
    throw std::exception("Not Implemented");
}
tstring Robo::getMuscleName(const Robo::RoboI& robo, Robo::muscle_t m)
{
    if (typeid(robo) == typeid(Robo::NewHand::Hand))
    {
        auto &hand = dynamic_cast<const Robo::NewHand::Hand&>(robo);
        return muscleName(hand.M(m));
    }
    else if (typeid(robo) == typeid(Robo::Mobile::Tank))
    {
        auto &tank = dynamic_cast<const Robo::Mobile::Tank&>(robo);
        return muscleName(tank.M(m));
    }
    throw std::exception("Not Implemented");
}
//-------------------------------------------------------------------------------

