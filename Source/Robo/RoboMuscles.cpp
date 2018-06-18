#include "StdAfx.h"
#include "RoboMuscles.h"


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
    if (muscle > Hand::MCount)
        throw std::runtime_error("Help of INVALID Hand::Muscle");
    return HHelps[muscle];
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
        _T("левая гусеница вперёд"), _T("левая гусеница назад"),
        _T("правая гусеница вперёд"), _T("правая гусеница назад")
    };
    if (muscle > Tank::MCount)
        throw std::runtime_error("Help of INVALID Tank::Muscle");
    return THelps[muscle];
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
