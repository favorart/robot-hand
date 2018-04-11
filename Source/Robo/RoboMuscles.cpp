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
        _T("двинуть ключицей вправо"), _T("двинуть ключицей влево")
    };
    if (muscle > Hand::MusclesMaxCount)
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
    if (muscle > Tank::MusclesMaxCount)
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

//bool Hand::musclesValidUnion(Muscle muscle)
//{
//  if ( !muscle
//      || ((Muscle::ClvclOpn & muscle) && (Muscle::ClvclCls & muscle))
//      || ((Muscle::ShldrOpn & muscle) && (Muscle::ShldrCls & muscle))
//      || ((Muscle::ElbowOpn & muscle) && (Muscle::ElbowCls & muscle))
//      || ((Muscle::WristOpn & muscle) && (Muscle::WristCls & muscle))
//     )
//  { return false; }
//  return true;
//}
////--------------------------------------------------------------------------------
//Hand::JointsEnum   NewHand::operator| (Hand::JointsEnum  j, Hand::JointsEnum  k)
//{ return static_cast<Hand::JointsEnum> (static_cast<uchar_t> (j) | static_cast<uchar_t> (k)); }
//Hand::JointsEnum   NewHand::operator& (Hand::JointsEnum  j, Hand::JointsEnum  k)
//{ return static_cast<Hand::JointsEnum> (static_cast<uchar_t> (j) & static_cast<uchar_t> (k)); }
//Hand::JointsEnum   NewHand::operator^ (Hand::JointsEnum  j, Hand::JointsEnum  k)
//{ return static_cast<Hand::JointsEnum> (static_cast<uchar_t> (j) ^ static_cast<uchar_t> (k)); }
//Hand::MusclesEnum  NewHand::operator| (Hand::MusclesEnum m, Hand::MusclesEnum k)
//{ return static_cast<Hand::MusclesEnum> (static_cast<uchar_t> (m) | static_cast<uchar_t> (k)); }
//Hand::MusclesEnum  NewHand::operator& (Hand::MusclesEnum m, Hand::MusclesEnum k)
//{ return static_cast<Hand::MusclesEnum> (static_cast<uchar_t> (m) & static_cast<uchar_t> (k)); }
//Hand::MusclesEnum  NewHand::operator^ (Hand::MusclesEnum m, Hand::MusclesEnum k)
//{ return static_cast<Hand::MusclesEnum> (static_cast<uchar_t> (m) ^ static_cast<uchar_t> (k)); }
//tostream&  NewHand::operator<< (tostream &out, Hand::MusclesEnum muscle)
//{
//  if ( !muscle )  return  out << _T(" EmptyMov"); // Hand::
//
//  for ( auto m : muscles )
//  {
//    if ( m & muscle )
//      switch ( m )
//      {
//        // case Hand::EmptyMov: out << _T(" EmptyMov"); break; // Hand::
//        case Hand::ClvclOpn: out << _T(" ClvclOpn"); break; // Hand::
//        case Hand::ClvclCls: out << _T(" ClvclCls"); break; // Hand::
//        case Hand::ShldrOpn: out << _T(" ShldrOpn"); break; // Hand::
//        case Hand::ShldrCls: out << _T(" ShldrCls"); break; // Hand::
//        case Hand::ElbowOpn: out << _T(" ElbowOpn"); break; // Hand::
//        case Hand::ElbowCls: out << _T(" ElbowCls"); break; // Hand::
//        case Hand::WristOpn: out << _T(" WristOpn"); break; // Hand::
//        case Hand::WristCls: out << _T(" WristCls"); break; // Hand::
//      }
//  }
//  return out;
//}
//tostream&  NewHand::operator<< (tostream &out, Hand::JointsEnum   joint)
//{
//  if ( !joint )  return  out << _T(" Empty"); // Hand::
//
//  for ( auto j : joints )
//  {
//    if ( j & joint )
//      switch ( j )
//      {
//        // case Hand::Empty: out << _T(" Empty"); break; // Hand::
//        case Hand::Clvcl: out << _T(" Clvcl"); break; // Hand::
//        case Hand::Shldr: out << _T(" Shldr"); break; // Hand::
//        case Hand::Elbow: out << _T(" Elbow"); break; // Hand::
//        case Hand::Wrist: out << _T(" Wrist"); break; // Hand::
//      }
//  
//  }
//  return out;
//}
