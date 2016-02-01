#include "StdAfx.h"
#include "NewHand.h"

//------------------------------------------------------------------------------
NewHand::Hand::JointsEnum   NewHand::operator| (NewHand::Hand::JointsEnum  j, NewHand::Hand::JointsEnum  k)
{ return static_cast<Hand::JointsEnum> (static_cast<uchar_t> (j) | static_cast<uchar_t> (k)); }
NewHand::Hand::JointsEnum   NewHand::operator& (NewHand::Hand::JointsEnum  j, NewHand::Hand::JointsEnum  k)
{ return static_cast<Hand::JointsEnum> (static_cast<uchar_t> (j) & static_cast<uchar_t> (k)); }
NewHand::Hand::JointsEnum   NewHand::operator^ (NewHand::Hand::JointsEnum  j, NewHand::Hand::JointsEnum  k)
{ return static_cast<Hand::JointsEnum> (static_cast<uchar_t> (j) ^ static_cast<uchar_t> (k)); }

NewHand::Hand::MusclesEnum  NewHand::operator| (NewHand::Hand::MusclesEnum m, NewHand::Hand::MusclesEnum k)
{ return static_cast<Hand::MusclesEnum> (static_cast<uchar_t> (m) | static_cast<uchar_t> (k)); }
NewHand::Hand::MusclesEnum  NewHand::operator& (NewHand::Hand::MusclesEnum m, NewHand::Hand::MusclesEnum k)
{ return static_cast<Hand::MusclesEnum> (static_cast<uchar_t> (m) & static_cast<uchar_t> (k)); }
NewHand::Hand::MusclesEnum  NewHand::operator^ (NewHand::Hand::MusclesEnum m, NewHand::Hand::MusclesEnum k)
{ return static_cast<Hand::MusclesEnum> (static_cast<uchar_t> (m) ^ static_cast<uchar_t> (k)); }

std::ostream&  NewHand::operator<< (std::ostream &out, NewHand::Hand::MusclesEnum muscle)
{
  if ( !muscle )  return  out << "Hand::EmptyMov ";

  for ( auto m : NewHand::muscles )
  {
    if ( m & muscle )
      switch ( m )
      {
        // case Hand::EmptyMov: out << "Hand::EmptyMov "; break;
        case Hand::ClvclOpn: out << "Hand::ClvclOpn "; break;
        case Hand::ClvclCls: out << "Hand::ClvclCls "; break;
        case Hand::ShldrOpn: out << "Hand::ShldrOpn "; break;
        case Hand::ShldrCls: out << "Hand::ShldrCls "; break;
        case Hand::ElbowOpn: out << "Hand::ElbowOpn "; break;
        case Hand::ElbowCls: out << "Hand::ElbowCls "; break;
        case Hand::WristOpn: out << "Hand::WristOpn "; break;
        case Hand::WristCls: out << "Hand::WristCls "; break;
      }
  }
  return out;
}
std::ostream&  NewHand::operator<< (std::ostream &out, NewHand::Hand::JointsEnum   joint)
{
  if ( !joint )  return  out << "Hand::Empty ";

  for ( auto j : NewHand::joints )
  {
    if ( j & joint )
      switch ( j )
      {
        // case Hand::Empty: out << "Hand::Empty "; break;
        case Hand::Clvcl: out << "Hand::Clvcl "; break;
        case Hand::Shldr: out << "Hand::Shldr "; break;
        case Hand::Elbow: out << "Hand::Elbow "; break;
        case Hand::Wrist: out << "Hand::Wrist "; break;
      }
  }
  return out;
}
//--------------------------------------------------------------------------------
bool  NewHand::muscleValidAtOnce (NewHand::Hand::MusclesEnum muscle)
{
  if ( !muscle
      || ((Hand::ClvclOpn & muscle) && (Hand::ClvclCls & muscle))
      || ((Hand::ShldrOpn & muscle) && (Hand::ShldrCls & muscle))
      || ((Hand::ElbowOpn & muscle) && (Hand::ElbowCls & muscle))
      || ((Hand::WristOpn & muscle) && (Hand::WristCls & muscle))
      )
  { return false; }
  return true;
}
//--------------------------------------------------------------------------------
NewHand::Hand::MusclesEnum  NewHand::muscleByJoint (NewHand::Hand:: JointsEnum joint, bool open)
{
  if ( !joint ) return Hand::EmptyMov;

  for ( auto j : NewHand::joints )
  {
    if ( j & joint )
      switch ( j )
      {
        case Hand::Clvcl: return (open) ? Hand::ClvclOpn : Hand::ClvclCls;
        case Hand::Shldr: return (open) ? Hand::ShldrOpn : Hand::ShldrCls;
        case Hand::Elbow: return (open) ? Hand::ElbowOpn : Hand::ElbowCls;
        case Hand::Wrist: return (open) ? Hand::WristOpn : Hand::WristCls;
        default:          return Hand::EmptyMov;
      }
  }
  return Hand::EmptyMov;
}
NewHand::Hand:: JointsEnum  NewHand::jointByMuscle (NewHand::Hand::MusclesEnum muscle)
{
  if ( !muscle ) return Hand::Empty;

  for ( auto m : NewHand::muscles )
  {
    if ( m & muscle )
      switch ( m )
      {
        default:             return Hand::Empty;
        case Hand::ClvclOpn:
        case Hand::ClvclCls: return Hand::Clvcl;
        case Hand::ShldrOpn:
        case Hand::ShldrCls: return Hand::Shldr;
        case Hand::ElbowOpn:
        case Hand::ElbowCls: return Hand::Elbow;
        case Hand::WristOpn:
        case Hand::WristCls: return Hand::Wrist;
      }
  }
  return Hand::Empty;
}
//--------------------------------------------------------------------------------
//NewHand::Hand::MusclesEnum  NewHand::selectHandMove (size_t choose)
//{
//  Hand::MusclesEnum  muscles;
//  switch ( choose )
//  {
//    default: muscles = Hand::EmptyMov;                                   break;
//    case  0: muscles = Hand::ClvclOpn;                                   break;
//    case  1: muscles = Hand::ShldrOpn;                                   break;
//    case  2: muscles = Hand::ElbowOpn;                                   break;
//    case  3: muscles = Hand::ClvclCls;                                   break;
//    case  4: muscles = Hand::ShldrCls;                                   break;
//    case  5: muscles = Hand::ElbowCls;                                   break;
//    case  6: muscles = Hand::ClvclOpn | Hand::ShldrOpn;                  break;
//    case  7: muscles = Hand::ClvclOpn | Hand::ElbowOpn;                  break;
//    case  8: muscles = Hand::ShldrOpn | Hand::ElbowOpn;                  break;
//    case  9: muscles = Hand::ClvclOpn | Hand::ShldrOpn | Hand::ElbowOpn; break;
//    case 10: muscles = Hand::ClvclOpn | Hand::ShldrOpn | Hand::ElbowCls; break;
//    case 11: muscles = Hand::ClvclCls | Hand::ShldrOpn;                  break;
//    case 12: muscles = Hand::ClvclCls | Hand::ElbowOpn;                  break;
//    case 13: muscles = Hand::ShldrCls | Hand::ElbowOpn;                  break;
//    case 14: muscles = Hand::ClvclCls | Hand::ShldrOpn | Hand::ElbowOpn; break;
//    case 15: muscles = Hand::ClvclCls | Hand::ShldrOpn | Hand::ElbowCls; break;
//    case 16: muscles = Hand::ClvclOpn | Hand::ShldrCls;                  break;
//    case 17: muscles = Hand::ClvclOpn | Hand::ElbowCls;                  break;
//    case 18: muscles = Hand::ShldrOpn | Hand::ElbowCls;                  break;
//    case 19: muscles = Hand::ClvclOpn | Hand::ShldrCls | Hand::ElbowOpn; break;
//    case 20: muscles = Hand::ClvclOpn | Hand::ShldrCls | Hand::ElbowCls; break;
//    case 21: muscles = Hand::ClvclCls | Hand::ShldrCls;                  break;
//    case 22: muscles = Hand::ClvclCls | Hand::ElbowCls;                  break;
//    case 23: muscles = Hand::ShldrCls | Hand::ElbowCls;                  break;
//    case 24: muscles = Hand::ClvclCls | Hand::ShldrCls | Hand::ElbowOpn; break;
//    case 25: muscles = Hand::ClvclCls | Hand::ShldrCls | Hand::ElbowCls; break;
//  }
//  return muscles;
//}

//--------------------------------------------------------------------------------