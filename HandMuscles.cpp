#include "StdAfx.h"
#include "NewHand.h"
#include "HandMuscles.h"

using namespace NewHand;
//------------------------------------------------------------------------------
Hand::JointsEnum   NewHand::operator| (Hand::JointsEnum  j, Hand::JointsEnum  k)
{ return static_cast<Hand::JointsEnum> (static_cast<uchar_t> (j) | static_cast<uchar_t> (k)); }
Hand::JointsEnum   NewHand::operator& (Hand::JointsEnum  j, Hand::JointsEnum  k)
{ return static_cast<Hand::JointsEnum> (static_cast<uchar_t> (j) & static_cast<uchar_t> (k)); }
Hand::JointsEnum   NewHand::operator^ (Hand::JointsEnum  j, Hand::JointsEnum  k)
{ return static_cast<Hand::JointsEnum> (static_cast<uchar_t> (j) ^ static_cast<uchar_t> (k)); }

Hand::MusclesEnum  NewHand::operator| (Hand::MusclesEnum m, Hand::MusclesEnum k)
{ return static_cast<Hand::MusclesEnum> (static_cast<uchar_t> (m) | static_cast<uchar_t> (k)); }
Hand::MusclesEnum  NewHand::operator& (Hand::MusclesEnum m, Hand::MusclesEnum k)
{ return static_cast<Hand::MusclesEnum> (static_cast<uchar_t> (m) & static_cast<uchar_t> (k)); }
Hand::MusclesEnum  NewHand::operator^ (Hand::MusclesEnum m, Hand::MusclesEnum k)
{ return static_cast<Hand::MusclesEnum> (static_cast<uchar_t> (m) ^ static_cast<uchar_t> (k)); }

std::ostream&  NewHand::operator<< (std::ostream &out, Hand::MusclesEnum muscle)
{
  if ( !muscle )  return  out << "Hand::EmptyMov ";

  for ( auto m : muscles )
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
std::ostream&  NewHand::operator<< (std::ostream &out, Hand::JointsEnum   joint)
{
  if ( !joint )  return  out << "Hand::Empty ";

  for ( auto j : joints )
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
bool  NewHand::muscleValidAtOnce (Hand::MusclesEnum muscle)
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
Hand::MusclesEnum  NewHand::muscleByJoint (Hand:: JointsEnum joint, bool open)
{
  if ( !joint ) return Hand::EmptyMov;

  for ( auto j : joints )
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
Hand:: JointsEnum  NewHand::jointByMuscle (Hand::MusclesEnum muscle)
{
  if ( !muscle ) return Hand::Empty;

  for ( auto m : muscles )
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
