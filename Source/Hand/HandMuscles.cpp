#include "StdAfx.h"
#include "Hand.h"
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

tostream&  NewHand::operator<< (tostream &out, Hand::MusclesEnum muscle)
{
  if ( !muscle )  return  out << _T(" EmptyMov"); // Hand::

  for ( auto m : muscles )
  {
    if ( m & muscle )
      switch ( m )
      {
        // case Hand::EmptyMov: out << _T(" EmptyMov"); break; // Hand::
        case Hand::ClvclOpn: out << _T(" ClvclOpn"); break; // Hand::
        case Hand::ClvclCls: out << _T(" ClvclCls"); break; // Hand::
        case Hand::ShldrOpn: out << _T(" ShldrOpn"); break; // Hand::
        case Hand::ShldrCls: out << _T(" ShldrCls"); break; // Hand::
        case Hand::ElbowOpn: out << _T(" ElbowOpn"); break; // Hand::
        case Hand::ElbowCls: out << _T(" ElbowCls"); break; // Hand::
        case Hand::WristOpn: out << _T(" WristOpn"); break; // Hand::
        case Hand::WristCls: out << _T(" WristCls"); break; // Hand::
      }
  }
  return out;
}
tostream&  NewHand::operator<< (tostream &out, Hand::JointsEnum   joint)
{
  if ( !joint )  return  out << _T(" Empty"); // Hand::

  for ( auto j : joints )
  {
    if ( j & joint )
      switch ( j )
      {
        // case Hand::Empty: out << _T(" Empty"); break; // Hand::
        case Hand::Clvcl: out << _T(" Clvcl"); break; // Hand::
        case Hand::Shldr: out << _T(" Shldr"); break; // Hand::
        case Hand::Elbow: out << _T(" Elbow"); break; // Hand::
        case Hand::Wrist: out << _T(" Wrist"); break; // Hand::
      }
  
  }
  return out;
}
//--------------------------------------------------------------------------------
bool  NewHand::musclesValidUnion (Hand::MusclesEnum muscle)
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
Hand::MusclesEnum  NewHand::muscleOpposite (Hand::MusclesEnum muscle)
{
  if ( !muscle ) return Hand::EmptyMov;

  for ( auto m : muscles )
  {
    if ( m & muscle )
      switch ( m )
      {
        default:             return Hand::EmptyMov;
        case Hand::ClvclOpn: return Hand::ClvclCls;
        case Hand::ClvclCls: return Hand::ClvclOpn;
        case Hand::ShldrOpn: return Hand::ShldrCls;
        case Hand::ShldrCls: return Hand::ShldrOpn;
        case Hand::ElbowOpn: return Hand::ElbowCls;
        case Hand::ElbowCls: return Hand::ElbowOpn;
        case Hand::WristOpn: return Hand::WristCls;
        case Hand::WristCls: return Hand::WristOpn;
      }
  }
  return Hand::EmptyMov;
}
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
tostream&  NewHand::operator<< (tostream &out, const Hand::Control &control)
{ return out << control.muscle << _T(" ") << control.start << _T (" ") << control.last; }
//--------------------------------------------------------------------------------