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

tostream&  NewHand::operator<< (tostream &out, Hand::MusclesEnum muscle)
{
  if ( !muscle )  return  out << "EmptyMov"; // Hand::

  for ( auto m : muscles )
  {
    if ( m & muscle )
      switch ( m )
      {
        // case Hand::EmptyMov: out << "EmptyMov "; break; // Hand::
        case Hand::ClvclOpn: out << "ClvclOpn"; break; // Hand::
        case Hand::ClvclCls: out << "ClvclCls"; break; // Hand::
        case Hand::ShldrOpn: out << "ShldrOpn"; break; // Hand::
        case Hand::ShldrCls: out << "ShldrCls"; break; // Hand::
        case Hand::ElbowOpn: out << "ElbowOpn"; break; // Hand::
        case Hand::ElbowCls: out << "ElbowCls"; break; // Hand::
        case Hand::WristOpn: out << "WristOpn"; break; // Hand::
        case Hand::WristCls: out << "WristCls"; break; // Hand::
      }
  }
  return out;
}
tostream&  NewHand::operator<< (tostream &out, Hand::JointsEnum   joint)
{
  if ( !joint )  return  out << "Empty"; // Hand::

  for ( auto j : joints )
  {
    if ( j & joint )
      switch ( j )
      {
        // case Hand::Empty: out << "Empty"; break; // Hand::
        case Hand::Clvcl: out << "Clvcl"; break; // Hand::
        case Hand::Shldr: out << "Shldr"; break; // Hand::
        case Hand::Elbow: out << "Elbow"; break; // Hand::
        case Hand::Wrist: out << "Wrist"; break; // Hand::
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
