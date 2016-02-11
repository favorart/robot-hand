#include "StdAfx.h"

#ifndef  _HAND_MUSCLES_H_
#define  _HAND_MUSCLES_H_
//------------------------------------------------------------------------------
namespace NewHand
{
  //------------------------------------------------------------------------------
  static const Hand::j_array   joints = { Hand::Clvcl, Hand::Shldr,
                                          Hand::Elbow, Hand::Wrist };

  static const Hand::m_array  muscles = { Hand::ClvclOpn, Hand::ClvclCls,
                                          Hand::ShldrOpn, Hand::ShldrCls,
                                          Hand::ElbowOpn, Hand::ElbowCls,
                                          Hand::WristOpn, Hand::WristCls };
  //------------------------------------------------------------------------------
  Hand::MusclesEnum  operator| (Hand::MusclesEnum m, Hand::MusclesEnum k);
  Hand::MusclesEnum  operator& (Hand::MusclesEnum m, Hand::MusclesEnum k);
  Hand::MusclesEnum  operator^ (Hand::MusclesEnum m, Hand::MusclesEnum k);

  Hand::JointsEnum   operator| (Hand::JointsEnum  j, Hand::JointsEnum  k);
  Hand::JointsEnum   operator& (Hand::JointsEnum  j, Hand::JointsEnum  k);
  Hand::JointsEnum   operator^ (Hand::JointsEnum  j, Hand::JointsEnum  k);
  //------------------------------------------------------------------------------
  std::ostream&  operator<< (std::ostream &out, Hand::MusclesEnum m);
  std::ostream&  operator<< (std::ostream &out, Hand::JointsEnum  j);
  //------------------------------------------------------------------------------
  bool  muscleValidAtOnce (Hand::MusclesEnum muscle);
  //------------------------------------------------------------------------------
  Hand::MusclesEnum  muscleByJoint (Hand::JointsEnum  joint, bool open);
  Hand::JointsEnum   jointByMuscle (Hand::MusclesEnum muscle);
//------------------------------------------------------------------------------
};
#endif // _HAND_MUSCLES_H_
