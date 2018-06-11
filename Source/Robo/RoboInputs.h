#pragma once

#include "Robo.h"
#include "Hand.h"
#include "Tank.h"

//--------------------------------------------------------------------------------
struct Robo::Mobile::Tank::JointInput : public Robo::JointInput
{
    Tank::Joint joint{ Tank::Joint::JInvalid };
    tstring name{ jointName(Tank::Joint::JInvalid) };

    JointInput() : Robo::JointInput() {}
    JointInput(Tank::Joint joint, joint_t type, const Point &openCoords,
               size_t maxMoveFrame, frames_t nMoveFrames,
               const MotionLaws::JointMotionLaw &frames, bool show) :
        Robo::JointInput(type, frames, show, openCoords, nMoveFrames, maxMoveFrame),
        joint(joint), name(jointName(joint))
    {}
};

//--------------------------------------------------------------------------------
struct Robo::NewHand::Hand::JointInput : public Robo::JointInput
{
    Hand::Joint joint{ Hand::Joint::JInvalid };
    tstring name{ jointName(Hand::Joint::JInvalid) }; // [ &palm, &hand, &arm, &shoulder ]
    double defaultPose{ 0. };

    JointInput() : Robo::JointInput() {}
    JointInput(Hand::Joint joint, joint_t type, const Point &openCoords,
               size_t maxMoveFrame, frames_t nMoveFrames, double defaultPose,
               const MotionLaws::JointMotionLaw &frames, bool show) :
        Robo::JointInput(type, frames, show, openCoords, nMoveFrames, maxMoveFrame),
        joint(joint), name(jointName(joint)), defaultPose(defaultPose)
    {}
};

//--------------------------------------------------------------------------------
