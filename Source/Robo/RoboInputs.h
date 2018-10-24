#pragma once

#include "Robo.h"
#include "Hand.h"
#include "Tank.h"

//--------------------------------------------------------------------------------
struct Robo::Mobile::Tank::JointInput : public Robo::JointInput
{
    Tank::Joint Joint() const { return static_cast<Tank::Joint>(joint); }
    tstring name() const { return jointName(Joint()); }
    void save(tptree &node) const
    {
        Robo::JointInput::save(node);
        node.put(_T("name"), name());
    }
    void load(tptree &node)
    {
        Robo::JointInput::load(node);
        //if (name() == node.get<tstring>(_T("name")))
        //    CERROR("Invalid name");
    }
};

//--------------------------------------------------------------------------------
struct Robo::NewHand::Hand::JointInput : public Robo::JointInput
{
    // [ &palm, &hand, &arm, &shoulder ]
    Hand::Joint Joint() const { return static_cast<Hand::Joint>(joint); }
    tstring name() const { return jointName(Joint()); }

    double defaultPose{ 0. };

    JointInput() : Robo::JointInput() {}
    JointInput(joint_t joint, const Point &openCoords, size_t maxMoveFrame, frames_t nMoveFrames,
               double defPose, const MotionLaws::JointMotionLaw &frames, bool show) :
        Robo::JointInput(joint, frames, show, openCoords, nMoveFrames, maxMoveFrame),
        defaultPose(defPose)
    {}
    void save(tptree &node) const
    {
        Robo::JointInput::save(node);
        node.put(_T("name"), name());
        node.put(_T("defPoseRatio"), defaultPose);
    }
    void load(tptree &node)
    {
        Robo::JointInput::load(node);
        //if (name() == node.get<tstring>(_T("name")))
        //    CERROR("Invalid name");
        defaultPose = node.get<double>(_T("defPoseRatio"));
    }
};

//--------------------------------------------------------------------------------
