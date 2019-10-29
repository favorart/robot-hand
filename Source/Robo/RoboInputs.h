#pragma once

#include "Robo.h"
#include "RoboMuscles.h"
#include "Hand.h"
#include "Tank.h"
//--------------------------------------------------------------------------------
struct Robo::Mobile::Tank::JointInput : public Robo::JointInput
{
    Tank::Joint Joint() const { return static_cast<Tank::Joint>(joint); }
    tstring name() const { return jointName(Joint()); }

    JointInput() : Robo::JointInput() {}
    JointInput(joint_t joint, bool show, const Point &openCoords,
               const MotionLaws::JointMotionLaw &frames) :
        Robo::JointInput(joint, show, openCoords, frames)
    {}
    void save(tptree &node) const
    {
        Robo::JointInput::save(node);
        node.put(_T("name"), ba::trim_copy(name()));
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
    distance_t defaultPose{ 0. };

    JointInput() : Robo::JointInput() {}
    JointInput(joint_t joint, bool show, const Point &openCoords,
               const MotionLaws::JointMotionLaw &frames, distance_t defPose) :
        Robo::JointInput(joint, show, openCoords, frames), defaultPose(defPose)
    {}
    void save(tptree &node) const
    {
        Robo::JointInput::save(node);
        node.put(_T("defPoseRatio"), defaultPose);
        node.put(_T("name"), ba::trim_copy(name()));
    }
    void load(tptree &node)
    {
        Robo::JointInput::load(node);
        defaultPose = node.get<double>(_T("defPoseRatio"));
        //if (name() == node.get<tstring>(_T("name")))
        //    CERROR("Invalid name");
    }
};

//--------------------------------------------------------------------------------
