#pragma once

#include "StdAfx.h"
#include "RoboMotionLaws.h"
#include "Robo.h"
#include "Hand.h"
#include "Tank.h"


namespace ConfigJSON
{
// http://zenol.fr/blog/boost-property-tree/en.html
//--------------------------------------------------------------------------------
void save(tptree&, const Point&);
void load(tptree&, Point&);
//--------------------------------------------------------------------------------
struct StageInput1
{
    double   draft_distance;
    unsigned lasts_init_value;
    unsigned lasts_incr_value;
};
//--------------------------------------------------------------------------------
using StageInput2 = StageInput1;
//--------------------------------------------------------------------------------
struct StageInput3
{
    unsigned tries;
    unsigned tries_break;
    unsigned random_try;

    double side;
    double side_decrease_step;
    bool   use_weighted_mean;
};
//--------------------------------------------------------------------------------
void save(tptree&, const StageInput1&, const StageInput2&, const StageInput3&);
void load(tptree&, StageInput1&, StageInput2&, StageInput3&);
//--------------------------------------------------------------------------------
struct TourInput
{
    tstring type;
    unsigned lasts_step_increment_thick;
    unsigned lasts_step_initiate;
    unsigned lasts_step_braking;
    Point pd_shift;
};
void save(tptree&, const TourInput&);
void load(tptree&, TourInput&);
//--------------------------------------------------------------------------------
struct TargetInput
{
    tstring type;
    unsigned hn_aims, vn_aims;
    double left, right;
    double top, bottom;
};
void save(tptree&, const TargetInput&);
void load(tptree&, TargetInput&);
//--------------------------------------------------------------------------------
void save(tptree&, const Robo::MotionLaws::JointMotionLaw&);
void load(tptree&, Robo::MotionLaws::JointMotionLaw&);
//--------------------------------------------------------------------------------
void save(tptree&, const Robo::JointInput&);
void load(tptree&, Robo::JointInput&);
//--------------------------------------------------------------------------------
void save(tptree&, const Robo::NewHand::Hand::JointInput&);
void load(tptree&, Robo::NewHand::Hand::JointInput&);
//--------------------------------------------------------------------------------
void save(tptree&, const Robo::Mobile::Tank::JointInput&);
void load(tptree&, Robo::Mobile::Tank::JointInput&);
//--------------------------------------------------------------------------------
void save(tptree &root, const tstring &robo_name, const Point& robo_base,
          const std::list<std::shared_ptr<Robo::JointInput>> &robo_joints);
void load(tptree &root, tstring &robo_name, Point& robo_base,
          std::list<std::shared_ptr<Robo::JointInput>> &robo_joints);
//--------------------------------------------------------------------------------
}
