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
void save(tptree &node, const Point& pt);
void load(tptree &node, Point& pt);
//--------------------------------------------------------------------------------
struct StageInput1
{
    double   draft_distance;
    unsigned lasts_init_value;
    unsigned lasts_incr_value;
};
//void save(tptree &node, const StageInput1& stage1);
//void load(tptree &node, StageInput1& stage1);
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
//void save(tptree &node, const StageInput3& pt);
//void load(tptree &node, StageInput3& stage3);
//--------------------------------------------------------------------------------
void save(tptree &root, const StageInput1& stage1, const StageInput2& stage2, const StageInput3& stage3);
void load(tptree &root, StageInput1& stage1, StageInput2& stage2, StageInput3& stage3);
//--------------------------------------------------------------------------------
struct TourInput
{
    tstring type;
    unsigned lasts_step_increment_thick;
    unsigned lasts_step_initiate;
    unsigned lasts_step_braking;
    Point pd_shift;
};
void save(tptree &root, const TourInput &tour);
void load(tptree &root, TourInput &tour);
//--------------------------------------------------------------------------------
struct TargetInput
{
    tstring type;
    unsigned hn_aims, vn_aims;
    double left, right;
    double top, bottom;
};
void save(tptree &root, const TargetInput &target);
void load(tptree &root, TargetInput &target);
//--------------------------------------------------------------------------------
void save(tptree &root, const Robo::MotionLaws::JointMotionLaw& ml);
void load(tptree &root, Robo::MotionLaws::JointMotionLaw& ml);
//--------------------------------------------------------------------------------
void save(tptree &root, const Robo::NewHand::Hand::JointInput&);
void load(tptree &root, Robo::NewHand::Hand::JointInput&);
//--------------------------------------------------------------------------------
void save(tptree &root, const Robo::Mobile::Tank::JointInput &input);
void load(tptree &root, Robo::Mobile::Tank::JointInput &input);
//--------------------------------------------------------------------------------
void save(tptree &root, const tstring &robo_name, const Point& robo_base,
          const std::list<std::shared_ptr<Robo::JointInput>> &robo_joints);
void load(tptree &root, tstring &robo_name, Point& robo_base,
          std::list<std::shared_ptr<Robo::JointInput>> &robo_joints);
//--------------------------------------------------------------------------------
}
