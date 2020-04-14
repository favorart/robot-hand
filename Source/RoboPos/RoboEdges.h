#include "StdAfx.h"

#pragma once

#include "Robo.h"

namespace Robo {
namespace NewHand {
class Hand;
}
namespace Mobile {
class Tank;
}
class RoboPhysics;

//-------------------------------------
class EnvEdges
{
protected:
    const RoboPhysics &robo_;
    const distance_t backpath_ratio_; ///< множитель реального изменения величины смещения для данного такта
    const distance_t damping_; ///< множитель, на который изменется prev_frame для вычисления смещения в будущий такт
    const distance_t borders_[2] = { (-1. + RoboI::minFrameMove), (+1. - RoboI::minFrameMove) }; ///< положение границ видимой области

    virtual bool isCollision(joint_t) const = 0;
    virtual bool checkBorders(joint_t) const = 0;
public:
    mutable bool collision{ false }; ///< сохранённый флаг - была ли коллизия в текущий такт
    EnvEdges(const RoboPhysics &robo, distance_t backpath, distance_t damping);
    virtual ~EnvEdges() {}
    virtual distance_t interaction(joint_t joint) const; ///< вычисление коэффициента взаимодейтсвия на перемещение модели робота в случае коллизии
};

//-------------------------------------
class EnvEdgesTank : public EnvEdges
{
protected:
    const Robo::Mobile::Tank& tank_;
    //const distance_t backpath_angle_;

    bool checkBorders(joint_t) const override;
    bool isCollision(joint_t) const override { return checkBorders(0); }
public:
    EnvEdgesTank(const RoboPhysics &robo, distance_t backpath = 20, distance_t damping = 3);
    distance_t interaction(joint_t joint) const override;
};

//-------------------------------------
class EnvEdgesHand : public EnvEdges
{
protected:
    const Robo::NewHand::Hand& hand_;

    bool checkFullOpened(joint_t) const;
    bool checkFullClosed(joint_t) const;
    bool checkBorders(joint_t) const override;
    bool isCollision(joint_t) const override;
public:
    EnvEdgesHand(const RoboPhysics &robo, distance_t backpath = 20, distance_t damping = 3);
    distance_t interaction(joint_t joint) const override;
};
} // end Robo
