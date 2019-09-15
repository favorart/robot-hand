#include "StdAfx.h"

#pragma once

#include "Robo.h"
#include "Hand.h"
#include "Tank.h"

namespace Robo {
class EnvEdges
{
protected:
    const distance_t backpath_ratio_;
    const distance_t damping_;
    const distance_t borders_[2] = { (-1. + RoboI::minFrameMove), (+1. - RoboI::minFrameMove) };
    virtual bool isCollision(bool, joint_t) const = 0;
public:
    bool collision{ false };
    EnvEdges(distance_t backpath, distance_t damping) : backpath_ratio_(backpath), damping_(damping) {}
    virtual ~EnvEdges() {}
    virtual void interaction(bool used, const Point &c={}, const Point &m={}, double a={}) = 0;
};

class EnvEdgesTank : public EnvEdges
{
protected:
    Robo::Mobile::Tank& tank_;
    const distance_t backpath_angle_{ 25 };
    bool border() const;
    bool isCollision(bool moves, joint_t) const override
    { return (moves && border()); }
public:
    EnvEdgesTank(Robo::Mobile::Tank &tank, distance_t backpath=20, distance_t damping=3) :
        EnvEdges(backpath, damping), tank_(tank)
    {}
    void interaction(bool used, const Point&, const Point&, double) override;
};

class EnvEdgesHand : public EnvEdges
{
protected:
    Robo::NewHand::Hand& hand_;
    bool full_opened(joint_t) const;
    bool full_closed(joint_t) const;
    bool border(joint_t) const;
    bool isCollision(bool moves, joint_t joint) const override
    { return (moves && (full_opened(joint) || full_closed(joint) || border(joint))); }
public:
    EnvEdgesHand(Robo::NewHand::Hand &hand, distance_t backpath=20, distance_t damping=3) :
        EnvEdges(backpath, damping), hand_(hand)
    {}
    void interaction(bool used, const Point&, const Point&, double) override;
};
}
