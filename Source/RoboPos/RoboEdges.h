#include "StdAfx.h"

#pragma once

#include "Robo.h"
#include "Hand.h"
#include "Tank.h"

namespace Robo {
class EnvEdges
{
protected:
    bool used_{ true };
    const frames_t damping_{ 2 };
    const distance_t borders_[2] = { (-1. + RoboI::minFrameMove),
                                     (+1. - RoboI::minFrameMove) };
public:
    EnvEdges(bool used) : used_(used) {}
    virtual ~EnvEdges() {}
    virtual void interaction() = 0;
    virtual void interaction(const Point&, const Point&, double) = 0;
};

class EnvEdgesTank : public EnvEdges
{
protected:
    Robo::Mobile::Tank& tank_;
    const distance_t backpath_ratio_{ 50 };
    const distance_t backpath_angle_{ 25 };
    bool border() const;
public:
    EnvEdgesTank(Robo::Mobile::Tank &tank, bool used) : EnvEdges(used), tank_(tank) {}
    void interaction() {}
    void interaction(const Point&, const Point&, double);
};

class EnvEdgesHand : public EnvEdges
{
protected:
    Robo::NewHand::Hand& hand_;
    const distance_t backpath_ratio_{ 20 };
    bool full_opened(joint_t) const;
    bool full_closed(joint_t) const;
    bool border(joint_t) const;
public:
    EnvEdgesHand(Robo::NewHand::Hand &hand, bool used) : EnvEdges(used), hand_(hand) {}
    void interaction();
    void interaction(const Point&, const Point&, double) {}
};
}
