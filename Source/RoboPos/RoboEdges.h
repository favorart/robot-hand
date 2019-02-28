#include "StdAfx.h"

#pragma once

#include "Robo.h"
#include "Hand.h"
#include "Tank.h"

namespace Robo {
class EnvEdges
{
protected:
    const distance_t borders_[2] = { (-1. + RoboI::minFrameMove), (+1. - RoboI::minFrameMove) };
    distance_t offset_{};
    frames_t damping_{ 2 };
    frames_t lasts_{};
public:
    virtual ~EnvEdges() {}
    virtual void interaction() = 0;
    virtual void interaction(const Point&, const Point&, double) = 0;
};

class EnvEdgesTank : public EnvEdges
{
protected:
    Robo::Mobile::Tank &tank_;
    Point moved_{};
    frames_t max_lasts_{};
    bool border() const;
public:
    EnvEdgesTank(Robo::Mobile::Tank &tank) : max_lasts_(2 * tank.feedback.visitedRarity), tank_(tank) {}
    void interaction() {}
    void interaction(const Point&, const Point&, double);
};

class EnvEdgesHand : public EnvEdges
{
protected:
    Robo::NewHand::Hand &hand_;
    bool full_opened(joint_t) const;
    bool full_closed(joint_t) const;
    bool border(joint_t) const;
public:
    EnvEdgesHand(Robo::NewHand::Hand &hand) : hand_(hand) {}
    void interaction();
    void interaction(const Point&, const Point&, double) {}
};
}
