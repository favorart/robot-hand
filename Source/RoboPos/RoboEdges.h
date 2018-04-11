#include "StdAfx.h"

#pragma once

#include "Robo.h"
#include "Hand.h"
#include "Tank.h"

namespace Robo
{
class EnvEdges
{
protected:
    const Point LBorder, RBorder;
    Point oscillate;
    bool state;
public:
    EnvEdges() :
        LBorder{ (-1. + RoboI::minFrameMove), (-1. + RoboI::minFrameMove) },
        RBorder{ (+1. - RoboI::minFrameMove), (+1. - RoboI::minFrameMove) },
        oscillate{},
        state{ false }
    {}
    virtual bool interaction(RoboI &robo, const Point &vecBodyVelocity) = 0;
    virtual ~EnvEdges() {}
};

class EnvEdgesTank : public EnvEdges
{
public:
    bool interaction(RoboI &robo, const Point &vecBodyVelocity);
};

class EnvEdgesHand : public EnvEdges
{
public:
    bool interaction(RoboI &robo, const Point &vecBodyVelocity);
};

}