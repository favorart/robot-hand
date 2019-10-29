#pragma once

//#include "Robo.h"
#include "Hand.h"
#include "Tank.h"

namespace Robo {
namespace NewHand {
tstring muscleHelp(Hand::Muscle muscle);
tstring muscleName(Hand::Muscle muscle);
tstring jointName(Hand::Joint joint);
}

namespace Mobile {
tstring muscleHelp(Tank::Muscle muscle);
tstring muscleName(Tank::Muscle muscle);
tstring jointName(Tank::Joint joint);
}

tstring getJointName(const Robo::RoboI&, Robo::joint_t);
tstring getMuscleName(const Robo::RoboI&, Robo::muscle_t);
} // Robo
