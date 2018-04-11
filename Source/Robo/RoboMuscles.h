#include "StdAfx.h"
#include "Hand.h"
#include "Tank.h"

#ifndef  _ROBO_MUSCLES_H_
#define  _ROBO_MUSCLES_H_
//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
}
#endif // _ROBO_MUSCLES_H_
