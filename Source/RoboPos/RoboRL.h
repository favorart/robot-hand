#pragma once

#include <gsl/gsl_vector.h>
#include "Robo.h"

class TargetI;
//------------------------------------------------------
namespace RoboMoves {
class Store;
}

namespace rl_problem {
//------------------------------------------------------
using S = ObservationRobo;
using A = Robo::muscle_t; //rl_problem::ActionRobo;
//------------------------------------------------------
void phi_direct(gsl_vector *phi, const S& s, const A& a);
//------------------------------------------------------
} // end namespace rl_problem
