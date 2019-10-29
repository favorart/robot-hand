#pragma once

#include <gsl/gsl_vector.h>
#include "Robo.h"

class TargetI;
namespace RoboMoves {
class Store;
}
namespace rl_problem {
using S = ObservationRobo;
using A = Robo::muscle_t; //rl_problem::ActionRobo;
//------------------------------------------------------
void startRL(Robo::RoboI&, RoboMoves::Store&, TargetI&, Robo::StateTrajectories &show_trajs);
//------------------------------------------------------
void RoboRL(Robo::RoboI&,
            RoboMoves::Store&,
            TargetI&,
            Robo::StateTrajectories &show_trajs);
//------------------------------------------------------
void phi_direct(gsl_vector *phi, const S& s, const A& a);
//------------------------------------------------------
} // namespace rl_problem
