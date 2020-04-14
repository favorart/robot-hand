#include "StdAfx.h"

#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <random>
#include <gsl/gsl_vector.h>

#include "rl.hpp"

#include "RoboRL.h"
#include "RoboRLSim.h"
#include "RoboPhysics.h"
//#include "rlException.hpp"

using namespace std::placeholders;
using namespace Robo;
//using namespace RoboMoves;
//using namespace rl_problem;

namespace rl_problem {
//------------------------------------------------------

// some exceptions for state and action consistancy
//class BadAction : public rl::exception::Any
//{
//public:
//    BadAction(std::string comment)
//        : Any(std::string("Bad action performed : ") + comment) {}
//};
//
//class BadState : public rl::exception::Any
//{
//public:
//    BadState(std::string comment)
//        : Any(std::string("Bad state found : ") + comment) {}
//};

//------------------------------------------------------
Robo::distance_t ObservationRobo::EpsPos = 0.0001;
Robo::distance_t ObservationRobo::EpsVel = 0.0000001;
Robo::distance_t ObservationRobo::EpsAcc = 0.0000001;

//------------------------------------------------------
bool ObservationRobo::compare(const ObservationRobo& o) const
{
    auto pred1 = [](const Point& l, const Point& r) { return l.hit(r, EpsPos); };
    if (!boost::equal(state.positions, o.state.positions, pred1))
        return false;

    auto pred2 = [](const Point& l, const Point& r) { return l.hit(r, EpsVel); };
    if (!boost::equal(state.velosities, o.state.velosities, pred2))
        return false;

    auto pred3 = [](const Point& l, const Point& r) { return l.hit(r, EpsAcc); };
    if (!boost::equal(state.accelerations, o.state.accelerations, pred3))
        return false;
    return true;
}

//------------------------------------------------------
bool ObservationRobo::compare(const Point &goal) const
{
    Point null{};
    return (goal.hit(state.positions[state.special_no], EpsPos))
        && (boost::all(state.velosities, [&null](const Point &p) { return p.hit(null, EpsVel); }))
        && (boost::all(state.accelerations, [&null](const Point &p) { return p.hit(null, EpsAcc); }));
}

//------------------------------------------------------

//double q_parametrized(const gsl_vector* theta, S s, A a)
//{ return gsl_vector_get(theta, TABULAR_Q_RANK(s, a)); }
//
//void grad_q_parametrized(const gsl_vector* theta,
//                         gsl_vector* grad_theta_sa,
//                         S s, A a)
//{ gsl_vector_set_basis(grad_theta_sa, TABULAR_Q_RANK(s, a)); } // all zeroes except one in (s,a)
//
//template<typename AITER, typename SCORES>
//double normalized_score(const S& s, const A& a,
//                        AITER action_begin, AITER action_end,
//                        const SCORES& scores)
//{
//    double score = exp(scores(s, a));
//    double Z = 0.0;
//    for (auto ai = action_begin; ai != action_end; ++ai)
//        Z += exp(scores(s, *ai));
//    return score / Z;
//}

//------------------------------------------------------
void printPhi(gsl_vector *phi, const S& s, const A& a)
{
    /*std::ofstream*/ tofstream fout(_T("phi.txt"), std::ios::app);
    fout << "a=" << int(a) << _T(' ') << s.state << std::endl;

    for (size_t i = 0; i < phi->size; ++i)
        fout << gsl_vector_get(phi, i) << _T(' ');
    fout << std::endl;
}

constexpr size_t N_COORDS = 2;
constexpr size_t N_ACTIONS = 4;
/*static constexpr*/ //int PHI_DIRECT_DIMENSION = robo.jointsCount()*3*2 + robo.musclesCount();
void phi_direct(gsl_vector *phi, const S& s, const A& a)
{
    //gsl_vector_set_zero(phi);
    gsl_vector_set_basis(phi, a); // all zeroes except one in (s,a)
    size_t j = N_ACTIONS;
    size_t n = N_ACTIONS / 2 * N_COORDS;
    for (size_t i = j; i < (j + n); i += 2)
    {
        auto k = (i - j) / 2u;
        gsl_vector_set(phi, i, s.state.positions[k].x);
        gsl_vector_set(phi, i + 1, s.state.positions[k].y);
    }
    j += n;
    for (size_t i = j; i < (j + n); i += 2)
    {
        auto k = (i - j) / 2u;
        gsl_vector_set(phi, i, s.state.velosities[k].x);
        gsl_vector_set(phi, i + 1, s.state.velosities[k].y);
    }
    j += n;
    for (size_t i = j; i < (j + n); i += 2)
    {
        auto k = (i - j) / 2u;
        gsl_vector_set(phi, i, s.state.accelerations[k].x);
        gsl_vector_set(phi, i + 1, s.state.accelerations[k].y);
    }
    //printPhi(phi, s, a);
}

//------------------------------------------------------
#define PHI_RBF_DIMENSION 30
void phi_rbf(gsl_vector *phi, const S& s, const A& a)
{
    //gsl_vector_set_zero(phi);
    gsl_vector_set_basis(phi, a); // all zeroes except one in (s,a)
    size_t j = N_ACTIONS;
    size_t n = N_ACTIONS / 2 * N_COORDS;
    for (size_t i = j; i < (j + n); i += 2)
    {
        auto k = (i - j) / 2u;
        gsl_vector_set(phi, i, s.state.positions[k].x);
        gsl_vector_set(phi, i + 1, s.state.positions[k].y);
    }
    j += n;
    for (size_t i = j; i < (j + n); i += 2)
    {
        auto k = (i - j) / 2u;
        gsl_vector_set(phi, i, s.state.velosities[k].x);
        gsl_vector_set(phi, i + 1, s.state.velosities[k].y);
    }
    j += n;
    for (size_t i = j; i < (j + n); i += 2)
    {
        auto k = (i - j) / 2u;
        gsl_vector_set(phi, i, s.state.accelerations[k].x);
        gsl_vector_set(phi, i + 1, s.state.accelerations[k].y);
    }
    //printPhi(phi, s, a);

    //std::array<double, 3> angle = { {-M_PI_4, 0, M_PI_4} };
    //std::array<double, 3> speed = { {-1,0,1} };
    //
    //int action_offset;
    //int i, j, k;
    //double dangle, dspeed;
    //
    //if (phi == (gsl_vector*)0)
    //    throw rl::exception::NullVectorPtr("in Feature::operator()");
    //else if ((int)(phi->size) != PHI_RBF_DIMENSION)
    //    throw rl::exception::BadVectorSize(phi->size, PHI_RBF_DIMENSION, "in Feature::operator()");
    //
    //switch (a)
    //{
    //case rl::problem::inverted_pendulum::Action::actionNone:
    //    action_offset = 0;
    //    break;
    //case rl::problem::inverted_pendulum::Action::actionLeft:
    //    action_offset = 10;
    //    break;
    //case rl::problem::inverted_pendulum::Action::actionRight:
    //    action_offset = 20;
    //    break;
    //default:
    //    throw rl::problem::inverted_pendulum::BadAction("in phi_gaussian()");
    //}
    //
    //gsl_vector_set_zero(phi);
    //for (i = 0, k = action_offset + 1; i < 3; ++i)
    //{
    //    dangle = s.angle - angle[i];
    //    dangle *= dangle;
    //    for (j = 0; j < 3; ++j, ++k)
    //    {
    //        dspeed = s.speed - speed[j];
    //        dspeed *= dspeed;
    //        gsl_vector_set(phi, k, exp(-.5*(dangle + dspeed)));
    //    }
    //    gsl_vector_set(phi, action_offset, 1);
    //}
}

//------------------------------------------------------

// This is an output iterator that notifies the visited states.
// The use within the run function is like
// VisitNotifier v;
// *(v++) = transition(s,a,r,s');
//
// Here, we will use transition(s,a,r,s') = s, see the lambda
// functions given to the run function.
template <typename S>
struct VisitNotifier
{
    StateTrajectory &t_;
    VisitNotifier(StateTrajectory &t) : t_(t) {}
    VisitNotifier(VisitNotifier &v) : t_(v.t_) {}

    VisitNotifier& operator*() { return *this; }
    VisitNotifier& operator++(int) { return *this; }

    void operator=(S &s) { t_.push_back(s); }
};

//------------------------------------------------------
double Reward::goalReward() const { return    0.; }
double Reward::stepReward() const { return   -1.; }
//double Reward::fallReward() const { return -100.; }
//double Reward::bumpReward() const { return  stepReward(); }
double Reward::collisionReward() const { return -10.; }
double Reward::reward(ObservationRobo &s, ObservationRobo &goal, bool isCollision) const
{
    return boost_distance(s(), goal()) * (isCollision ? -1. : -0.1); /*-100.*/
}
double Reward::fallReward() const { return -100000000.; }

//------------------------------------------------------
SimRobo::SimRobo(Robo::RoboI &robo, const Point &goal)
  : r(0.),
    robo(robo),
    goals(size_t{ 1 }, observation_type{ goal, robo.jointsCount(), 0 }),
    start(robo.getCurrState()),
    curr_goal(goals.front())
{
    restart();
}
//------------------------------------------------------
Robo::distance_t SimRobo::isGoal(const ObservationRobo &o) const
{ return o.compare(curr_goal); }
//------------------------------------------------------
void SimRobo::timeStep(const action_type &a)
{
    if (current == start)
        stepStart(a);
    else if (current == curr_goal)
    {
        stepGoal(a);
        // nextGoal ???
    }
    else
        step(a);
}
void SimRobo::stepGoal(const action_type &/*a*/)
{
    //r = rw.goalReward();
    r = rw.reward(current, curr_goal, robo.isCollision());
    throw rl::exception::Terminal("Transition to goal");
}
void SimRobo::stepStart(const action_type &a)
{
    if (a == 0)
    {
        r = rw.fallReward();
        throw rl::exception::Terminal("stopped");
    }
    step(a);
}
void SimRobo::step(const action_type &a)
{
    if (/*a == 0 &&*/ robo.moveEnd())
    {
        r = rw.fallReward();
        throw rl::exception::Terminal("stopped");
    }
    robo.step(Robo::RoboI::bitwise{ a }/*.actors*/);
    current = robo.getCurrState();
    //r = (robo.isCollision()) ? rw.stepReward() : rw.collisionReward();
    r = rw.reward(current, curr_goal, robo.isCollision());
}
void SimRobo::restart()
{
    r = 0.;
    robo.reset();
    current = robo.getCurrState();
}
void SimRobo::setPhase(const phase_type &phase)
{
    robo.setJoints(phase);
    current = robo.getCurrState();
}

//------------------------------------------------------
} // namespace rl_problem
