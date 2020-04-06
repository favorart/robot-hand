#pragma once

#include "Robo.h"

namespace rl_problem {
//------------------------------------------------------
struct ActionRobo final
{
    Robo::RoboI::bitwise actors;

    ActionRobo(unsigned long long val) : actors(val) {}
    operator unsigned long long() { return actors.to_ulong(); }

    //static bool insert(const Robo::Control &controls, Robo::frames_t t)
    //{
    //    for (auto& c : controls)
    //        return
    //}
    //ActionRobo(const Robo::Control &controls, Robo::frames_t t) {
    //    for (auto& m : actors)
    //        insert(controls, t);
    //}
    //ActionRobo(const Robo::Control &c) : actors(actors), b_acts(0) {}
    //int64_t nActions() const { return (1ULL << actors); }
    //bool setActs(unsigned acts) { b_acts = acts; }
    //bool getAct(unsigned i) const
    //{
    //    i = ((i >= actors) ? (actors - 1) : i); //  return last
    //    return ((b_acts & (1 << i)) != 0);
    //}
};

//------------------------------------------------------
struct ObservationRobo final
{
    static Robo::distance_t EpsPos;
    static Robo::distance_t EpsVel;
    static Robo::distance_t EpsAcc;

    int special_no{ 0 };
    std::vector<Point> positions{};
    std::vector<Point> velosities{};
    std::vector<Point> accelerations{};

    ObservationRobo() {}
    ObservationRobo(const Point &pos, size_t n_joints, int special_j = 0) :
        special_no(special_j),
        positions(n_joints, Point{}),
        velosities(n_joints, Point{}),
        accelerations(n_joints, Point{})
    {
        positions[special_no] = pos;
    }

    Point& operator()() { return positions[special_no]; }
    Point operator()() const { return positions[special_no]; }

    bool compare(const ObservationRobo& o) const;
    bool operator==(const ObservationRobo& o) const
    { return compare(o); }

    bool compare(const Point &goal) const;
    bool operator==(const Point& goal) const
    { return compare(goal); }
};

//------------------------------------------------------
struct Reward final
{
    double goalReward() const;
    double stepReward() const;
    //double fallReward() const;
    //double bumpReward() const;
    double Reward::reward(ObservationRobo &s,
                          ObservationRobo &goal,
                          bool isCollision) const;
    double collisionReward() const;
    double fallReward() const;
};

//------------------------------------------------------
class SimRobo final
{
    //static_assert<(Actors % 2) != 0>;
public:
    using phase_type = Robo::JointsOpenPercent;
    using observation_type = ObservationRobo;
    using action_type = Robo::muscle_t;//ActionRobo;
    using reward_type = double;

private:
    double r; ///< reward
    Robo::RoboI& robo; ///< robot model
    observation_type current;
    Reward rw;

    std::vector<observation_type> goals;
    observation_type start;
    observation_type& curr_goal;

    Robo::distance_t isGoal(const ObservationRobo &o) const;

public:
    void timeStep(const action_type &a);

    void stepGoal(const action_type &a);
    void stepStart(const action_type &a);
    void step(const action_type &a);
    void restart();
    void setPhase(const phase_type &phase);

    Robo::StateTrajectory trajectory() const { return robo.trajectory(); }
    reward_type reward() const { return r; }
    reward_type last_reward() const { return rw.fallReward(); }
    const observation_type& sense() const { return current; }

    SimRobo(Robo::RoboI &robo, const Point &goal);
    ~SimRobo() {}
    SimRobo(const SimRobo&) = delete;
    SimRobo& operator=(const SimRobo&) = delete;
};
} // end namespace rl_problem
