#pragma once

#include "Robo.h"
#include "RoboPhysics.h"
#include "RoboMotionLaws.h"
#include "RoboLearnMoves.h"
#include "RoboMovesStore.h"

// TODO:
//  + kd-tree
//  + tables of different configurations
//  + switch robo-motion-laws
//  + auto find all params for algorithm

//JointInput
//{
//    MotionLaws::JointMotionLaw frames{ nullptr, nullptr };
//    bool show = true;
//    Point base{};
//    size_t nMoveFrames{};
//    distance_t maxMoveFrame{};
//    size_t joint{}; // not in order of `joint_t`, but Type::Joints order
//}


namespace test {

enum class RoboType { Tank=1, Hand/*2=2, Hand3=3, Hand4=4*/ };
enum LMAdmix { GRADIENT=0, WMEAN=1, RUNDOWN=2, MAINDIR=4 };

struct Params
{
    RoboType ROBO_TYPE;
    Robo::joint_t N_JOINTS;
    Robo::muscle_t N_MUSCLES() const { return 2 * N_JOINTS; }
    Robo::Enviroment ENVIROMENT;

    std::vector<Robo::MotionLaws::JointMotionLaw> FRAMES;  // size() == N_JOINTS
    
    size_t TARGET_N_ROWS = 20;//200
    size_t TARGET_N_COLS = 20;//200
    double TARGET_LFT = -0.41;
    double TARGET_RGH = +0.43;
    double TARGET_TOP = -0.03;
    double TARGET_BTM = -0.85;

    unsigned         LM_N_TRIES   = 8;
    unsigned         LM_TRY_BREK  = 100;
    unsigned         LM_TRY_RAND  = 3;
    Robo::distance_t LM_SIDE      = 0.2;
    Robo::distance_t LM_SIDE_DECR = 0.005;
    LMAdmix          LM_USE_WMEAN = WMEAN;
    
    void config(Robo::joint_t n_joints)
    {
        N_JOINTS = n_joints;
        FRAMES.resize(n_joints);
    }
};

class Test
{
    using pRoboI = std::unique_ptr<Robo::RoboI>;
private:
    Params params;
    //RoboPos::LearnMoves lm;
    void config();
public:
    pRoboI makeRobot(Robo::MotionLaws::MLaw mlaw);
    void restart();

    void printConfig() const;
    void printStat2(const RoboMoves::Store&, const Robo::RoboI&) const;
    void printStat1(const RoboMoves::Store&, const Robo::RoboI&) const;
    void printStat3(const RoboMoves::Store&, const Robo::RoboI&, const Robo::Trajectory&) const;

    void testMLAW();
    void testNFrames();
    void testAll();

    Test();
    ~Test() {}
    Test(const Test&) = delete;
    Test& operator=(const Test&) = delete;
};
} // end namespace test+
