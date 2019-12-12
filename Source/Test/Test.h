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
//------------------------------------------------------
enum class RoboType : uint8_t { 
    None=0, Tank=1, Hand/*2=2, Hand3=3, Hand4=4*/, 
    _LAST_=3
};
constexpr std::array<const TCHAR*, size_t(RoboType::_LAST_)> RoboTypeOutputs = {
    _T("NONE"), _T("TANK"), _T("HAND")
};

//------------------------------------------------------
enum class LMAdmix : uint8_t { 
    NOADMIX=0, GRADIENT=(1<<0), WMEAN=(1<<1), RUNDOWN=(1<<2), MAINDIR=(1<<3), 
    _LAST_=5
};
constexpr std::array<const TCHAR*, size_t(LMAdmix::_LAST_)> LMAdmixOutputs = {
    _T("NOADMIX"), _T("GRADIENT"), _T("WMEAN"), _T("RUNDOWN"), _T("MAINDIR")
};

//------------------------------------------------------
enum LMTours : uint8_t {
    TOUR1_GRID, TOUR1_EVO, TOUR1_EVOSTEP, TOUR1_RL, TOUR2_TARGET
};
constexpr TCHAR* LMToursOutputs[] = {
    _T("TOUR1_GRID"), _T("TOUR1_EVO"), _T("TOUR1_EVOSTEP"), _T("TOUR1_RL"), _T("TOUR2_TARGET")
};

//------------------------------------------------------
constexpr TCHAR* VerboseLevelOutputs[] = {
    _T("COUTALL"), _T("CDEBUG"), _T("CINFO"), _T("CWARN"), _T("CERROR"), _T("CALERT")
};
int scanVerboseLevel(const tstring&);

//------------------------------------------------------
struct Params
{
    RoboType          ROBO_TYPE{ RoboType::None };
    Point             ROBO_BASE{};
    Robo::joint_t     N_JOINTS{ 0 };
    Robo::muscle_t    N_MUSCLES() const { return 2*N_JOINTS; }
    Robo::Enviroment  ENVIROMENT{ Robo::Enviroment::NOTHING };

    Robo::JointsInputsPtrs JINPUTS{}; //size()==N_JOINTS
    
    size_t TARGET_N_ROWS{ 20/*200*/ };
    size_t TARGET_N_COLS{ 20/*200*/ };
    double TARGET_LFT{ -0.41 };
    double TARGET_RGH{ +0.43 };
    double TARGET_TOP{ -0.03 };
    double TARGET_BTM{ -0.85 };

    unsigned         LM_N_TRIES   = 8;
    unsigned         LM_TRY_BREK  = 100;
    unsigned         LM_TRY_RAND  = 3;
    Robo::distance_t LM_SIDE      = 0.2;
    Robo::distance_t LM_SIDE_DECR = 0.005;
    LMAdmix          LM_ADMIXES   = LMAdmix::WMEAN;
    tstring          LM_CONFIG_FN{};
    tstring          STORE_LOAD_FN{};
    tstring          GNUPLOT_PATH{};
    int              VERBOSE_LEVEL{};
    
    void scanLaws(tptree&);
    void scan(tptree&);
};

//------------------------------------------------------
class Test
{
    Params params;
    //RoboPos::LearnMoves lm;
public:
    Robo::pRoboI makeRobot();
    void restart();

    tptree readTestsFile(const tstring &tests_file);
    void printConfig() const;
    static void plotStoreAdj(const RoboMoves::adjacency_ptrs_t& range, const Point &aim, const Point &hit);
    void plotStoreState(const RoboMoves::Store&, const tstring &test_name);
    void plotRobotMotionLaw(const Robo::RoboI&, const tstring &test_name);

    void printStat2(const RoboMoves::Store&, const Robo::RoboI&) const;
    void printStat1(const RoboMoves::Store&, const Robo::RoboI&) const;
    void printStat3(const RoboMoves::Store&, const Robo::RoboI&, const Robo::Trajectory&) const;

    void testMotionLaws(const tstring &test_name);
    void testNFrames(const tstring &test_name);
    void testAll();

    Test(const tstring &testsfile);
    //virtual ~Test() {}
    Test(const Test&) = delete;
    Test& operator=(const Test&) = delete;
};
} // end namespace test

#ifdef WIN32
#include <winnt.h>
DEFINE_ENUM_FLAG_OPERATORS(test::LMAdmix)
#endif

