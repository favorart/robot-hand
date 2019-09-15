#include "StdAfx.h"
#include "Test.h"
#include "Hand.h"
#include "Tank.h"
#include "HandMotionLaws.h"

using namespace std;
using namespace test;
using namespace Robo;
using namespace Robo::MotionLaws;
using namespace RoboPos;
using namespace RoboMoves;


//------------------------------------------------------
void test::Test::restart()
{

}

//------------------------------------------------------
void test::Test::config()
{}

//------------------------------------------------------
test::Test::Test()
{
    config();
    restart();
}

//------------------------------------------------------
MLaw& operator++(MLaw &mlaw)
{ return (mlaw = static_cast<MLaw>(static_cast<int>(mlaw) + 1)); }

//------------------------------------------------------
auto test::Test::makeRobot(MLaw mlaw) -> pRoboI
{
    switch (params.ROBO_TYPE)
    {
    case RoboType::Hand:
    {
        const Point bases[] = { {-0.75, 1.05},{-0.70, 1.00},{ 0.10, 0.85 },{0.75, 0.25} };
        const double dHandDrivesDistance[] = { 1.74533, 2.3562, 1.8326, 0.1 };

        JointsInputsPtrs joints;
        for (joint_t j = 0; j < params.N_JOINTS; ++j)
        {
            auto law = JointMotionLaw{ mlaw, 300, dHandDrivesDistance[j], 0.35 }; // TODO: <<
            auto pji = std::make_shared<JointInput>(j, law, true, bases[j]);
            joints.push_back(pji);
        }
        Point base{ 0.75, 0.25 };
        return pRoboI{ new Robo::NewHand::Hand(base, joints) };
    }
    case RoboType::Tank:
    {
        Point base;
        JointsInputsPtrs joints; // TODO: <<
        return pRoboI{ new Robo::Mobile::Tank(base, joints) };
    }
    default:
        CERROR("Invalid robot!");
        return pRoboI{ nullptr };
    }
}

//------------------------------------------------------
void test::Test::testAll()
{
    params.ROBO_TYPE = RoboType::Tank;
    params.N_JOINTS = 2;
    params.ENVIROMENT = MUTIAL_BLOCKING|MUTIAL_DYNAMICS|EDGES; // WINDY|START_FRICTION

    params.FRAMES.resize(params.N_JOINTS);

    {
        testMLAW();
        testNFrames();
    }

    params.ROBO_TYPE = RoboType::Hand;
    params.N_JOINTS = 2;
    {
        testMLAW();
        testNFrames();    
    }

    params.ROBO_TYPE = RoboType::Hand;
    params.N_JOINTS = 3;
    {
        testMLAW();
        testNFrames();
    }

    //params.ROBO_TYPE = RoboType::Hand;
    //params.N_JOINTS = 4;
    //{
    //    testMLAW();
    //    testNFrames();
    //}
}

//------------------------------------------------------
void test::Test::testMLAW()
{
    unique_ptr<TargetI> pTarget(new RecTarget(params.TARGET_N_ROWS, params.TARGET_N_COLS, 
                                             params.TARGET_LFT, params.TARGET_RGH, 
                                             params.TARGET_TOP, params.TARGET_BTM));
    printConfig();
    for (MLaw mlaw = MLaw::SLOW; mlaw < MLaw::_COUNT_; ++mlaw)
    {
        Store store;

        pRoboI pRobo = makeRobot(mlaw);
        pRobo->setEnvCond(params.ENVIROMENT);

        LearnMoves lm(store, *pRobo, *pTarget, _T(".\\Resource\\Tour.txt"));

        lm.STAGE_1();
        printStat1(store, *pRobo);

        lm.STAGE_2();
        printStat2(store, *pRobo);

        Robo::Trajectory uncovered;
        lm.STAGE_3(uncovered);
        printStat3(store, *pRobo, uncovered);
    }
}

//------------------------------------------------------
void test::Test::testNFrames()
{}

//------------------------------------------------------
void printMotionLaw(const Robo::MotionLaws::JointMotionLaw &law, joint_t j)
{
    tcout << setw(8) << "MOTION_LAW" << setw(2) << j << " = " << law << endl;
}

//------------------------------------------------------
void printLMAdmix(LMAdmix admix)
{
    cout << "GRADIENT";
    const char *outputs[] = { "WMEAN", "RUNDOWN", "MAINDIR" };
    for (int i = 1, j = 0; i < Enviroment::_LAST_; i <<= 1, ++j)
        if (i & admix)
            cout << "|" << outputs[j];
}

//------------------------------------------------------
void Robo::printEnviroment(Enviroment env)
{
    if (env == NOTHING)
    {
        cout << "NOTHING";
        return;
    }
    const char *outputs[] = { "WINDY", "OPPOSITE_HANDLE", "MUTIAL_BLOCKING", "MUTIAL_DYNAMICS",
        "MOMENTUM_CHANGES" , "SYSTEMATIC_CHANGES", "START_FRICTION", "EDGES", "WEATHER" };

    for (int i = 1, j = 0; i < Enviroment::_LAST_; i <<= 1, ++j)
        if (i & env)
            cout << ((i == 1) ? "|" : "") << outputs[j];
}

//------------------------------------------------------
void test::Test::printConfig() const
{
    cout << setw(10) << "ROBO_TYPE" << " = " << ((params.ROBO_TYPE == RoboType::Hand) ? "Hand" : "Tank") << endl;
    cout << setw(10) << "N_JOINTS"  << " = " << params.N_JOINTS << endl;
    cout << setw(10) << "N_MUSCLES" << " = " << params.N_MUSCLES() << endl;
    
    cout << setw(10) << "ENVIROMENT" << " = ";
    printEnviroment(params.ENVIROMENT);
    cout << endl;
    
    for (joint_t j = 0; j < params.N_JOINTS; ++j)
    {
        //cout << setw(8) << "N_MOVE_FRAMES"  << setw(2) << j << " = " << params.N_MOVE_FRAMES[j] << endl;
        //cout << setw(8) << "MAX_MOVE_FRAME" << setw(2) << j << " = " << params.MAX_MOVE_FRAME[j] << endl;
        // Robo::MotionLaws::JointMotionLaw frames
        printMotionLaw(params.FRAMES[j], j);
    }
    cout << endl;

    cout << setw(10) << "TARGET_N_ROWS" << " = " << params.TARGET_N_ROWS << endl;
    cout << setw(10) << "TARGET_N_COLS" << " = " << params.TARGET_N_COLS << endl;
    cout << setw(10) << "TARGET_LFT"    << " = " << params.TARGET_LFT   << endl;
    cout << setw(10) << "TARGET_RGH"    << " = " << params.TARGET_RGH   << endl;
    cout << setw(10) << "TARGET_TOP"    << " = " << params.TARGET_TOP   << endl;
    cout << setw(10) << "TARGET_BTM"    << " = " << params.TARGET_BTM   << endl << endl;

    cout << setw(10) << "LM_N_TRIES"   << " = " << params.LM_N_TRIES << endl;
    cout << setw(10) << "LM_TRY_BREK"  << " = " << params.LM_TRY_BREK << endl;
    cout << setw(10) << "LM_TRY_RAND"  << " = " << params.LM_TRY_RAND << endl;
    cout << setw(10) << "LM_SIDE"      << " = " << params.LM_SIDE << endl;
    cout << setw(10) << "LM_SIDE_DECR" << " = " << params.LM_SIDE_DECR << endl;

    cout << setw(10) << "LM_USE_WMEAN" << " = ";
    printLMAdmix(params.LM_USE_WMEAN);
    cout  << endl << endl;
}

//------------------------------------------------------
void test::Test::printStat1(const Store &store, const RoboI &robo) const
{
    cout << "STAGE 1 --- Store.sz=" << store.size() << endl;


}
//------------------------------------------------------
void test::Test::printStat2(const Store &store, const RoboI &robo) const
{
    cout << "STAGE 2 --- Store.sz=" << store.size() << endl;



}
//------------------------------------------------------
void test::Test::printStat3(const Store &store, const RoboI &robo, const Trajectory &uncovered) const
{
    cout << "STAGE 3 --- Store.sz=" << store.size() << endl;



}
