#include "StdAfx.h"
#include "Test.h"
#include "Hand.h"
#include "Tank.h"
#include "HandMotionLaws.h"
#include "RoboInputs.h"

using namespace std;
using namespace test;
using namespace Robo;
using namespace Robo::MotionLaws;
using namespace RoboPos;
using namespace RoboMoves;
//------------------------------------------------------
MLaw& operator++(MLaw &mlaw) { return (mlaw = static_cast<MLaw>(static_cast<int>(mlaw) + 1)); }

//------------------------------------------------------
using SetterF = std::function<void(const tstring&)>;
static
void scanParam(tptree &root, tstring &buf, const tstring &name, SetterF setter)
{
    buf = root.get_optional<tstring>(name).get_value_or(_T(""));
    //tcout << name << "\t=\t" << buf << std::endl;
    if (!buf.empty())
    {
        boost::trim(buf);
        if (buf.front() != _T('"') || buf.back() != _T('"'))
        {
            ba::replace_all(buf, " ", "");
            ba::replace_all(buf, "\t", "");
        }
        setter(buf);
    }
}

//------------------------------------------------------
using SetterArrF = std::function<void(const tstring&, int)>;
static
int scanArrParam(tptree &root, tstring &buf, const tstring &name, SetterArrF setter)
{
    buf = root.get_optional<tstring>(name).get_value_or(_T(""));
    //tcout << name << "\t=\t" << buf << "\t";
    int i = 0;
    if (!buf.empty())
    {
        ba::replace_all(buf, " ", "");
        ba::replace_all(buf, "\t", "");
        tseparator sep(_T("{},"));
        ttokenizer tokens(buf, sep);
        for (auto &tok : tokens)
        {
            //tcout << tok << " ";
            setter(tok, i++);
        }
    }
    //tcout << std::endl;
    return i;
}

//------------------------------------------------------
static
Point scanSubPoint(const tstring &buf)
{
    tseparator sep(_T(";"));
    ttokenizer tokens(buf, sep);
    Point p{};
    int i = 0;
    for (auto &tok : tokens)
    {
        //tcout << tok << ";";
        p[i++] = std::stod(tok);
    }
    return p;
};

//------------------------------------------------------
#define SCAN_FUNC_CALL(param,f)        [&p=param](const tstring&s){p=f(s);}
#define SCAN_PARAM(param,f)            scanParam(root,buf,_T(STRINGIFY(param)),SCAN_FUNC_CALL(param,f))

#define SCAN_FUNC_CALL2(param,f,outs)  [&p=param](const tstring&s){p=f(s,outs);}
#define SCAN_PARAM2(param,f,outs)      scanParam(root,buf,_T(STRINGIFY(param)),SCAN_FUNC_CALL2(param,f,outs))

#define SCAN_ARR_FUNC_CALL(param,f)    [&p=param](const tstring&s, int i){p[i]=f(s);}
#define SCAN_ARR_CHECK(name)           if(i!=0&&i!=N_JOINTS)CERROR("Invalid "<<name<<' '<<i);min_i=std::min(i,min_i)
#define SCAN_ARR_PARAM(name,param,f)   scanArrParam(root,buf,name,SCAN_ARR_FUNC_CALL(param,f))

//------------------------------------------------------
void test::Params::scanLaws(tptree &root)
{
    MLaw     mlaws          [Robo::RoboI::jointsMaxCount]{}; //MOTION_LAW    ={CONAC , CONAC  }//MLaw
    size_t   n_move_frames  [Robo::RoboI::jointsMaxCount]{}; //N_MOVE_FRAMES ={350   , 470    }//size_t
    double   move_distances [Robo::RoboI::jointsMaxCount]{}; //MOVE_DISTANCE ={2.3562, 1.8326 }//double
    double   inertia_ratio  [Robo::RoboI::jointsMaxCount]{}; //INERTIA_RATIO ={0.31  , 0.35   }//double=35% от общ.пробега
    tstring  stablev_ratio  [Robo::RoboI::jointsMaxCount]{}; //STAB_LEV_RATIO={0.55  , 0.55   }//double=55% от общ.пробега
    Point    j_input_bases  [Robo::RoboI::jointsMaxCount]{}; //JOINT_BASES   ={{-0.7;1.0},{0.1;0.85}}
    double   hand_def_poses [Robo::RoboI::jointsMaxCount]{}; //HAND_DEF_POSES={70.0  ,0.0     }
    joint_t  hand_use_joints[Robo::RoboI::jointsMaxCount]{}; //HAND_USE_JOINTS={  1  ,2       }//#{ELBOW ,SHOULDER}

    int      min_i{ N_JOINTS };
    int      i{};
    tstring  buf{};

    i = SCAN_ARR_PARAM(_T("MOTION_LAW"), mlaws, scanMLaw);
    SCAN_ARR_CHECK("MOTION_LAW");

    i = SCAN_ARR_PARAM(_T("N_MOVE_FRAMES"), n_move_frames, std::stoull);
    SCAN_ARR_CHECK("N_MOVE_FRAMES");

    i = SCAN_ARR_PARAM(_T("MOVE_DISTANCE"), move_distances, std::stod);
    SCAN_ARR_CHECK("MOVE_DISTANCE");

    i = SCAN_ARR_PARAM(_T("INERTIA_RATIO"), inertia_ratio, std::stod);
    SCAN_ARR_CHECK("INERTIA_RATIO");

    i = SCAN_ARR_PARAM(_T("STAB_LEV_RATIO"), stablev_ratio, [](auto p){return p;}/*std::stod*/);
    SCAN_ARR_CHECK("STAB_LEV_RATIO");

    i = SCAN_ARR_PARAM(_T("JOINT_BASES"), j_input_bases, scanSubPoint);
    SCAN_ARR_CHECK("JOINT_BASES");

    i = SCAN_ARR_PARAM(_T("HAND_DEF_POSES"), hand_def_poses, std::stod);
    SCAN_ARR_CHECK("HAND_DEF_POSES");

    i = SCAN_ARR_PARAM(_T("HAND_USE_JOINTS"), hand_use_joints, std::stoi);
    SCAN_ARR_CHECK("HAND_USE_JOINTS");

    if (min_i < N_JOINTS)
        return;

    for (joint_t j = 0; j < N_JOINTS; ++j)
    {
        std::shared_ptr<Robo::JointInput> rji;
        auto law = MotionLaws::JointMotionLaw(mlaws[j],
                                              n_move_frames[j],
                                              move_distances[j],
                                              inertia_ratio[j],
                                              stablev_ratio[j]);
        switch (ROBO_TYPE)
        {
        default: CERROR("Invalid robot!");
        case RoboType::Hand:
        {
            auto ji = std::make_shared<Robo::NewHand::Hand::JointInput>(hand_use_joints[j],
                                                                        true,
                                                                        j_input_bases[j],
                                                                        law,
                                                                        hand_def_poses[j]);
            rji = std::dynamic_pointer_cast<Robo::JointInput>(ji);
            break;
        }
        case RoboType::Tank:
        {
            auto ji = std::make_shared<Robo::Mobile::Tank::JointInput>(j, true, j_input_bases[j], law);
            rji = std::dynamic_pointer_cast<Robo::JointInput>(ji);
            break;
        }
        }
        JINPUTS.push_back(rji);
    }
}

//------------------------------------------------------
void test::Params::scan(tptree &root)
{
    tstring buf;
    SCAN_PARAM(TARGET_N_ROWS, std::stoull);
    SCAN_PARAM(TARGET_N_COLS, std::stoull);
    SCAN_PARAM(TARGET_LFT,    std::stod);
    SCAN_PARAM(TARGET_RGH,    std::stod);
    SCAN_PARAM(TARGET_TOP,    std::stod);
    SCAN_PARAM(TARGET_BTM,    std::stod);
    
    SCAN_PARAM(LM_N_TRIES,    std::stoul);
    SCAN_PARAM(LM_TRY_BREK,   std::stoul);
    SCAN_PARAM(LM_TRY_RAND,   std::stoul);
    SCAN_PARAM(LM_SIDE,       std::stod);
    SCAN_PARAM(LM_SIDE_DECR,  std::stod);
    SCAN_PARAM2(LM_ADMIXES,   scanEnumOneHot<LMAdmix>, LMAdmixOutputs);

    SCAN_PARAM(LM_CONFIG_FN,  [](auto&s){return s;});
    SCAN_PARAM(GNUPLOT_PATH,  [](auto&s){return s;});
    SCAN_PARAM(STORE_LOAD_FN, [](auto&s){return s;});
    SCAN_PARAM(N_JOINTS,      std::stoi);
    SCAN_PARAM2(ROBO_TYPE,    scanEnumOneHot<RoboType>, RoboTypeOutputs);
    SCAN_PARAM2(ENVIROMENT,   scanEnumOneHot<Robo::Enviroment>, Robo::enviroment_outputs);
    
    int res = SCAN_ARR_PARAM(_T("ROBO_BASE"), ROBO_BASE, std::stod);
    if (res != Point::ndimensions && res != 0)
        CERROR("Invalid ROBO_BASE");
    scanLaws(root);
}

//------------------------------------------------------
test::Test::Test(const tstring &testsfile)
{    
    //tptree root(tstring(std::istreambuf_iterator<wchar_t>(wifstream(testsfile)),
    //                    std::istreambuf_iterator<wchar_t>()));
    tptree root;
    tfstream fin = Utils::utf8_stream(testsfile, std::ios::in);
    if (!fin.is_open())
        throw std::runtime_error("read_config: file is not exist");
    pt::read_ini(fin, root);

    tptree node = root.get_child(_T("common"));
    params.scan(node);

    for (auto &test : root)
    {
        if (test.first == _T("common"))
            continue;
        tcout << "testing " << test.first << " .." << std::endl;
        params.scan(test.second);
        //restart();
        testMotionLaws(test.first);
        //testAll();
        // ==============================
        boost::this_thread::interruption_point();
        // ==============================
    }
}

//------------------------------------------------------
void test::Test::restart()
{

}

//------------------------------------------------------
Robo::pRoboI test::Test::makeRobot()
{
    if (params.JINPUTS.empty())
        CERROR("Invalid JINPUTS");
    switch (params.ROBO_TYPE)
    {
    case RoboType::Hand:
    {
        //return Robo::NewHand::Hand::make(Robo::NewHand::Hand::name(),\
              tptree(tstring(std::istreambuf_iterator<wchar_t>(wifstream("config.json")),\
                             std::istreambuf_iterator<wchar_t>())));
        return pRoboI{ new Robo::NewHand::Hand(params.ROBO_BASE, params.JINPUTS) };
    }
    case RoboType::Tank:
    {
        return pRoboI{ new Robo::Mobile::Tank(params.ROBO_BASE, params.JINPUTS) };
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
    params.ENVIROMENT = ENV::MUTIAL_BLOCKING|ENV::MUTIAL_DYNAMICS|ENV::EDGES; // ENV::WINDY|ENV::START_FRICTION

    //params.FRAMES.resize(params.N_JOINTS);
    tstring test_name;
    {
        testMotionLaws(test_name);
        testNFrames(test_name);
    }

    params.ROBO_TYPE = RoboType::Hand;
    params.N_JOINTS = 2;
    {
        testMotionLaws(test_name);
        testNFrames(test_name);
    }

    params.ROBO_TYPE = RoboType::Hand;
    params.N_JOINTS = 3;
    {
        testMotionLaws(test_name);
        testNFrames(test_name);
    }

    //params.ROBO_TYPE = RoboType::Hand;
    //params.N_JOINTS = 4;
    //{
    //    testMLAW(test_name);
    //    testNFrames(test_name);
    //}
}

//------------------------------------------------------
void test::Test::testMotionLaws(const tstring &test_name)
{
    Store store;
    unique_ptr<TargetI> pTarget(new RecTarget(params.TARGET_N_ROWS, params.TARGET_N_COLS,
                                              params.TARGET_LFT, params.TARGET_RGH,
                                              params.TARGET_TOP, params.TARGET_BTM));

    auto pRobo = makeRobot();
    //pRobo->setEnvCond(params.ENVIROMENT);

    printConfig();
    plotRobotMotionLaw(*pRobo, test_name);

    CINFO("Read Config LM...");
    LearnMoves lm(store, *pRobo, *pTarget, params.LM_CONFIG_FN);

    CINFO("STAGE 1...");
    lm.STAGE_1();
    printStat1(store, *pRobo);

    CINFO("STAGE 2...");
    lm.STAGE_2();
    printStat2(store, *pRobo);

    CINFO("STAGE 3...");
    Robo::Trajectory uncovered;
    lm.STAGE_3(uncovered);
    printStat3(store, *pRobo, uncovered);
    // ==============================
    boost::this_thread::interruption_point();
    // ==============================
}

//------------------------------------------------------
void test::Test::testNFrames(const tstring &test_name)
{

}

//------------------------------------------------------
void test::Test::plotRobotMotionLaw(Robo::RoboI &robo, const tstring &test_name)
{
    std::ostringstream ss;
    ss << "test-motion-law-";
    ss << Utils::ununi(test_name) << '-';
    ss << Utils::ununi(robo.getName()) << '-';
    for (auto &pji : params.JINPUTS)
        ss << Utils::ununi(Robo::MotionLaws::name(pji->frames.type)) << '-';
    ss << Utils::now(); /*append ext!*/
    //-------------------------
    RoboPhysics *rphy = dynamic_cast<RoboPhysics*>(&robo);    
    rphy->plotMotionLaws(Utils::uni(ss.str()), RoboPhysics::jointsAll);
    //-------------------------
#if !defined(GNUPLOT_SILENCE)
    std::string filename = ss.str();
    ss.str("");

    ss << "\"start \"GNUPLOT\" " << Utils::ununi(params.GNUPLOT_PATH);
    ss << " \"" << filename << ".plt\" --persist\"" << std::endl;
    //-------------------------
    CINFO(Utils::uni(ss.str()));
    std::system(ss.str().c_str());
    //-------------------------
#endif // !GNUPLOT_SILENCE
}

//------------------------------------------------------
void test::Test::printConfig() const
{
    const std::streamsize w = 20;
    cout << setw(w) << "ROBO_TYPE" << " = " << ((params.ROBO_TYPE == RoboType::Hand) ? "Hand" : "Tank") << endl;
    cout << setw(w) << "N_JOINTS"  << " = " << int(params.N_JOINTS) << endl;
    cout << setw(w) << "N_MUSCLES" << " = " << int(params.N_MUSCLES()) << endl;    
    cout << setw(w) << "ENVIROMENT" << " = ";
    printEnumOneHot<Enviroment>(params.ENVIROMENT, Robo::enviroment_outputs);
    cout << endl;

    auto jit = params.JINPUTS.cbegin();
    for (joint_t j = 0; j < params.N_JOINTS; ++j, ++jit)
    {
        tcout << setw(w-2) << "MOTION_LAW" << setw(2) << j << " = " << (**jit).frames << "\t"
              << setw(12) << "JOINT_BASES" << setw(2) << j << " = " << (**jit).base << endl;
        //if (ROBO_TYPE == RoboType::Hand)
        //    tcout << setw(8) << "HAND_DEF_POSES" << setw(2) << j << " = " << robo->status.def_pos() << endl;
    }
    cout << endl;

    cout << setw(w) << "TARGET_N_ROWS" << " = " << params.TARGET_N_ROWS << endl;
    cout << setw(w) << "TARGET_N_COLS" << " = " << params.TARGET_N_COLS << endl;
    cout << setw(w) << "TARGET_LFT"    << " = " << params.TARGET_LFT   << endl;
    cout << setw(w) << "TARGET_RGH"    << " = " << params.TARGET_RGH   << endl;
    cout << setw(w) << "TARGET_TOP"    << " = " << params.TARGET_TOP   << endl;
    cout << setw(w) << "TARGET_BTM"    << " = " << params.TARGET_BTM   << endl << endl;

    cout << setw(w) << "LM_N_TRIES"   << " = " << params.LM_N_TRIES << endl;
    cout << setw(w) << "LM_TRY_BREK"  << " = " << params.LM_TRY_BREK << endl;
    cout << setw(w) << "LM_TRY_RAND"  << " = " << params.LM_TRY_RAND << endl;
    cout << setw(w) << "LM_SIDE"      << " = " << params.LM_SIDE << endl;
    cout << setw(w) << "LM_SIDE_DECR" << " = " << params.LM_SIDE_DECR << endl;
    cout << setw(w) << "LM_ADMIXES" << " = ";
    printEnumOneHot<LMAdmix>(params.LM_ADMIXES, LMAdmixOutputs);
    cout << endl;
    tcout << setw(w) << _T("LM_CONFIG_FN") << _T(" = ") << params.LM_CONFIG_FN << endl;
    tcout << setw(w) << _T("STORE_LOAD_FN") << _T(" = ") << params.STORE_LOAD_FN << endl;
    cout << endl << endl;
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
