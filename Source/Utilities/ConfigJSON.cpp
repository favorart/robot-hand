#include "StdAfx.h"
#include "ConfigJSON.h"
#include "RoboMuscles.h"
#include "HandMotionLaws.h"

#include "RoboInputs.h"

//--------------------------------------------------------------------------------
void ConfigJSON::save(tptree &node, const Point& pt)
{
    tptree xelem, yelem;
    xelem.put_value(pt.x); node.push_back(std::make_pair(_T(""), xelem));
    yelem.put_value(pt.y); node.push_back(std::make_pair(_T(""), yelem));
}
void ConfigJSON::load(tptree &node, Point& pt)
{
    assert(node.size() == 2);
    pt.x = node.front().second.get_value<double>();
    pt.y = node.back().second.get_value<double>();
}
//--------------------------------------------------------------------------------
namespace ConfigJSON
{
void save(tptree &node, const StageInput1& stage1);
void load(tptree &node, StageInput1& stage1);

void save(tptree &node, const StageInput3& pt);
void load(tptree &node, StageInput3& stage3);
}
//--------------------------------------------------------------------------------
void ConfigJSON::save(tptree &node, const StageInput1& stage1)
{
    node.put(_T("draft_distance"),   stage1.draft_distance);
    node.put(_T("lasts_init_value"), stage1.lasts_init_value);
    node.put(_T("lasts_incr_value"), stage1.lasts_incr_value);
}
void ConfigJSON::load(tptree &node, StageInput1& stage1)
{
    //assert(node.size() == 3);
    stage1.draft_distance   = node.get<double>  (_T("draft_distance"));
    stage1.lasts_init_value = node.get<unsigned>(_T("lasts_init_value"));
    stage1.lasts_incr_value = node.get<unsigned>(_T("lasts_incr_value"));
}
//--------------------------------------------------------------------------------
void ConfigJSON::save(tptree &node, const StageInput3& stage3)
{
    node.put(_T("tries")              , stage3.tries              );
    node.put(_T("tries_break")        , stage3.tries_break        );
    node.put(_T("random_try")         , stage3.random_try         );

    node.put(_T("side")               , stage3.side               );
    node.put(_T("side_decrease_step") , stage3.side_decrease_step );
    node.put(_T("use_weighted_mean")  , stage3.use_weighted_mean  );
}
void ConfigJSON::load(tptree &node, StageInput3& stage3)
{
    //assert(node.size() == 6);
    stage3.tries               = node.get<unsigned> (_T("tries"));
    stage3.tries_break         = node.get<unsigned> (_T("tries_break"));
    stage3.random_try          = node.get<unsigned> (_T("random_try"));

    stage3.side                = node.get<double>   (_T("side"));
    stage3.side_decrease_step  = node.get<double>   (_T("side_decrease_step"));
    stage3.use_weighted_mean   = node.get<bool>     (_T("use_weighted_mean"));
}
//--------------------------------------------------------------------------------
void ConfigJSON::save(tptree &root, const StageInput1& stage1, const StageInput2& stage2, const StageInput3& stage3)
{
    {
        tptree node;
        ConfigJSON::save(node, stage1);
        root.add_child(_T("stage1"), node);
    }
    {
        tptree node;
        ConfigJSON::save(node, stage2);
        root.add_child(_T("stage2"), node);
    }
    {
        tptree node;
        ConfigJSON::save(node, stage3);
        root.add_child(_T("stage3"), node);
    }
}
void ConfigJSON::load(tptree &root, StageInput1& stage1, StageInput2& stage2, StageInput3& stage3)
{
    {
        auto &node = root.get_child(_T("stage1"));
        ConfigJSON::load(node, stage1);
    }
    {
        auto &node = root.get_child(_T("stage2"));
        ConfigJSON::load(node, stage2);
    }
    {
        auto &node = root.get_child(_T("stage3"));
        ConfigJSON::load(node, stage3);
    }
}
//--------------------------------------------------------------------------------
void ConfigJSON::save(tptree &root, const TargetInput &target)
{
    tptree node;
    node.put(_T("type"), target.type);
    node.put(_T("hn_aims"), target.hn_aims);
    node.put(_T("vn_aims"), target.vn_aims);
    node.put(_T("left"), target.left);
    node.put(_T("right"), target.right);
    node.put(_T("top"), target.top);
    node.put(_T("bottom"), target.bottom);
    root.add_child(_T("target"), node);
}
void ConfigJSON::load(tptree &root, TargetInput &target)
{
    auto &node = root.get_child(_T("target"));
    //assert(node.size() == 6);
    target.type = node.get<tstring>(_T("type"));
    target.hn_aims = node.get<unsigned>(_T("hn_aims"));
    target.vn_aims = node.get<unsigned>(_T("vn_aims"));
    target.left = node.get<double>(_T("left"));
    target.right = node.get<double>(_T("right"));
    target.top = node.get<double>(_T("top"));
    target.bottom = node.get<double>(_T("bottom"));
}
//--------------------------------------------------------------------------------
void ConfigJSON::save(tptree &root, const TourInput &tour)
{
    tptree node;
    node.put(_T("type"), tour.type);
    node.put(_T("lasts_step_increment_thick"), tour.lasts_step_increment_thick);
    node.put(_T("lasts_step_initiate"), tour.lasts_step_initiate);
    node.put(_T("lasts_step_braking"), tour.lasts_step_braking);
    root.add_child(_T("tour"), node);
}
void ConfigJSON::load(tptree &root, TourInput &tour)
{
    auto &node = root.get_child(_T("tour"));
    //assert(node.size() == 5);
    tour.type = node.get<tstring>(_T("type"));
    tour.lasts_step_increment_thick = node.get<unsigned>(_T("lasts_step_increment_thick"));
    tour.lasts_step_initiate = node.get<unsigned>(_T("lasts_step_initiate"));
    tour.lasts_step_braking = node.get<unsigned>(_T("lasts_step_braking"));
}
//--------------------------------------------------------------------------------
void ConfigJSON::save(tptree &root, const Robo::MotionLaws::JointMotionLaw& ml)
{
    root.put(_T("type"),  ml.type);
    root.put(_T("name"),  ml.typeName);
    root.put(_T("param"), ml.param);
    root.put(_T("stopD"), ml.stopDistanceRatio);
}
void ConfigJSON::load(tptree &root, Robo::MotionLaws::JointMotionLaw& ml)
{
    assert(root.size() == 4);
    auto type = static_cast<Robo::MotionLaws::HandMLaw>(root.get<uint8_t>(_T("type")));
    auto param = root.get<tstring>(_T("param"));
    //typeName = root.get<tstring>(_T("name"));
    ml = Robo::MotionLaws::getHandMLaw(type, param);
    ml.stopDistanceRatio = root.get<double>(_T("stopD"));
}
//--------------------------------------------------------------------------------
void ConfigJSON::save(tptree &root, const Robo::NewHand::Hand::JointInput &input)
{
    root.put(_T("name"), Robo::NewHand::jointName(input.joint));
    root.put<uint8_t>(_T("joint"), static_cast<uint8_t>(input.joint));
    root.put(_T("defPoseRatio"), input.defaultPose);
    ConfigJSON::save(root, dynamic_cast<const Robo::JointInput&>(input));
}
void ConfigJSON::load(tptree &root, Robo::NewHand::Hand::JointInput &input)
{
    assert(root.size() == 9);
    //input.name
    input.joint = static_cast<Robo::NewHand::Hand::Joint>(root.get<uint8_t>(_T("joint")));
    input.defaultPose = root.get<double>(_T("defPoseRatio"));
    ConfigJSON::load(root, dynamic_cast<Robo::JointInput&>(input));
}
//--------------------------------------------------------------------------------
void ConfigJSON::save(tptree &root, const Robo::Mobile::Tank::JointInput &input)
{
    root.put(_T("name"), Robo::Mobile::jointName(input.joint));
    root.put<uint8_t>(_T("joint"), static_cast<uint8_t>(input.joint));
    ConfigJSON::save(root, dynamic_cast<const Robo::JointInput&>(input));
}
void ConfigJSON::load(tptree &root, Robo::Mobile::Tank::JointInput &input)
{
    //assert(root.size() == 7);
    //input.name
    input.joint = static_cast<Robo::Mobile::Tank::Joint>(root.get<uint8_t>(_T("joint")));
    ConfigJSON::load(root, dynamic_cast<Robo::JointInput&>(input));
}
//--------------------------------------------------------------------------------
void ConfigJSON::save(tptree &root, const Robo::JointInput &input)
{
    root.put(_T("type"), input.type);
    root.put(_T("show"), input.show);

    tptree base;
    ConfigJSON::save(base, input.base);
    root.add_child(_T("base"), base);

    root.put(_T("nMoveFrames"), input.nMoveFrames);
    root.put(_T("maxMoveFrame"), input.maxMoveFrame);

    tptree ml;
    ConfigJSON::save(ml, input.frames);
    root.add_child(_T("motionLaw"), ml);
}
void ConfigJSON::load(tptree &root, Robo::JointInput &input)
{
    input.type = root.get<Robo::joint_t>(_T("type"));
    input.show = root.get<bool>(_T("show"));

    ConfigJSON::load(root.get_child(_T("base")), input.base);

    input.nMoveFrames = root.get<size_t>(_T("nMoveFrames"));
    input.maxMoveFrame = root.get<double>(_T("maxMoveFrame"));

    ConfigJSON::load(root.get_child(_T("motionLaw")), input.frames);
}
//--------------------------------------------------------------------------------
void ConfigJSON::save(tptree &root, const tstring &robo_name, const Point& robo_base,
                      const std::list<std::shared_ptr<Robo::JointInput>> &robo_joints)
{
    tptree robo;
    robo.put(_T("name"), robo_name);

    tptree base;
    ConfigJSON::save(base, robo_base);
    robo.add_child(_T("base"), base);

    tptree joints;
    for (const auto &pinput : robo_joints)
    {
        if (robo_name == _T("Hand-v3") /* TODO: Hand::name() */)
        {
            Robo::JointInput *p = pinput.get();
            auto *pp = dynamic_cast<Robo::NewHand::Hand::JointInput*>(p);
            ConfigJSON::save(joints, *pp);
        }
        else if (robo_name == _T("Tank-v1") /* TODO: Tank::name() */)
        {
            Robo::JointInput *p = pinput.get();
            auto *pp = dynamic_cast<Robo::Mobile::Tank::JointInput*>(p);
            ConfigJSON::save(joints, *pp);

        }
        else throw std::logic_error("Not implemented");
    }
    robo.add_child(_T("joints"), joints);
    root.add_child(_T("robo"), robo);
}
void ConfigJSON::load(tptree &root, tstring &robo_name, Point& robo_base,
                      std::list<std::shared_ptr<Robo::JointInput>> &robo_joints)
{
    auto &node = root.get_child(_T("robo"));
    //assert(root.size() == 3);
    robo_name = node.get<tstring>(_T("name"));

    ConfigJSON::load(node.get_child(_T("base")), robo_base);
    for (tptree::value_type &v : node.get_child(_T("joints")))
    {
        std::shared_ptr<Robo::JointInput> pinput;
        if (robo_name == _T("Hand-v3") /* TODO: Hand::name() */)
        {
            pinput = std::make_shared<Robo::NewHand::Hand::JointInput>();
            ConfigJSON::load(v.second, *dynamic_cast<Robo::NewHand::Hand::JointInput*>(pinput.get()));
        }
        else if (robo_name == _T("Tank-v1") /* TODO: Tank::name() */)
        {
            pinput = std::make_shared<Robo::Mobile::Tank::JointInput>();
            ConfigJSON::load(v.second, *dynamic_cast<Robo::Mobile::Tank::JointInput*>(pinput.get()));
        }
        else throw std::logic_error("Not implemented");
        robo_joints.push_back(pinput);
    }
}
//--------------------------------------------------------------------------------