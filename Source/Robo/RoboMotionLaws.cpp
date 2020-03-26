#include "RoboInputs.h"
#include "RoboMotionLaws.h"
#include "RoboMotionLawsTypes.h"


using namespace std;
using namespace Robo;
using namespace MotionLaws;
//--------------------------------------------------------------------------------
// JointMoveLawI
//--------------------------------------------------------------------------------
const double JointMoveLawI::Epsilont = Utils::EPSILONT;

//--------------------------------------------------------------------------------
void JointMoveLawI::normalize(IterVecDoubles iter, size_t n, double left, double right, double summary) const
{
    for (size_t i = 0; i < n; ++i)
    {
        *iter = *iter * (right - (n + 1) * left) / summary + left;
        ++iter;
    }
}

//------------------------------------------------------------------------------
// MotionLaws
//------------------------------------------------------------------------------
tstring Robo::MotionLaws::name(MLaw ml)
{
    if (ml == MLaw::INVALID || ml >= MLaw::_COUNT_)
        throw std::logic_error{ "Invalid MotionLaw" };
    const tstring data[] = { _T(""), _T("SLOW"), _T("FAST"), _T("STAB"), _T("CONAC"), _T("MANGO") };
    return data[static_cast<int>(ml)];
}

//--------------------------------------------------------------------------------
MLaw Robo::MotionLaws::scanMLaw(const tstring &ml_name)
{
    for (int ml = 1; ml < static_cast<int>(MLaw::_COUNT_); ++ml)
        if (ml_name == name(MLaw(ml)))
            return MLaw(ml);
    CERROR("Invalid scan MotionLaw");
    return MLaw::INVALID;
}


//--------------------------------------------------------------------------------
// JointMotionLaw
//--------------------------------------------------------------------------------
Robo::MotionLaws::JointMotionLaw::JointMotionLaw(MLaw ml, size_t nMoveFrames, double dMoveDistance,
                                                 double dInertiaRatio, tstring param) :
    type(ml), nMoveFrames(nMoveFrames), dMoveDistance(dMoveDistance), 
    dInertiaRatio(dInertiaRatio), param(param)
{ init(); }

//--------------------------------------------------------------------------------
void Robo::MotionLaws::JointMotionLaw::init()
{
    switch (type)
    {
    case MLaw::SLOW:
    {
        moveLaw = make_shared<ContinuousSlowAcceleration>();
        stopLaw = make_shared<ContinuousDeceleration>();
        break;
    }
    case MLaw::FAST:
    {
        moveLaw = make_shared<ContinuousFastAcceleration>();
        stopLaw = make_shared<ContinuousDeceleration>();
        break;
    }
    case MLaw::STAB:
    {
        dStableRatio = std::stod(param, nullptr);
        dStableRatio = std::max(dStableRatio, 0.10);
        dStableRatio = std::min(dStableRatio, 0.99);
        moveLaw = make_shared<ContinuousAccelerationThenStabilization>(dStableRatio);
        stopLaw = make_shared<ContinuousDeceleration>();
        break;
    }
    case MLaw::CONAC:
    {
        moveLaw = make_shared<ContinuousAcceleration>();
        stopLaw = make_shared<ContinuousDeceleration>();
        break;
    }
    case MLaw::MANGO:
    {
        tstring mangoMove = _T("Resource/Hand/") + param + _T("MoveFrames.txt");
        tstring mangoStop = _T("Resource/Hand/") + param + _T("StopFrames.txt");
        moveLaw = make_shared<MangoAcceleration>(mangoMove);
        stopLaw = make_shared<MangoDeceleration>(mangoStop);
        break;
    }
    default:
        throw std::logic_error{ "Invalid MotionLaw" };
    }
}

//--------------------------------------------------------------------------------
bool Robo::MotionLaws::JointMotionLaw::operator==(const JointMotionLaw &ml) const
{
    if (this == &ml)
        return true;
    return (type == ml.type &&
            nMoveFrames == ml.nMoveFrames &&
            dMoveDistance == ml.dMoveDistance &&
            dInertiaRatio == ml.dInertiaRatio &&
            dStableRatio == ml.dStableRatio /*&&
            param == ml.param*/);
}

//--------------------------------------------------------------------------------
void Robo::MotionLaws::JointMotionLaw::save(tptree &ml) const
{
    ml.put(_T("name"), MotionLaws::name(type));
    ml.put(_T("type"), static_cast<uint8_t>(type));
    ml.put(_T("param"), param);
    ml.put(_T("nMoveFrames"), nMoveFrames);
    ml.put(_T("dMoveDistance"), dMoveDistance);
    ml.put(_T("dInertiaRatio"), dInertiaRatio);
    ml.put(_T("dStableRatio"), dStableRatio);
}

//--------------------------------------------------------------------------------
void Robo::MotionLaws::JointMotionLaw::load(tptree &ml)
{
    //typeName = ml.get<tstring>(_T("name"));
    type = static_cast<MLaw>(ml.get<uint8_t>(_T("type")));
    param = ml.get<tstring>(_T("param"));
    nMoveFrames = ml.get<size_t>(_T("nMoveFrames"));
    dMoveDistance = ml.get<double>(_T("dMoveDistance"));
    dInertiaRatio = ml.get<double>(_T("dInertiaRatio"));
    dStableRatio = ml.get<double>(_T("dStableRatio"));
    init();
}

//--------------------------------------------------------------------------------
tostream& Robo::MotionLaws::operator<<(tostream &s, const JointMotionLaw &law)
{
    return s << "{ " << MotionLaws::name(law.type)
      //<< " " << law.type
      //<< " " << law.param
      << " " << law.nMoveFrames
      << " " << law.dMoveDistance
      << " " << law.dInertiaRatio
      << " " << law.dStableRatio << " }";
}

//--------------------------------------------------------------------------------
std::ostream& Robo::MotionLaws::operator<<(std::ostream &s, const JointMotionLaw &law)
{
    return s << "{ " << Utils::ununi(MotionLaws::name(law.type))
      //<< " " << law.type
      //<< " " << law.param
      << " " << law.nMoveFrames
      << " " << law.dMoveDistance
      << " " << law.dInertiaRatio
      << " " << law.dStableRatio << " }";
}


//--------------------------------------------------------------------------------
// JointInput
//--------------------------------------------------------------------------------
void Robo::JointInput::save(tptree &root) const
{
    root.put(_T("joint"), joint);
    root.put(_T("show"), show);

    tptree pbase;
    base.save(pbase);
    root.add_child(_T("base"), pbase);

    tptree ml;
    frames.save(ml);
    root.add_child(_T("motionLaw"), ml);
}

//--------------------------------------------------------------------------------
void Robo::JointInput::load(tptree &root)
{
    joint = root.get<joint_t>(_T("joint"));
    show = root.get<bool>(_T("show"));
    base.load(root.get_child(_T("base")));
    frames.load(root.get_child(_T("motionLaw")));
}

