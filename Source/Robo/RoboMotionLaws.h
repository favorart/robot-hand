#pragma once

namespace Robo {
namespace MotionLaws {
//------------------------------------------------------------------------------
/*  Здесь указан интерфейс, в котором модель робота принимает "закон движения" мускула.
 *  Мускулы (прямой и обратный) одного сочленения получают одинаковую разбивку по кадрам и одинаковые граничные условия.
 */
class JointMoveLawI
{
public:
    static const double Epsilont;
    using IterVecDoubles = std::vector<double>::iterator;
    enum class Law { Move, Stop } law;

    explicit JointMoveLawI(Law law) : law(law) {}
    virtual void generate(IterVecDoubles first, size_t frames_count, double left_border, double right_border, double max_velosity = 1.) const = 0;
protected:
    virtual void normalize(IterVecDoubles iter, size_t n, double left, double right, double summary) const;
};

//------------------------------------------------------------------------------
class ContinuousAcceleration;
class ContinuousDeceleration;

class ContinuousAccelerationThenStabilization;
class ContinuousSlowAcceleration;
class ContinuousFastAcceleration;

class MangoAcceleration;
class MangoDeceleration;

class PhisicalAcceleration;
class PhisicalDeceleration;

//------------------------------------------------------------------------------
enum class MLaw : uint8_t { INVALID, SLOW, FAST, STAB, CONAC, MANGO, _COUNT_ };

tstring name(MLaw ml);
MLaw scanMLaw(const tstring &ml_name);

//------------------------------------------------------------------------------
struct JointMotionLaw
{
    std::shared_ptr<JointMoveLawI> moveLaw{};
    std::shared_ptr<JointMoveLawI> stopLaw{};

    MLaw type{ MLaw::INVALID };
    size_t nMoveFrames{};
    double dMoveDistance{};
    double dInertiaRatio{ 0.35 }; // 35% от общего пробега
    double dStableRatio{ 0.55 }; // 55% от общего пробега
    tstring param{};

    JointMotionLaw() {}
    JointMotionLaw(MLaw ml, size_t nMoveFrames, double dMoveDistance, 
                   double dInertiaRatio, tstring param = _T(""));
    void init();
    void save(tptree &root) const;
    void load(tptree &root);
    friend tostream& operator<<(tostream&, const JointMotionLaw&);
    friend std::ostream& operator<<(std::ostream&, const JointMotionLaw&);
    bool operator==(const JointMotionLaw &ml) const;
    bool operator!=(const JointMotionLaw &ml) const { return !(*this == ml); }
};

} // MotionLaws
} // Robo

