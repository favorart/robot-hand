#pragma once
#include "Robo.h"

namespace Robo
{
class EnvEdges;
class EnvEdgesHand;
using pEnvEdges = std::shared_ptr<EnvEdges>;

class RoboPhysics : public RoboI
{
private:
    struct Status;
    struct EnvPhyState;
    using pEnvPhyState = std::shared_ptr<RoboPhysics::EnvPhyState>;
    using pStatus = std::shared_ptr<RoboPhysics::Status>;

    pStatus status;
    pEnvPhyState env;

    /*virtual*/ distance_t Imoment(joint_t) const;
    /*virtual*/ bool updateEnvChanges(muscle_t, joint_t, distance_t &Frame);
    /*virtual*/ void step(muscle_t, frames_t);

protected:
    /*virtual*/ distance_t jointShift(joint_t) const;

    virtual bool realMove() = 0;
    virtual int specPoint() const = 0;
    virtual distance_t prismaticFactor(joint_t) const = 0;
    virtual void updateStep();

    const Point& basePos(joint_t) const;
    const Point& curPos(joint_t) const;
    Point& currPos(joint_t);

public:
    static const frames_t LastsTooLong = 10000;
    RoboPhysics(const Point &base /*Clavicle|Center*/,
                const JointsInputsPtrs &joint_inputs,
                pEnvEdges eiges);

    muscle_t musclesCount() const override;
    joint_t jointsCount() const override;

    frames_t move(IN const Control &controls, IN frames_t max_frames) override;
    frames_t move(IN const bitset_t &muscles, IN frames_t lasts, IN frames_t max_frames) override;
    frames_t move(IN frames_t max_frames) override;
    frames_t move(IN const std::vector<muscle_t>&, IN frames_t max_frames) override;
    frames_t move(IN const BitsControl<musclesMaxCount + 1> &controls, IN frames_t max_frames) override;

    void step() override { updateStep(); }
    void step(IN const Control &control) override;
    void step(IN const Control &control, OUT size_t &control_curr) override;
    void step(IN const bitset_t &muscles, IN frames_t lasts) override;
    void step(const Robo::RoboI::bitwise &muscles) override;
    void reset() override;

#ifdef DEBUG_RM
    virtual frames_t muscleStatus(muscle_t m) const;
    virtual frames_t lastsStatus(muscle_t m) const;
    virtual TCHAR lastsStatusT(muscle_t m) const;
#endif // DEBUG_RM

    virtual void resetJoint(joint_t);
    const Point& jointPos(joint_t) const;
    StateTrajectory& traj() { return _trajectory; };

    rl_problem::ObservationRobo getCurrState() const override;
    State currState() const override;
    bool isCollision() const override;
    Enviroment getEnvCond() const override;
    void setEnvCond(Enviroment conditions) override;
    bool moveEnd() const override;
    const Point& position() const override;

    tstring getName() const = 0; // override { return RoboPhysics::name(); };
    static tstring name() { return _T("RoboPhysics"); };

    static const joint_t jointsAll = 0xFF;
    void plotMotionLaws(const tstring &fn, joint_t joint) const;

private:
    static pRoboI make(const tstring &/*type*/, tptree &/*node*/){ throw std::logic_error("Depricated"); };
    friend class Robo::EnvEdges;
    friend class Robo::EnvEdgesHand;
};
}

