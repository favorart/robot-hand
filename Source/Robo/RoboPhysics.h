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
    /*virtual*/ void updateStepSignal(muscle_t, frames_t);
    /*virtual*/ void updateStep();

protected:
    /*virtual*/ distance_t jointShift(joint_t) const;

    virtual void realMove() = 0;
    virtual int specPoint() const = 0;
    virtual distance_t prismaticFactor(joint_t) const = 0;

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

    frames_t move(IN const Control&, IN frames_t max_frames) override;
    frames_t move(IN const vBitwise&, IN frames_t max_frames) override;
    frames_t move(IN const bitset_t&, IN frames_t lasts, IN frames_t max_frames) override;
    //frames_t move(IN const BitsControl<musclesMaxCount + 1>&, IN frames_t max_frames) override;
    //frames_t move(IN const std::vector<muscle_t>&, IN frames_t max_frames); override;
    frames_t move(IN muscle_t, IN frames_t, IN frames_t max_frames) override;
    frames_t move(IN frames_t max_frames) override;

    void step() override;
    void step(IN const Control &control, OUT size_t &control_curr) override;
    void step(bitset_t muscles, frames_t lasts) override;
    void step(const RoboI::bitwise &muscles) override;

    void reset() override;

#ifdef DEBUG_RM
    virtual frames_t muscleStatus(muscle_t m) const;
    virtual frames_t lastsStatus(muscle_t m) const;
    virtual TCHAR lastsStatusT(muscle_t m) const;
#endif // DEBUG_RM

    virtual void resetJoint(joint_t);
    virtual StateTrajectory& traj() { return _trajectory; };

    rl_problem::ObservationRobo getCurrState() const override;
    State currState() const override;
    bool isCollision() const override;
    ENV  getEnvCond() const override;
    bool envi(ENV) const override;
    void setEnvCond(ENV) override;
    bool moveEnd() const override;
    const Point& position() const override;
    const Point& jointPos(joint_t) const override;

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

