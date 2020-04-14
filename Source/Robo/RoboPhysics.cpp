#include "RoboPhysics.h"
#include "RoboEdges.h"
#include "RoboPhysicsStatus.h"
#include "RoboEnviroment.h"
#include "RoboMuscles.h"

using namespace Robo;
//--------------------------------------------------------------------------------
tostream& Robo::operator<<(tostream &s, const State &state)
//tostream& Robo::State::operator<<(tostream &s) const
{
    auto vec_print = [&s](const tstring name, const auto &vec) {
        s << name << _T("=[ ");
        for (const auto &el : vec)
            s << el << _T(", ");
        s << _T("] ");
    };
    s << _T("s[") << state.special_no << _T("]={ ");
    vec_print(_T("pos"), state.positions);
    vec_print(_T("vel"), state.velosities);
    vec_print(_T("acc"), state.accelerations);
    s << _T("}");
    return s;
}
//--------------------------------------------------------------------------------
bool RoboI::operator==(const RoboI &r) const
{
    if (this == &r)
        return true;
    if (getName() == r.getName() && _base == r._base)
    {
        auto it = _joint_inputs.begin();
        auto jt = r._joint_inputs.begin();
        while (it != _joint_inputs.end() && jt != r._joint_inputs.end())
        {
            if (!(**it).show) { ++it; continue; }
            if (!(**jt).show) { ++jt; continue; }
            if (it == _joint_inputs.end() || jt == r._joint_inputs.end()
                || (**it).joint != (**jt).joint
                || (**it).base != (**jt).base || (**it).frames != (**jt).frames)
                return false;
            ++it;
            ++jt;
        }
    }
    return true;
}
void RoboI::save(tptree &root) const
{
    tptree pbase;
    _base.save(pbase);
    tptree joints;
    for (const auto &pJI : _joint_inputs)
    {
        //auto &ji = dynamic_cast<Robot::JointInput&>(*pJI.get());
        //ji.save(joints);
        tptree node;
        pJI->save(node);
        joints.push_back(std::make_pair(_T(""), node));
    }
    tptree robo;
    robo.put(_T("enviroment"), putEnviroment(getEnvCond()));
    robo.put(_T("type"), getName());
    robo.add_child(_T("base"), pbase);
    robo.add_child(_T("joints"), joints);
    root.add_child(_T("robo"), robo);
}
//--------------------------------------------------------------------------------
void RoboPhysics::Status::musclesAllDrivesStop()
{
#define NO_ENVI
#ifndef NO_ENVI // !!!!!!!!!!!!! ENVIROMTN
    for (muscle_t m = 0; m < musclesCount; ++m)
        muscleDriveStop(m);
    _moveEnd = true;
#endif
}
void RoboPhysics::Status::muscleDriveStop(muscle_t muscle)
{
    musclesMove[muscle] = 0;
    prevFrame[muscle] = 0;
    shifts[muscle] = 0;
    lastsMove[muscle] = 0;
    lastsStop[muscle] = 0;
    lasts[muscle] = 0;
}
//--------------------------------------------------------------------------------
bool RoboPhysics::Status::muscleDriveFrame(muscle_t muscle, RoboPhysics &self)
{
    const joint_t joint = self.jointByMuscle(muscle);
    distance_t Frame = 0.;
    //------------------------------------------------
    if (movingOn(muscle))
    {
        auto last = lastsMove[muscle] - 1;
        const auto &frames = self.env->framesMove[joint];
        // TODO: restore after MUTUAL_BLOCKING
        do
        {
            //if (last >= frames.size()) throw std::runtime_error("DriveMove: Invalid lasts " + std::to_string(joint));
            Frame = (last >= frames.size()) ? frames.back() : frames[last];
        } while (prevFrame[muscle] > Frame && ++last < frames.size());
    }
    else if (movingOff(muscle))
    {
        auto last = lastsStop[muscle] - 1;
        const auto &frames = self.env->framesStop[joint];
        do
        {
            //if (last >= frames.size()) throw std::runtime_error("DriveStop: Invalid lasts " + std::to_string(joint));
            Frame = (last >= frames.size()) ? frames.back() : frames[last];
        } while (prevFrame[muscle] < Frame && ++last < frames.size());

    }
    else throw std::logic_error("!lastsMove & !lastsStop");
    //------------------------------------------------
    prevFrame[muscle] = Frame;
    // -------------------------------------------
    bool result = self.updateEnvChanges(muscle, joint, Frame);
    // -------------------------------------------
    shifts[muscle] = Frame;
    // -------------------------------------------
    if (std::isnan(shifts[muscle]) || std::isinf(shifts[muscle]))
        CERROR("shift NAN");
    // -------------------------------------------
    return result;
}
bool RoboPhysics::updateEnvChanges(muscle_t muscle, joint_t joint, distance_t &Frame)
{
    const frames_t lastsMove = status->movingOnFrame(muscle);
    // --- WIND ----------------------------------
    if (containE(getEnvCond(), ENV::WINDY) && lastsMove == 1)
    {
        Frame = Utils::random(RoboI::minFrameMove, env->max_frames[joint]);
        CDEBUG(" wind[" << muscle << "]: " << Frame);
    }

    // --- Start FRICTION ------------------------
    else if (containE(getEnvCond(), ENV::START_FRICTION)
             && lastsMove > 0 && lastsMove <= env->st_friction_n_frames)
    {
        Frame = ((lastsMove == env->st_friction_n_frames) ? env->st_friction_big_frame[joint] : 0.);
        CDEBUG(" start_friction[" << muscle << "]" << lastsMove << ": " << Frame);
    }

    // --- MOMENTUM CHANGES ----------------------
    else if (containE(getEnvCond(), ENV::MOMENTUM_CHANGES)
             && env->momentum_happened < env->momentum_max_happens 
             && Utils::random(100) < env->momentum_expect_happens)
    {
        auto oldFrame = Frame;
        Frame = Utils::random(RoboI::minFrameMove, env->max_frames[joint]);
        ++env->momentum_happened;
        CDEBUG(" momentum[" << muscle << "]" << env->momentum_happened << ": " << Frame << " (" << oldFrame << ")");
    }

    // --- SYSTEMATIC CHANGES --------------------
    else if (containE(getEnvCond(), ENV::SYSTEMATIC_CHANGES)
             && (status->movingOn(muscle) || status->movingOff(muscle)))
    {
        auto oldFrame = Frame;
        Frame = Frame + env->systematic_factor;
        Frame = std::min(Frame, env->max_frames[joint]);
        Frame = std::max(Frame, 0.);
        CDEBUG(" systematic[" << muscle << "]" << lastsMove << ": " << Frame << " (" << oldFrame << "|f=" << env->systematic_factor << ")");
    }

    // ---- MUTIAL_BLOCKING ----------------------
    bool result = (fabs(Frame) >= RoboI::minFrameMove);
    if (!result && (containE(getEnvCond(), ENV::MUTIAL_BLOCKING) || containE(getEnvCond(), ENV::START_FRICTION)) && status->movingOn(muscle))
    {
        result = true; /* если блокировка противоположным мускулом, продолжаем движение */
        CDEBUG(" mutial_block[" << muscle << "]: " << Frame);
    }
    return result;
}
void RoboPhysics::Status::muscleDriveMove(frames_t frame, muscle_t muscle, frames_t lasts_signal, RoboPhysics &self)
{
    /* если не производится никакого движения и нет сигнала о начале нового */
    if (!lasts_signal && !lastsMove[muscle] && !lastsStop[muscle])
        return;
    //-------------------------------------------------------
    _moveEnd = false;
    //-------------------------------------------------------
    if (lasts_signal > 0)
    {
        if (lastsMove[muscle] == 0)
        {
            /* движение не было - начало нового движения */
            lastsStop[muscle] = 0;
            lastsMove[muscle] = 1;
            lasts[muscle] = lasts_signal;

            if (!lastsStop[muscle])
                musclesMove[muscle] = frame;
        }
        else
        {
            /* движение было - остановка по сигналу */
            lastsStop[muscle] = 1;
            lastsMove[muscle] = 0;
            lasts[muscle] = 0;
        }
    }
    //-------------------------------------------------------
    if (lastsMove[muscle] > 0)
    {
        bool muscleRelocation = muscleDriveFrame(muscle, self);
        if (lasts[muscle] < lastsMove[muscle]
            || !muscleRelocation /* или, если пустой кадр - остался на месте */)
        {
            /* остановка основного движения по истечении заявленной длительности */
            lastsStop[muscle] = 1;
            lastsMove[muscle] = 0;
            lasts[muscle] = 0;
        }
        else
        {
            /* Time is moving forward! */
            ++lastsMove[muscle];
            ++musclesMove[muscle];
        }
    }
    //-------------------------------------------------------
    if (lastsStop[muscle] > 0)
    {
        /* движение по инерции */
        if (!muscleDriveFrame(muscle, self))
        {
            muscleDriveStop(muscle);
            /* проверяем, что остальные двигатели уже остановились - полная остановка */
            if (!somethingMoving())
                _moveEnd = true;
        }
        else
        {
            /* Time is moving forward! */
            ++lastsStop[muscle];
            ++musclesMove[muscle];
        }
    }
}
bool RoboPhysics::Status::muscleMoveIsFrame(muscle_t m, frames_t _frame) const
{
    if (musclesMove[m] != (0) &&
        musclesMove[m] != (_frame) &&
        musclesMove[m] != (_frame + 1))
        throw std::runtime_error("Controls invalid!");
    return (musclesMove[m] == _frame);
}
//--------------------------------------------------------------------------------
void RoboPhysics::step(muscle_t m, frames_t lasts)
{
    if (status->muscleMoveIsFrame(m, _frame) || lasts > 0)
        status->muscleDriveMove(_frame, m, lasts, *this);
    // !!! NO for, thus NO realMove - need updateStep()!
}
void RoboPhysics::step(const bitwise &muscles)
{
    status->step(muscles, *this);
    updateStep();
}
//--------------------------------------------------------------------------------
void RoboPhysics::Status::step(const bitwise &muscles, RoboPhysics &self)
{
    //if (!muscles.any() /*|| muscles.to_ullong() >= frames_t(1ULL << musclesCount())*/)
    //    throw std::runtime_error("Controls invalid!");
    if (!muscles.any() && !somethingMoving())
    {
#ifndef NO_ENVI // !!!!!!!!!!!!! ENVIROMTN
        _moveEnd = true;
#endif
        return;
    }
    for (muscle_t m = 0; m < musclesCount; ++m)
    {
        if (musclesMove[m] > 0) // если уже движется
        {
            //if (status->musclesMove[m] < _frame || status->lasts[m] != (status->lastsMove[m] - 1))
            //    throw std::runtime_error("Controls invalid frame!");
            if (muscles[m]) // продолжаем активность
                ++lasts[m];
        }
        else if (muscles[m]) // начинаем движение
            muscleDriveMove(self.frame(), m, 1, self);
    }
}
//--------------------------------------------------------------------------------
void RoboPhysics::updateStep()
{
    for (muscle_t m = 0; m < musclesCount(); ++m)
        step(m, 0);
    // =================
    bool notMove = realMove(); // update() needs !!! FOR REAL MOVE !!!
    if (notMove)
        status->musclesAllDrivesStop();
    status->shifts.fill(0);
    // =================
    if (_trajectory_save /*&& !(_frame % getVisitedRarity()) && (_frame / getVisitedRarity()) >= _trajectory.size()*/)
    {
        State state;
        status->getCurState(state, specPoint());
        _trajectory.push_back(std::move(state));
    }
    status->updateState();
    // =================
    ++_frame;
}
void RoboPhysics::step(IN const bitset_t &muscles, IN frames_t lasts)
{
    for (muscle_t m = 0; m < musclesCount(); ++m)
        if (muscles[m])
            step(m, lasts);
    updateStep();
}
void RoboPhysics::step(IN const Control &control)
{
    control.validated(musclesCount());
    frames_t start = control[0].start /*, lasts = control[0].lasts*/;
    for (auto &c : control)
    {
        if (/*lasts != c.lasts ||*/ start != c.start)
            throw std::runtime_error("Controls invalid!");
        step(c.muscle, c.lasts);
    }
    updateStep();
}
void RoboPhysics::step(IN const Control &control, OUT size_t &control_curr)
{
    control.validated(musclesCount());
    for (size_t i = control_curr; i < control.size(); ++i)
    {
        if (control[i].start < _frame)
            throw std::runtime_error("Controls invalid!");
        if (control[i].start > _frame)
            break;
        step(control[i].muscle, control[i].lasts);
        control_curr = i + 1;
    }
    updateStep();
}

//--------------------------------------------------------------------------------
frames_t RoboPhysics::move(IN const std::vector<muscle_t> &controls, IN frames_t max_frames)
{
    if (controls.size() > RoboPhysics::LastsTooLong)
        CERROR(" move lasts too long ");
    for (frames_t i = 0; (i < controls.size() || !moveEnd()) /*&& _frame < max_frames*/; ++i)
    {
        step({ controls[i] });
        boost::this_thread::interruption_point();
    }
    return move(max_frames);
}
//--------------------------------------------------------------------------------
frames_t RoboPhysics::move(IN frames_t max_frames)
{
    if (!status->somethingMoving()) // Исключить незадействованные двигатели
    {
        CDEBUG("Nothing moving");
        return _frame;
    }
    while (_frame < max_frames && !moveEnd()) // just moving
    {
        if (frame() > RoboPhysics::LastsTooLong * 2)
            CERROR(" move lasts too long ");
        if (!status->somethingMoving())
            return _frame;
        updateStep();
        boost::this_thread::interruption_point();
    }
    if (_trajectory_save)
    {
        State state;
        status->getCurState(state, specPoint());
        _trajectory.push_back(std::move(state)); // put last
    }
    return _frame;
}
//--------------------------------------------------------------------------------
frames_t RoboPhysics::move(IN const bitset_t &muscles, IN frames_t lasts, IN frames_t max_frames)
{
    /* Что-то должно двигаться, иначе беск.цикл */
    if (!muscles.any() || muscles.to_ullong() >= frames_t(1ULL << musclesCount()) || lasts == 0)
        throw std::runtime_error("Controls invalid!");
    step(muscles, lasts);
    return move(max_frames);
}
//--------------------------------------------------------------------------------
frames_t RoboPhysics::move(IN const Control &controls, IN frames_t max_frames)
{
    controls.validated(musclesCount());
    CDEBUG("C:" << controls);
    for (auto &a : controls) // starts all moves
    {
        if (a.lasts > RoboPhysics::LastsTooLong ||
            a.start > RoboPhysics::LastsTooLong ||
            frame() > RoboPhysics::LastsTooLong * 2)
            CERROR(" move lasts too long ");
        //CDEBUG("c:" << c);
        while (a.start > _frame)
            updateStep();
        step(a.muscle, a.lasts);
    }
    return move(max_frames);
}
//--------------------------------------------------------------------------------
frames_t RoboPhysics::move(IN const BitsControl<musclesMaxCount + 1> &controls, IN frames_t max_frames)
{
    for (const auto &bits : controls)
    {
        step(bits);
        boost::this_thread::interruption_point();
    }
    return move(max_frames);
}
//--------------------------------------------------------------------------------
void RoboPhysics::resetJoint(joint_t joint)
{
    status->muscleDriveStop(muscleByJoint(joint, true));
    status->muscleDriveStop(muscleByJoint(joint, false));
}
void RoboPhysics::reset()
{
    _reset();
    status->musclesAllDrivesStop();
    for (joint_t j = 0; j < jointsCount(); ++j)
        resetJoint(j);
    status->reset();
    env->reset();
}
//--------------------------------------------------------------------------------
RoboPhysics::RoboPhysics(const Point &base, const JointsInputsPtrs &joint_inputs, pEnvEdges edges) :
    RoboI(base, joint_inputs),
    status(std::make_shared<Status>(Status::prepareBasePoses(base, joint_inputs))),
    env(std::make_shared<EnvPhyState>(/*base,*/ joint_inputs, edges))
{
    if (!ba::is_sorted(joint_inputs, [](const auto &a, const auto &b) { return (*a < *b); }))
        throw std::runtime_error("Joint Inputs are not sorted!");
}
//--------------------------------------------------------------------------------
RoboPhysics::EnvPhyState::EnvPhyState(/*const Point &base,*/ const JointsInputsPtrs &joint_inputs, pEnvEdges edges) :
    edges(edges)
{
    joint_t j = 0;
    for (const auto &j_in : joint_inputs)
    {
        if (!j_in->show)
            continue;

        laws[j] = j_in->frames;
        
        auto nMoveFrames = laws[j].nMoveFrames;
        auto nStopFrames = static_cast<frames_t>(nMoveFrames * laws[j].dInertiaRatio) + 2;
        auto dMoveDistance = laws[j].dMoveDistance;

        assert(RoboI::minFrameMove < (dMoveDistance * laws[j].dInertiaRatio));
        assert(RoboI::minFrameMove < (dMoveDistance));
        assert(nMoveFrames > 1);
        assert(nStopFrames > 1);
        
        framesMove[j].resize(nMoveFrames);
        framesStop[j].resize(nStopFrames);

        laws[j].moveLaw->generate(framesMove[j].begin(), nMoveFrames,
                                  RoboI::minFrameMove, dMoveDistance);

        distance_t maxVelosity = *boost::max_element(framesMove[j]);
        laws[j].stopLaw->generate(framesStop[j].begin(), nStopFrames - 1,
                                  RoboI::minFrameMove, (dMoveDistance * laws[j].dInertiaRatio),
                                  maxVelosity);
        /* last frame must be 0 to dead-end */
        framesStop[j][nStopFrames - 1] = 0.;
                
        max_frames[j] = maxVelosity; /* max frame */

        st_friction_n_frames = 31;
        st_friction_big_frame[j] = (maxVelosity / 3.);

        assert(st_friction_n_frames < nMoveFrames);
        assert(st_friction_big_frame[j] < maxVelosity);

        systematic_changes = std::make_shared<SystematicChanges<>>(112, -(maxVelosity / 100.), (maxVelosity / 100.));

        printFrames(j);
        ++j;
    }
}
void RoboPhysics::EnvPhyState::printFrames(joint_t j) const
{
#ifdef DEBUG_PLOT_PHY_STATE
    printEnumOneHot<Enviroment>(conditions, Robo::enviroment_outputs);
    std::cout << std::endl;

    std::stringstream ss;
    ss << "motion-law-" << Utils::ununi(Robo::MotionLaws::name(laws[j].type)) << "-" << int(j) << ".plt";
    std::cout << ss.str() << std::endl;
    // jointName(J(j))
    std::ofstream fout(ss.str());

    fout << "plot '-' using 1:2 with lines" << std::endl;
    unsigned count = 0;
    double sum = 0.;
    auto fprinter = [&fout, &count, &sum](double item) {
        fout << count++ << '\t' << item << std::endl;
        sum += item;
    };

    std::for_each(framesMove[j].begin(), framesMove[j].end(), fprinter);
    std::cout << "framesMove[" << int(j) << "]=" << framesMove[j].size() << " sum=" << sum << std::endl;

    sum = 0.;
    std::for_each(framesStop[j].begin(), framesStop[j].end(), fprinter);
    std::cout << "framesStop[" << int(j) << "]=" << framesStop[j].size() << " sum=" << sum << std::endl;
    //std::system("\"C:\\Program Files (x86)\\gnuplot\\bin\\gnuplot\" \"motion-law.dat\" -presist");
#endif // DEBUG_PLOT_PHY_STATE
}
void RoboPhysics::EnvPhyState::reset()
{
    momentum_happened = 0;
    systematic_factor *= systematic_changes->get();
}
//--------------------------------------------------------------------------------
Enviroment RoboPhysics::getEnvCond() const { return env->conditions; }
void RoboPhysics::setEnvCond(Enviroment conditions) { env->conditions = conditions; }
//--------------------------------------------------------------------------------
RoboPhysics::Status::Status(PreparedBasePoints bases) :
    jointsCount(joint_t(bases.size() - 1)),
    musclesCount(RoboI::musclesPerJoint * jointsCount),
    basePos(bases.begin(), bases.end()),
    currPos(bases.begin(), bases.end())
{
    if (!jointsCount)
        throw std::runtime_error("No active joints!");
    reset();
}
//--------------------------------------------------------------------------------
auto RoboPhysics::Status::prepareBasePoses(const Point &base, const JointsInputsPtrs &joint_inputs) -> PreparedBasePoints
{
    PreparedBasePoints bases;
    for (auto &j_in : joint_inputs)
        if (j_in->show)
            bases.push_back(j_in->base);
    bases.push_back(base);
    return bases;
}
//--------------------------------------------------------------------------------
void RoboPhysics::Status::reset()
{
    //musclesAllDrivesStop();
    _moveEnd = false;
    //auto it = basePos.cbegin();
    for (joint_t joint = 0; joint < jointsCount; ++joint)//, ++it)
    {
        prevPos[joint] = currPos[joint]/* = *it*/; // позиция сочленений текущая будет в предыдущей в след.такт
        prevVel[joint] = { 0., 0. }; // скорость сочленения в данный такт
        //acceleration[joint] = Point{}; // ускорение сочленения в данный такт
        shifts[joint] = 0.;
    }
}
void RoboPhysics::Status::updateState()
{
    for (joint_t joint = 0; joint < jointsCount; ++joint)
    {
        prevVel[joint] = (curPos(joint) - prevPos[joint]);
        prevPos[joint] = curPos(joint);
    }
}
void RoboPhysics::Status::getCurState(State &state, int spec) const
{
    state.special_no = spec;
    state.accelerations.resize(jointsCount);
    state.velosities.resize(jointsCount);
    state.positions.resize(jointsCount);
    for (joint_t joint = 0; joint < jointsCount; ++joint)
    {
        state.positions[joint] = curPos(joint);
        state.velosities[joint] = curPos(joint) - prevPos[joint]; // momentum velosity
        state.accelerations[joint] = (state.velosities[joint] - prevVel[joint]); // momentum acceleration
    }
}
//--------------------------------------------------------------------------------
bool RoboPhysics::isCollision() const { return env->edges->collision; }
//--------------------------------------------------------------------------------
muscle_t RoboPhysics::musclesCount() const { return status->musclesCount; }
joint_t RoboPhysics::jointsCount() const { return status->jointsCount; }
//--------------------------------------------------------------------------------
bool RoboPhysics::moveEnd() const { return status->moveEnd(); }
const Point& RoboPhysics::position() const { return status->curPos(specPoint()); }
const Point& RoboPhysics::jointPos(IN joint_t joint) const
{
    if (joint >= jointsCount())
        throw std::logic_error("jointPos: Incorrect joint");
    return status->curPos(joint);
}

const Point& RoboPhysics::basePos(joint_t j) const  { return status->basePos [(j > jointsCount()) ? jointsCount() : j]; }
const Point& RoboPhysics::curPos(joint_t j) const   { return status->curPos  ((j > jointsCount()) ? jointsCount() : j); }
Point& RoboPhysics::currPos(joint_t j)              { return status->currPos [(j > jointsCount()) ? jointsCount() : j]; }
//--------------------------------------------------------------------------------
distance_t RoboPhysics::jointShift(joint_t joint) const
{
    const auto mo = muscleByJoint(joint, true);
    const auto mc = muscleByJoint(joint, false);

    auto shift = (status->shifts[mo] - status->shifts[mc]);
    if (shift)
    {
        auto moment = Imoment(joint);
        auto interact = env->edges->interaction(joint);

        CINFO(" muscleMove[" << mo << "]=" << status->movingOnFrame(mo) << " muscleMove[" << mc << "]=" << status->movingOnFrame(mc));
        //CINFO("  lastsMove[" << mo << "]=" << status->lastsMove[mo]   << "  lastsMove[" << mc << "]=" << status->lastsMove[mc]);
        CINFO(" jointShift[" << joint << "]" << shift << " Imoment " << moment << " interact " << interact << " =" << (shift - moment) * interact << std::endl);

        return (shift - moment) * interact;
    }
    return 0.;

    ////return ( -(status->shifts[mc] - status->shifts[mo]) + Imoment(joint) ) * env->edges->interaction(joint);
    //return ((status->shifts[mo] - status->shifts[mc]) - Imoment(joint)) * env->edges->interaction(joint);
    //
    ////(status->shifts[LTrackFrw] - status->shifts[LTrackBck]) - Imoment(RTrack);
    ////(status->shifts[RTrackFrw] - status->shifts[RTrackBck]) - Imoment(LTrack);
}
//--------------------------------------------------------------------------------
distance_t RoboPhysics::Imoment(joint_t j) const
{
    const auto m_o = muscleByJoint(j, true);
    const auto m_c = muscleByJoint(j, false);
    // Добавить моменты инерции, для соседей
    // Когда "сочленение заблокировано"=(1,1) - то перемещается, как "монолит",
    // Иначе на него действуют смещения остальных сочленений
    distance_t Imoment = 0.;
    if (containE(getEnvCond(), ENV::MUTIAL_DYNAMICS) &&
        containE(getEnvCond(), ENV::MUTIAL_BLOCKING) &&
         !(status->shifts[m_c] > RoboI::minFrameMove &&
           fabs(status->shifts[m_c] - status->shifts[m_o]) < RoboI::minFrameMove))
    {
        for (joint_t jj = j, gain = 4; jj > 0; --jj, gain <<= 1)
        {
            const auto mo = muscleByJoint(jj - 1, true);
            const auto mc = muscleByJoint(jj - 1, false);
            Imoment += (status->shifts[mc] - status->shifts[mo]) / gain;
        }
        for (joint_t jj = j + 1, gain = 4; jj < jointsCount(); ++jj, gain <<= 1)
        {
            const auto mo = muscleByJoint(jj, true);
            const auto mc = muscleByJoint(jj, false);
            Imoment += (status->shifts[mc] - status->shifts[mo]) / gain;
        }
    }
    return Imoment;
}
//--------------------------------------------------------------------------------
#include "RoboRLSim.h"
rl_problem::ObservationRobo RoboPhysics::getCurrState() const
{
    rl_problem::ObservationRobo o;
    status->getCurState(o.state, specPoint());
    return o;
}
State RoboPhysics::currState() const
{
    State state;
    status->getCurState(state, specPoint());
    return state;
}

#ifdef DEBUG_RM
frames_t RoboPhysics::Status::muscleStatus(muscle_t m) const { return musclesMove[m]; }
frames_t RoboPhysics::Status::lastsStatus(muscle_t m) const { return std::max(lastsMove[m], lastsStop[m]); }
TCHAR RoboPhysics::Status::lastsStatusT(muscle_t m) const { return (lastsMove[m] > lastsStop[m]) ? ((lastsMove[m] > 0) ? 'm' : '0') : 's'; }

frames_t RoboPhysics::muscleStatus(muscle_t m) const { return status->muscleStatus(m); }
frames_t RoboPhysics::lastsStatus(muscle_t m) const { return status->lastsStatus(m); }
TCHAR RoboPhysics::lastsStatusT(muscle_t m) const { return status->lastsStatusT(m); }
#endif // DEBUG_RM

//--------------------------------------------------------------------------------
void RoboPhysics::plotMotionLaws(const tstring &fn, joint_t joint) const
{
#ifdef DEBUG_PLOTTING
    auto fname = Utils::ununi(fn);
    //ss.str(""); // CLEAR ss
    //std::system(("del " +).c_str());

    std::ofstream fplot(fname + ".plt");
    fplot << "set title 'Test Motion Laws " << Utils::ununi(getName()) << "' " << std::endl;  // plot title
    fplot << "set xlabel 'Time' " << std::endl;                                               // x - axis label
    fplot << "set ylabel 'Distance' " << std::endl;                                           // y - axis label
    fplot << "set grid " << std::endl;
    fplot << "set autoscale " << std::endl;
    fplot << "set size ratio -1 " << std::endl;
    // labels
    //ss << "set label \"boiling point\" at 10, 212 " << std::endl;

    ////# key/legend
    //ss << "set key top right " << std::endl;
    //ss << "set key box " << std::endl;
    //ss << "set key left bottom " << std::endl;
    //ss << "set key bmargin " << std::endl;
    //ss << "set key 0.01, 100 " << std::endl;
    //ss << "set nokey" << std::endl; // no key
    //// arrow
    //ss << "set arrow from 1, 1 to 5, 10" << std::endl;

    //fplot << "set style line 1 lt 2 " << std::endl; // dashtype 2
    for (joint_t j = 0; j < jointsCount(); ++j)
    {
        auto  no = int(j) * 10;
        auto nno = int(jointsCount() - 1 - j) * 10;
        //"set dashtype 10 ( 0,10,30,0) "
        //"set dashtype 11 (10,10,20,0) "
        //"set dashtype 12 (20,10,10,0) "
        //"set dashtype 13 (30,10, 0,0) "
        fplot << "set dashtype 1" << int(j) << " (" << no << ",10," << nno << ",0) " << std::endl;
    }
    if (joint == jointsAll)
    {
        auto &robo = dynamic_cast<const RoboI&>(*this);
        fplot << "plot '" << fname << ".dat' ";
        for (joint_t j = 0; j < jointsCount(); ++j)
        {
            if (j > 0)
                fplot << ", \\" << std::endl << "     '" << fname << ".dat' ";
            fplot << " using 1:" << int(j+2) << " with lines dt 1" << int(j) << " ";
            fplot << " title '" << Utils::ununi(Robo::getJointName(robo, j)) << "' ";
        }
        fplot << std::endl;

        auto max_sz = env->nFramesAll(0);
        for (joint_t j = 1; j < jointsCount(); ++j)
            if (max_sz < env->nFramesAll(j))
                max_sz = env->nFramesAll(j);

        std::ofstream fdat(fname + ".dat");
        for (frames_t x = 0; x < max_sz; ++x)
        {
            fdat << x << '\t';
            for (joint_t j = 0; j < jointsCount(); ++j)
            {
                if (x < env->nFramesMove(j))
                    fdat << env->framesMove[j][x] * this->prismaticFactor(j);
                else if (x < env->nFramesAll(j))
                    fdat << env->framesStop[j][x - env->nFramesMove(j)] * this->prismaticFactor(j);
                else
                    fdat << 0;
                if (j+1 != jointsCount())
                    fdat << '\t';
            }
            fdat << std::endl;
        }
    }
    else // One joint
    {
        fplot << "plot '-'  using 1:2 with lines ";
        auto &robo = dynamic_cast<const RoboI&>(*this);
        fplot << " title '" << Utils::ununi(Robo::getJointName(robo, joint)) << "' ";
        fplot << std::endl;
        frames_t x = 0;
        const auto prismatic = this->prismaticFactor(joint);
        auto fprinter = [&fplot, &x, &prismatic](Robo::distance_t item) {
            fplot << x++ << '\t' << item * prismatic << std::endl;
        };
        br::for_each(env->framesMove[joint], fprinter);
        br::for_each(env->framesStop[joint], fprinter);        
    }
#endif // DEBUG_PLOTTING
}

