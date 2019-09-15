#include "RoboPhysics.h"
#include "HandMotionLaws.h"
#include "RoboEdges.h"
#include "RoboPhysicsStatus.h"

using namespace Robo;
//--------------------------------------------------------------------------------
bool RoboI::operator==(const RoboI &r) const
{
    if (this == &r)
        return true;
    bool res =
        (getName() == r.getName()) &&
        (_base() == r._base()) &&
        (_joint_inputs.size() == r._joint_inputs.size());
    if (res)
    {
        auto it = _joint_inputs.begin();
        auto jt = r._joint_inputs.begin();
        /*&& jt != r._joint_inputs.end()*/
        while (it != _joint_inputs.end())
        {
            if (*it != *jt)
            {
                res = false;
                break;
            }
            ++it;
            ++jt;
        }
    }
    return res;
}
void RoboI::save(tptree &root) const
{
    tptree pbase;
    _base().save(pbase);
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
    robo.put(_T("type"), getName());
    robo.add_child(_T("base"), pbase);
    robo.add_child(_T("joints"), joints);
    root.add_child(_T("robo"), robo);
}
//--------------------------------------------------------------------------------
bool RoboPhysics::somethingMoving() const
{ return !(ba::all_of_equal(status->musclesMove, 0)); }
//--------------------------------------------------------------------------------
void RoboPhysics::muscleDriveStop(muscle_t muscle)
{
    status->musclesMove[muscle] = 0;
    status->prevFrame[muscle] = 0;
    status->shifts[muscle] = 0;

    status->lastsMove[muscle] = 0;
    status->lastsStop[muscle] = 0;
    status->lasts[muscle] = 0;
}
bool RoboPhysics::muscleDriveFrame(muscle_t muscle)
{
    const joint_t joint = jointByMuscle(muscle);
    const auto maxFrame = env->framesMove[joint].back(); //*boost::max_element(env->framesMove[joint]);
    double Frame = 0.;
    //------------------------------------------------
    if (status->lastsMove[muscle] > 0)
    {
        auto last = status->lastsMove[muscle] - 1;
        const auto &frames = env->framesMove[joint];
        //------------------------------------------------
        if (env->conditions & START_FRICTION)
        {
            if (last <= env->st_friction_n_frames)
            {
                Frame = (last == env->st_friction_n_frames) ? RoboI::minFrameMove : maxFrame / 10;
                goto SkipFrameVal;
            }
            last -= env->st_friction_n_frames;
        }
        //------------------------------------------------
        do
        {
            Frame = (last >= frames.size()) ? frames.back() : frames[last];
            //if (last > physics.framesMove[joint].size())
            //    throw std::runtime_error("DriveMove: Invalid lasts " + std::to_string(joint));
        } while (status->prevFrame[muscle] > Frame && ++last < frames.size());
SkipFrameVal:;
    }
    else if (status->lastsStop[muscle] > 0)
    {
        auto last = status->lastsStop[muscle] - 1;
        const auto &frames = env->framesStop[joint];

        do
        {
            Frame = (last >= frames.size()) ? frames.back() : frames[last];
            //if (last > physics.framesStop[joint].size())
            //    throw std::runtime_error("DriveStop: Invalid lasts " + std::to_string(joint));
        } while (status->prevFrame[muscle] < Frame && ++last < frames.size());
    }
    else throw std::logic_error("!lastsMove & !lastsStop");
    //------------------------------------------------
    status->prevFrame[muscle] = Frame;

    // --- WIND ----------------------------------
    if ((env->conditions & WINDY) && status->lastsMove[muscle] == 1)
    {
        Frame = Utils::random(RoboI::minFrameMove, maxFrame);
    }
    // --- MOMENTUM CHANGES ----------------------
    if (env->conditions & MOMENTUM_CHANGES)
    {
        // TODO: several frames <<<
        Frame = Utils::random(RoboI::minFrameMove, maxFrame);
        env->momentum_happened = true; // in this move (episode) only once
    }
    // -------------------------------------------
    status->shifts[muscle] = Frame;

    if (std::isnan(status->shifts[muscle]) || std::isinf(status->shifts[muscle]))
        CERROR("shift NAN");
    // -------------------------------------------
    return (fabs(Frame) >= RoboI::minFrameMove);
}
void RoboPhysics::muscleDriveMove(frames_t frame, muscle_t muscle, frames_t lasts)
{
    /* если не производится никакого движения и нет сигнала о начале нового */
    if (!lasts && !status->lastsMove[muscle] && !status->lastsStop[muscle])
        return;
    //-------------------------------------------------------
    status->moveEnd = false;
    //-------------------------------------------------------
    if (lasts > 0)
    {
        if (status->lastsMove[muscle] == 0)
        {
            /* движение не было - начало нового движения */
            status->lastsStop[muscle] = 0;
            status->lastsMove[muscle] = 1;
            status->lasts[muscle] = lasts;

            if (!status->lastsStop[muscle])
                status->musclesMove[muscle] = frame;
        }
        else
        {
            /* движение было - остановка по сигналу */
            status->lastsStop[muscle] = 1;
            status->lastsMove[muscle] = 0;
            status->lasts[muscle] = 0;
        }
    }
    //-------------------------------------------------------
    if (status->lastsMove[muscle] > 0)
    {
        if (status->lasts[muscle] < status->lastsMove[muscle]
            /* продолжение движения, если остался на месте (блокировка противоположным мускулом) */
            || !muscleDriveFrame(muscle))
        {
            /* остановка основного движения по истечении заявленной длительности */
            status->lastsStop[muscle] = 1;
            status->lastsMove[muscle] = 0;
            status->lasts[muscle] = 0;
        }
        else
        {
            /* Time is moving forward! */
            ++status->lastsMove[muscle];
            ++status->musclesMove[muscle];
        }
    }
    //-------------------------------------------------------
    if (status->lastsStop[muscle] > 0)
    {
        /* движение по инерции */
        if (!muscleDriveFrame(muscle))
        {
            muscleDriveStop(muscle);
            /* проверяем, что остальные двигатели уже остановились - полная остановка */
            if (ba::all_of_equal(status->musclesMove, 0))
                status->moveEnd = true;
        }
        else
        {
            /* Time is moving forward! */
            ++status->lastsStop[muscle];
            ++status->musclesMove[muscle];
        }
    }
}
//--------------------------------------------------------------------------------
void RoboPhysics::step(const bitwise &muscles)
{
    //if (!muscles.any() /*|| muscles.to_ullong() >= frames_t(1ULL << musclesCount())*/)
    //    throw std::runtime_error("Controls invalid!");
    if (!muscles.any() && !somethingMoving())
    {
        status->moveEnd = true;
        return;
    }
    for (muscle_t m = 0; m < musclesCount(); ++m)
    {
        if (status->musclesMove[m] > 0) // если уже движется
        {
            //if (status->musclesMove[m] < _frame || status->lasts[m] != (status->lastsMove[m] - 1))
            //    throw std::runtime_error("Controls invalid frame!");
            if (muscles[m]) // продолжаем активность
                ++status->lasts[m];
        }
        else if (muscles[m]) // начинаем движение
            muscleDriveMove(_frame, m, 1);
    }
    step();
}

//--------------------------------------------------------------------------------
void RoboPhysics::step(muscle_t m, IN frames_t lasts)
{
    if (status->musclesMove[m] != (0) &&
        status->musclesMove[m] != (_frame) &&
        status->musclesMove[m] != (_frame + 1))
        throw std::runtime_error("Controls invalid!");
    if (lasts > 0 || status->musclesMove[m] == _frame)
        muscleDriveMove(_frame, m, lasts);
    // !!! NO for, thus NO realMove - need step()!
}

//--------------------------------------------------------------------------------
void RoboPhysics::step()
{
    for (muscle_t m = 0; m < musclesCount(); ++m)
        step(m, 0);

    realMove(); // step() needs !!! FOR REAL MOVE !!!
    
    State state;
    status->calcCurState(state, specPoint());
    //auto rarity = getVisitedRarity();
    if (_trajectory_save /*&& !(_frame % rarity) && (_frame / rarity) >= _trajectory.size()*/)
        _trajectory.push_back(std::move(state));
    _frame++;
}
void RoboPhysics::step(IN const bitset_t &muscles, IN frames_t lasts)
{
    for (muscle_t m = 0; m < musclesCount(); ++m)
        if (muscles[m])
            step(m, lasts);
    step();
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
    step();
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
    step();
}

//--------------------------------------------------------------------------------
frames_t RoboPhysics::move(IN const std::vector<muscle_t> &controls, IN frames_t max_frames)
{
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
    if (!somethingMoving()) // Исключить незадействованные двигатели
    {
        CDEBUG("Nothing moving");
        return _frame;
    }
    while (_frame < max_frames && !moveEnd()) // just moving
    {
        if (!somethingMoving())
            return _frame;
        step();
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
    for (auto &c : controls) // starts all moves
    {
        //CDEBUG("c:" << c);
        while (c.start > _frame)
            step();
        step(c.muscle, c.lasts);
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
void RoboPhysics::reset()
{
    _reset();
    for (muscle_t m = 0; m < musclesCount(); ++m)
        muscleDriveStop(m);
    for (joint_t j = 0; j < jointsCount(); ++j)
        resetJoint(j);
    status->moveEnd = false;
    status->reset();
    env->reset();
}
//--------------------------------------------------------------------------------
RoboPhysics::RoboPhysics(const Point &base, const JointsInputsPtrs &joint_inputs, pEnvEdges edges) :
    RoboI(joint_inputs),
    status{ std::make_shared<Status>(base, joint_inputs) },
    env{ std::make_shared<EnvPhyState>(base, joint_inputs, edges) }
{
    if (!ba::is_sorted(joint_inputs, [](const auto &a, const auto &b) { return (*a < *b); }))
        throw std::runtime_error("Joint Inputs are not sorted!");
}
//--------------------------------------------------------------------------------
RoboPhysics::EnvPhyState::EnvPhyState(const Point &base, const JointsInputsPtrs &joint_inputs, pEnvEdges edges) :
    edges(edges)
{
    joint_t j = 0;
    for (const auto &j_in : joint_inputs)
    {
        if (!j_in->show)
            continue;

        auto law = j_in->frames;
        
        auto nMoveFrames = law.nMoveFrames;
        auto nStopFrames = static_cast<frames_t>(nMoveFrames * law.dInertiaRatio) + 2;
        
        framesMove[j].resize(nMoveFrames);
        framesStop[j].resize(nStopFrames);

        law.moveLaw->generate(framesMove[j].begin(), nMoveFrames,
                              RoboI::minFrameMove, law.dMoveDistance);

        double maxVelosity = *boost::max_element(framesMove[j]);
        law.stopLaw->generate(framesStop[j].begin(), nStopFrames - 1,
                              RoboI::minFrameMove, law.dMoveDistance * law.dInertiaRatio,
                              maxVelosity);
        /* last frame must be 0 to deadend */
        framesStop[j][nStopFrames - 1] = 0.;
#ifdef DEBUG_RM
        {
            printEnviroment(conditions);
            std::cout << std::endl;

            std::stringstream ss;
            ss << "motion-law-" << Utils::ununi(Robo::MotionLaws::name(law.type)) << "-" << int(j) << ".plt";
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
        }
#endif // DEBUG_RM
        ++j;
    }
}
void RoboPhysics::EnvPhyState::reset()
{
    momentum_happened = false;
    // momentum_n_frames = random(1,5);
}
//--------------------------------------------------------------------------------
Enviroment RoboPhysics::getEnvCond() const { return env->conditions; }
void RoboPhysics::setEnvCond(Enviroment conditions) { env->conditions = conditions; }
//--------------------------------------------------------------------------------
RoboPhysics::Status::Status(const Point &base, const JointsInputsPtrs &joint_inputs)
{
    jointsCount = 0;
    for (auto &j_in : joint_inputs)
    {
        if (!j_in->show)
            continue;
        currPos[jointsCount] = j_in->base;
        basePos[jointsCount] = j_in->base;
        ++jointsCount;
    }
    if (!jointsCount)
        throw std::runtime_error("No active joints!");
    currPos[jointsCount] = base;
    basePos[jointsCount] = base;
    musclesCount = RoboI::musclesPerJoint * jointsCount;
    reset();
}
//--------------------------------------------------------------------------------
void RoboPhysics::Status::reset()
{
    auto it = basePos.cbegin();
    for (joint_t joint = 0; joint < jointsCount; ++joint, ++it)
    {
        prevPos[joint] = currPos[joint]; // позиция сочленений текущая будет в предыдущей в след.такт
        prevVel[joint] = Point{}; // скорость сочленения в данный такт
        //acceleration[joint] = Point{}; // ускорение сочленения в данный такт
        //shifts[joint] = 0.;
    }
}
void RoboPhysics::Status::calcCurState(State &state, int spec)
{
    state.special_no = spec;
    state.accelerations.resize(jointsCount);
    state.velosities.resize(jointsCount);
    state.positions.resize(jointsCount);
    for (joint_t joint = 0; joint < jointsCount; ++joint)
    {
        auto vel = (currPos[joint] - prevPos[joint]); // momentum velosity
        state.accelerations[joint] = (vel - prevVel[joint]); // momentum acceleration
        state.velosities[joint] = prevVel[joint] = vel;
        state.positions[joint] = prevPos[joint] = currPos[joint];
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
        auto vel = currPos[joint] - prevPos[joint]; // momentum velosity
        state.accelerations[joint] = (vel - prevVel[joint]); // momentum acceleration
        state.velosities[joint] = vel;
        state.positions[joint] = currPos[joint];
    }
}
bool RoboPhysics::isCollision() const { return env->edges->collision; }
//--------------------------------------------------------------------------------
muscle_t RoboPhysics::musclesCount() const { return status->musclesCount; }
joint_t RoboPhysics::jointsCount() const { return status->jointsCount; }
//unsigned RoboPhysics::getVisitedRarity() const { return env->visitedRarity; }
//void RoboPhysics::setVisitedRarity(unsigned rarity) { env->visitedRarity = rarity; }
Point RoboPhysics::_base() const { return status->basePos[jointsCount()/*base_center*/]; }
//--------------------------------------------------------------------------------
bool RoboPhysics::moveEnd() const { return status->moveEnd; }
const Point& RoboPhysics::position() const { return status->currPos[0]; }
const Point& RoboPhysics::jointPos(IN joint_t joint) const
{
    if (joint >= jointsCount())
        throw std::logic_error("jointPos: Inorrect joint");
    return status->currPos[joint];
}
const Point& RoboPhysics::basePos(joint_t j) const
{ return status->basePos[(j > jointsCount()) ? jointsCount() : j];}
Point& RoboPhysics::currPos(joint_t j)
{ return status->currPos[(j > jointsCount()) ? jointsCount() : j]; }
distance_t RoboPhysics::Imoment(joint_t j) const
{
    const auto mo = muscleByJoint(j, true);
    const auto mc = muscleByJoint(j, false);
    // Добавить моменты инерции, для соседей
    // Когда сочленение заблокировано (1,1) - перемещается, как монолит
    // Иначе на него действуют остальные смещения
    distance_t Imoment = 0.;
    if ((env->conditions & MUTIAL_DYNAMICS) &&
        ((env->conditions & MUTIAL_BLOCKING) && // actuators mutual blocking
         !(status->shifts[mc] > RoboI::minFrameMove &&
           fabs(status->shifts[mc] - status->shifts[mo]) < RoboI::minFrameMove)))
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
void RoboPhysics::getCurrState(rl_problem::ObservationRobo &o) const
{
    o.special_no = specPoint();
    o.positions.resize(jointsCount());
    o.velosities.resize(jointsCount());
    o.accelerations.resize(jointsCount());
    for (joint_t j = 0; j < jointsCount(); ++j)
    {
        o.positions[j] = status->currPos[j];
        o.velosities[j] = status->currPos[j] - status->prevPos[j];
        o.accelerations[j] = o.velosities[j] - status->prevVel[j];
    }
}
void RoboPhysics::currState(State &state) const
{ return status->getCurState(state, specPoint()); }

#ifdef DEBUG_RM
frames_t RoboPhysics::muscleStatus(muscle_t m) const
{ return status->musclesMove[m]; }
frames_t RoboPhysics::lastsStatus(muscle_t m) const
{ return std::max(status->lastsMove[m], status->lastsStop[m]); }
TCHAR RoboPhysics::lastsStatusT(muscle_t m) const
{
    return (status->lastsMove[m] > status->lastsStop[m]) ? (
        (status->lastsMove[m] > 0) ? 'm' : '0') : 's';
}
#endif // DEBUG_RM
