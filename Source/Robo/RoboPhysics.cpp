#include "RoboPhysics.h"
#include "RoboEdges.h"
#include "RoboPhysicsStatus.h"
#include "RoboEnviroment.h"
#include "RoboMuscles.h"

using namespace Robo;
//--------------------------------------------------------------------------------
bool RoboI::operator==(const RoboI &r) const
{
    if (this == &r)
        return true;
    if (getName() == r.getName() && _base() == r._base())
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
        if (containE(getEnvCond(), ENV::START_FRICTION))
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
    if (containE(getEnvCond(), ENV::WINDY) && status->lastsMove[muscle] == 1)
    {
        Frame = Utils::random(RoboI::minFrameMove, maxFrame);
    }
    // --- MOMENTUM CHANGES ----------------------
    if (containE(getEnvCond(), ENV::MOMENTUM_CHANGES))
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
    if (!somethingMoving()) // Исключить незадействованные двигатели
    {
        CDEBUG("Nothing moving");
        return _frame;
    }
    while (_frame < max_frames && !moveEnd()) // just moving
    {
        if (frame() > RoboPhysics::LastsTooLong * 2)
            CERROR(" move lasts too long ");
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
    for (auto &a : controls) // starts all moves
    {
        if (a.lasts > RoboPhysics::LastsTooLong ||
            a.start > RoboPhysics::LastsTooLong ||
            frame() > RoboPhysics::LastsTooLong * 2)
            CERROR(" move lasts too long ");
        //CDEBUG("c:" << c);
        while (a.start > _frame)
            step();
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
    status(std::make_shared<Status>(base, joint_inputs)),
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
        
        framesMove[j].resize(nMoveFrames);
        framesStop[j].resize(nStopFrames);

        laws[j].moveLaw->generate(framesMove[j].begin(), nMoveFrames,
                                  RoboI::minFrameMove, laws[j].dMoveDistance);

        double maxVelosity = *boost::max_element(framesMove[j]);
        laws[j].stopLaw->generate(framesStop[j].begin(), nStopFrames - 1,
                                  RoboI::minFrameMove, laws[j].dMoveDistance * laws[j].dInertiaRatio,
                                  maxVelosity);
        /* last frame must be 0 to deadend */
        framesStop[j][nStopFrames - 1] = 0.;
#ifdef DEBUG_PLOT_PHY_STATE
        {
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
        }
#endif // DEBUG_PLOT_PHY_STATE
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
    const auto m_o = muscleByJoint(j, true);
    const auto m_c = muscleByJoint(j, false);
    // Добавить моменты инерции, для соседей
    // Когда "сочленение заблокировано"=(1,1) - то перемещается, как "монолит",
    // Иначе на него действуют смещения остальных сочленений
    distance_t Imoment = 0.;
    if (containE(getEnvCond(), ENV::MUTIAL_DYNAMICS) &&
        containE(getEnvCond(), ENV::MUTIAL_BLOCKING) && // actuators mutual blocking
         !(status->shifts[m_c] > RoboI::minFrameMove &&
           fabs(status->shifts[m_c] - status->shifts[m_o]) < RoboI::minFrameMove))//)
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
                    fdat << env->framesMove[j][x] * this->prismatic_factor(j);
                else if (x < env->nFramesAll(j))
                    fdat << env->framesStop[j][x - env->nFramesMove(j)] * this->prismatic_factor(j);
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
        const auto prismatic = this->prismatic_factor(joint);
        auto fprinter = [&fplot, &x, &prismatic](Robo::distance_t item) {
            fplot << x++ << '\t' << item * prismatic << std::endl;
        };
        br::for_each(env->framesMove[joint], fprinter);
        br::for_each(env->framesStop[joint], fprinter);        
    }
#endif // DEBUG_PLOTTING
}

