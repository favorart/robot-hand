#include "Robo.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"
#include "RoboLearnMoves.h"
#include "Combinations.h"

using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;


const Robo::frames_t start_next = 1;

//------------------------------------------------------------------------------
/// Добавить к текущему лучшему массиву управлений мини-движения незадействованных мускулов
void LearnMoves::rundownControls(IN OUT Control &controls)
{
    for (joint_t j = 0; j < robo_njoints; ++j)
    {
        auto mo = RoboI::muscleByJoint(j, true);
        auto mc = RoboI::muscleByJoint(j, false);

        auto it_o = br::find(controls | boost::adaptors::reversed, mo);
        auto it_c = br::find(controls | boost::adaptors::reversed, mc);

        if (it_o == controls.rend())
        {
            frames_t start_mo = (it_c != controls.rend()) ? (it_c->lasts + start_next) : 0;
            controls.append({ mo, start_mo, lasts_min });
        }
        if (it_c == controls.rend())
        {
            frames_t start_mc = (it_o == controls.rend()) ? (br::find(controls, mo)->lasts + start_next) : 0;
            controls.append({ mc, start_mc, lasts_min });
        }
    }

    controls.order(robo_nmuscles);
}

//------------------------------------------------------------------------------

//static
//void correctOppositeStart(Robo::Control::iterator &it)
//{
//    if (it->start == 0)
//    {
//        auto  m_op = RoboI::muscleOpposite(it->muscle);
//        auto it_op = br::find(controls | boost::adaptors::reversed, m_op);
//        it_op->start = (it->lasts + start_next);
//    }
//}

//------------------------------------------------------------------------------
/// For Main Directions
bool LearnMoves::rundownNextControl(IN OUT Control  &controls, IN OUT   size_t &controls_curr,
                                    IN OUT frames_t &velosity, IN OUT frames_t &velosity_prev)
{
    if (controls_curr >= controls.size())
        return true;
    // ---------------------------------
    auto it = controls.begin();
    if (controls_curr == 0)
    {
        it->lasts += velosity;
        //correctOppositeStart(it);
        if (it->start == 0)
        {
            auto  m_op = RoboI::muscleOpposite(it->muscle);
            auto it_op = br::find(controls | boost::adaptors::reversed, m_op);
            it_op->start = (it->lasts + start_next);
        }
    }
    else
    {
        std::advance(it, controls_curr / RoboI::musclesPerJoint);
        // ---------------------------------
        if (controls_curr % RoboI::musclesPerJoint)
        {
            if (it->lasts >= (velosity_prev + velosity))
                it->lasts -= (velosity_prev + velosity);
            else
            {
                auto  m_op = RoboI::muscleOpposite(it->muscle);
                auto it_op = br::find(controls | boost::adaptors::reversed, m_op);
                // ---------------------------------
                it_op->lasts += (velosity_prev + velosity - it->lasts);
                it->lasts = 0;
                // ---------------------------------
                if (it_op->start == 0)
                {
                    it->start = (it_op->lasts + start_next);
                    it_op->start = 0;
                }
                else
                {
                    it_op->start = (it->lasts + start_next);
                    it->start = 0;
                }
            } // end else

            if (it->start == 0)
            {
                auto  m_op = RoboI::muscleOpposite(it->muscle);
                auto it_op = br::find(controls, m_op);
                it_op->start = (it->lasts + start_next);
            }
        }
        else
        {
            auto jt = std::prev(it);
            std::swap(it, jt);
            // ---------------------------------
            (it)->lasts += velosity_prev;
            (jt)->lasts += velosity;
            // ---------------------------------
            if (it->start == 0)
            {
                auto  m_op = RoboI::muscleOpposite(it->muscle);
                auto it_op = br::find(controls, m_op);
                it_op->start = (it->lasts + 1U);
            }
            // ---------------------------------
            if (jt->start == 0)
            {
                auto  m_op = RoboI::muscleOpposite(jt->muscle);
                auto jt_op = br::find(controls, m_op);
                jt_op->start = (jt->lasts + 1U);
            }
            // ---------------------------------
        } // end else
    } // end else
    // ---------------------------------
    velosity_prev = velosity;
    // ---------------------------------
    ++controls_curr;
    return false;
}

//------------------------------------------------------------------------------
Robo::distance_t LearnMoves::rundownMainDir(IN const Point &aim)
{
    /// !!!!!!!!!!! store.approx().
    auto p = _store->getClosestPoint(aim, annealing);
    if (!p.first)
        CERROR(_T("rundownMainDir: Empty adjacency"));
    // -----------------------------------------------
    Control controls{ p.second.controls };
    rundownControls(controls);
    // -----------------------------------------------
    distance_t distance, prev_distance, next_distance;
    next_distance = distance = bg::distance(aim, p.second.hit);
    // -----------------------------------------------
    frames_t velosity = frames_t(floor(distance / _target->precision() + 0.5));
    frames_t velosity_prev = 0;
    // -----------------------------------------------
    size_t controls_curr = 0;
    // -----------------------------------------------
    while (!rundownNextControl(controls, controls_curr,
                               velosity, velosity_prev))
    {
        do
        {
            // -----------------------------------------------
            prev_distance = next_distance;
            next_distance = actionRobo(aim, controls);
            //++_rundown_maindir_complexity;
            updateReachedStat(Admix::DirRundown);
            // -----------------------------------------------
            if (less(distance, next_distance))
            {
                distance = next_distance;
                if (check_precision(distance, aim))
                    break;
            }
            // -----------------------------------------------
            velosity = frames_t(floor(distance / _target->precision() + 0.5));
            // -----------------------------------------------
            auto it = controls.begin();
            std::advance(it, controls_curr / 2U);
            // ---------------------------------
            if ((controls_curr % 2U))
            {
                if (it->lasts >= velosity)
                { it->lasts -= velosity; }
                else
                {
                    auto  m_op = RoboI::muscleOpposite(it->muscle);
                    auto it_op = boost::range::find(controls, m_op);
                    // ---------------------------------
                    it_op->lasts += (velosity - it->lasts);
                    it->lasts = 0U;
                    // ---------------------------------
                    if (it_op->start == 0U)
                    { it->start = (it_op->lasts + 1U); }
                    else
                    { it_op->start = (it->lasts + 1U); }
                }
            }
            else
            { it->lasts += velosity; }
            // ---------------------------------
            if (it->start == 0U)
            {
                auto  m_op = RoboI::muscleOpposite(it->muscle);
                auto it_op = boost::range::find(controls, m_op);
                it_op->start = (it->lasts + 1U);
            }
            // ---------------------------------        
            velosity_prev = velosity;
            // ---------------------------------
        } while (less(prev_distance, next_distance));
    } // end while  rundownNextControl

    CINFO(_T("rundownMainDir precision: ") << distance);
    return distance;
}

//------------------------------------------------------------------------------
class LearnMoves::RundownAllDirsIncrementor final
{
    const size_t directions_ = 2; // +/-
    const joint_t n_joints_;
    const muscle_t n_muscles_;
    size_t n_combs_ = 1;
    std::vector<int> alphabet_;
public:
    RundownAllDirsIncrementor(joint_t n_joints) :
        n_joints_(n_joints), 
        n_muscles_(n_joints * RoboI::musclesPerJoint)//,
        //alphabet_(size_t(n_joints  * RoboI::musclesPerJoint * directions_), 0)
    {
        alphabet_.resize(RoboI::musclesPerJoint * n_joints * directions_);
        reset();
    }
    void reset()
    {
        n_combs_ = 1;
        int inc = 0;
        for (auto &letter : alphabet_)
            letter = inc++;
        //print_state();
    }
    void print_state() const
    {
        tcout << _T("alphabet: ");
        for (auto it = alphabet_.begin(); it != (alphabet_.begin() + n_combs_); ++it)
            tcout << *it << _T(" ");
        tcout << std::endl;
    }
    bool next(changes_t &changes)
    {
        bool result, one_more;
        do
        {
            changes.assign(size_t(n_muscles_), 0);
            one_more = false;
            change_t it_prev = n_muscles_;
            for (auto it = alphabet_.begin(); it != (alphabet_.begin() + n_combs_); ++it)
            {
                change_t it_curr = *it % n_muscles_;
                if (n_combs_ > 1 && it_prev == it_curr)
                {
                    one_more = true;
                    break;
                }
                changes[it_curr] = (*it >= n_muscles_) ? -1 : 1;
                it_prev = it_curr;
            } // end for
            result = next_combination(alphabet_.begin(),
                                      alphabet_.begin() + n_combs_,
                                      alphabet_.end());
            //print_state();
            if (!result && ++n_combs_ > changes.size())
                reset();
        } while (one_more);
        return !result;
    }
    static void print_changes(changes_t &changes)
    {
        tcout << _T("lasts_changes: ");
        for (auto &chg : changes)
            tcout << chg << _T(" ");
        tcout << std::endl;
    }
};

//------------------------------------------------------------------------------
bool LearnMoves::rundownNextControl(IN OUT Robo::Control  &controls,
                                    IN OUT changes_t      &lasts_changes,
                                    IN OUT Robo::frames_t  velosity,
                                    std::vector<int> &muscles_repeats)
{
    if (!velosity)
        return false;

    bool one_more = false;
    for (size_t pos = 0; pos < controls.size(); ++pos)
    {
        if (lasts_changes[controls[pos].muscle] < 0) // укорачивать имеющееся управление
        {
            if (muscles_repeats[controls[pos].muscle] < 0) // ????
            {
                muscles_repeats[controls[pos].muscle] = int(pos);
                controls.shorter(pos, velosity,
                                 true/*уточнить ли время старта противоположного?*/,
                                 true/*добавлять ли в противоположное управление? какое?*/);
            }
            //else one_more = true; // ???
        }
        else if (lasts_changes[controls[pos].muscle] > 0) // добавлять в имеющееся управление
        {
            if (muscles_repeats[controls[pos].muscle] < 0) // ????
            {
                muscles_repeats[controls[pos].muscle] = int(pos);
                controls.longer(pos, velosity,
                                true/*уточнить ли время старта противоположного?*/);
            }
            //else one_more = true; // ???
        }
    }
    if (!controls.validate(robo_nmuscles))
        tcout << std::endl;
    //controls.order(_robo->musclesCount());
    return one_more;
}

//------------------------------------------------------------------------------
Robo::frames_t LearnMoves::rundownVelosity(Robo::distance_t distance)
{ return static_cast<Robo::frames_t>( std::max(1., std::ceil(distance / _target->precision())) ); }

//------------------------------------------------------------------------------
Robo::distance_t LearnMoves::rundownAllDirs(IN const Point &aim)
{
    auto p = _store->getClosestPoint(aim, annealing);
    if (!p.first)
        CERROR(_T("rundownAllDirs: Empty adjacency"));
    // -----------------------------------------------
    distance_t distance, next_distance;
    next_distance = distance = bg::distance(aim, p.second.hit);
    // -----------------------------------------------
    Control controls{ p.second.controls };
    rundownControls(controls);
    CINFO(_T("rundownControls: ") << controls);
    // -----------------------------------------------
    Control tmp(controls);
    // -----------------------------------------------
    RundownAllDirsIncrementor increm(robo_njoints);
    changes_t lasts_changes(size_t{ robo_nmuscles });
    bool finish;
    // -----------------------------------------------
    do
    {
        tmp = controls;
        /* Взять новое сочетание */
        finish = increm.next(lasts_changes);
        // -----------------------------------------------
        RundownAllDirsIncrementor::print_changes(lasts_changes);
        // -----------------------------------------------
        /* что такое muscles_repeats ?? */
        std::vector<int> muscles_repeats(size_t(robo_nmuscles), -1);
        std::vector<int> muscles_repeats_prev(size_t(robo_nmuscles), -1);
        bool closer, repeat, reset = false;
        // -----------------------------------------------
        do
        {
            auto velosity = rundownVelosity(distance);
            repeat = rundownNextControl(tmp, lasts_changes, velosity, muscles_repeats);
            // -----------------------------------------------
#ifdef USE_REACH_STAT
            CINFO(_T("rundownNextControl: ") << tmp << " v=" << velosity);
            tcout << " muscles_repeats={ ";
            for (auto &r : muscles_repeats) tcout << r << " , ";
            tcout << " } repeat=" << repeat << std::endl;
#endif
            // -----------------------------------------------
            //auto hit = predict(controls);
            //auto d = bg::distance(aim, hit);
            //if (less(distance, d))
            {
                next_distance = actionRobo(aim, tmp);
                //++_rundown_alldirs_complexity;
                updateReachedStat(Admix::AllRundown);
            }
            // -----------------------------------------------
            if (less(distance, next_distance))
            {
                distance = next_distance;
                //if (check_precision(distance, aim))
                {
                    finish = true;
                    break;
                }
                //controls = tmp;
                //muscles_repeats = muscles_repeats_prev;
                reset = closer = true;
                //tcout << " closer" << std::endl;
            }
            else
            {
                muscles_repeats_prev = muscles_repeats;
                closer = false;
                //if (closer) finish = true;
            }
        } while (/*closer ||*/ repeat);
        //if (reset) increm.reset(); // начинаем перебор сначала с нового лучшего управления
    } while (!finish);

    CINFO("rundownAllDirs precision: " << distance);
    return distance;
}

//------------------------------------------------------------------------------

