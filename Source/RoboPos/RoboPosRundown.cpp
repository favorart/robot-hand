#include "StdAfx.h"
#include "Robo.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"
#include "RoboLearnMoves.h"
#include "Combinations.h"


using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;
//------------------------------------------------------------------------------
void   LearnMoves::rundownControls(IN OUT Control &controls)
{
    for (joint_t j = 0; j < _robo.jointsCount(); ++j)
    {
        auto mo = RoboI::muscleByJoint(j, true);
        auto mc = RoboI::muscleByJoint(j, false);

        auto it_o = std::find(controls.begin(), controls.end(), mo);
        auto it_c = std::find(controls.begin(), controls.end(), mc);

        if (it_o == controls.end() && it_c != controls.end())
        { controls.append({ mo, it_c->lasts + 1U, lasts_min }); }
        else if (it_o == controls.end())
        { controls.append({ mo, 0U, lasts_min }); }

        if (it_c == controls.end())
        {
            if (it_o == controls.end())
            { it_o = std::find(controls.begin(), controls.end(), mo); }
            controls.append({ mc, it_o->lasts + 1U, lasts_min });
        }
    }
}
//------------------------------------------------------------------------------
bool   LearnMoves::rundownNextControl(IN OUT Control  &controls, IN OUT   size_t &controls_curr,
                                      IN OUT frames_t &velosity, IN OUT frames_t &velosity_prev)
{
    if (controls_curr >= controls.size())
    { return true; }
    // ---------------------------------
    auto it = controls.begin();
    if (controls_curr == 0U)
    {
        it->lasts += velosity;
        if (it->start == 0U)
        {
            auto  m_op = RoboI::muscleOpposite(it->muscle);
            auto it_op = boost::range::find(controls, m_op);
            it_op->start = (it->lasts + 1U);
        }
    }
    else
    {
        std::advance(it, controls_curr / 2U);
        // ---------------------------------
        if ((controls_curr % 2U))
        {
            if (it->lasts >= (velosity_prev + velosity))
            { it->lasts -= (velosity_prev + velosity); }
            else
            {
                auto  m_op = RoboI::muscleOpposite(it->muscle);
                auto it_op = boost::range::find(controls, m_op);
                // ---------------------------------
                it_op->lasts += (velosity_prev + velosity - it->lasts);
                it->lasts = 0U;
                // ---------------------------------
                if (it_op->start == 0U)
                {
                    it->start = (it_op->lasts + 1U);
                    it_op->start = 0U;
                }
                else
                {
                    it_op->start = (it->lasts + 1U);
                    it->start = 0U;
                }
            } // end else

            if (it->start == 0U)
            {
                auto  m_op = RoboI::muscleOpposite(it->muscle);
                auto it_op = boost::range::find(controls, m_op);
                it_op->start = (it->lasts + 1U);
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
            if (it->start == 0U)
            {
                auto  m_op = RoboI::muscleOpposite(it->muscle);
                auto it_op = boost::range::find(controls, m_op);
                it_op->start = (it->lasts + 1U);
            }
            // ---------------------------------
            if (jt->start == 0U)
            {
                auto  m_op = RoboI::muscleOpposite(jt->muscle);
                auto jt_op = boost::range::find(controls, m_op);
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
    auto p = _store.getClosestPoint(aim, annealing);
    if (!p.first)
        CERROR(_T("rundownMainDir: Empty adjacency"));
    // -----------------------------------------------
    Control controls{ p.second.controls };
    rundownControls(controls);
    // -----------------------------------------------
    distance_t distance, prev_distance, next_distance;
    next_distance = distance = bg::distance(aim, p.second.hit);
    // -----------------------------------------------
    frames_t velosity = frames_t(floor(distance / _target.precision() + 0.5));
    frames_t velosity_prev = 0;
    // -----------------------------------------------
    size_t  controls_curr = 0;
    // -----------------------------------------------
    while (!rundownNextControl(controls, controls_curr,
                               velosity, velosity_prev))
    {
        do
        {
            // -----------------------------------------------
            prev_distance = next_distance;
            next_distance = actionRobo(aim, controls);
            //++rundown_complexity;
            // -----------------------------------------------
            if (next_distance < distance)
            {
                distance = next_distance;
                if (distance < _target.precision())
                {
                    CINFO(aim << _T(" reached"));
                    break;
                }
            }
            // -----------------------------------------------
            velosity = frames_t(floor(distance / _target.precision() + 0.5));
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
        } while (next_distance < prev_distance);
    } // end while  rundownNextControl

    CINFO(_T("rundownMainDir precision: ") << distance);
    //CINFO(_T("rundown complexity: ") << rundown_complexity);
    return distance;
}


//------------------------------------------------------------------------------
class RundownFullIncrementor
{
    std::vector<int>  alphabet_;
    const size_t directions_ = 2U; // +/-
    size_t  n_joints_;
    size_t  n_combs_ = 1U;

public:
    RundownFullIncrementor(size_t n_joints) :
        n_joints_(n_joints),
        alphabet_(2 * n_joints * directions_)
    {
        reset();
    }

    void  reset()
    {
        n_combs_ = 1U;
        // --------------------------------------------------
        int inc = 0;
        // --------------------------------------------------
        for (auto &letter : alphabet_)
        { letter = inc++; }
    }
    bool  next(std::vector<int> &currents)
    {
        bool result, one_more;
        // --------------------------------------------------
        do
        {
            one_more = false;
            currents.assign(currents.size(), 0);
            // --------------------------------------------------
            int  it_curr = 0,
                it_prev = static_cast<int> (currents.size());
            // --------------------------------------------------
            for (auto it = alphabet_.begin();
                 it != alphabet_.begin() + n_combs_;
                 ++it)
            {
                it_curr = *it % currents.size();
                if (n_combs_ > 1 && it_prev == it_curr)
                {
                    one_more = true;
                    break;
                }
                // --------------------------------------------------
                currents[it_curr] = (*it >= int(currents.size())) ? -1 : 1;
                it_prev = it_curr;
            } // end for
            // --------------------------------------------------
            result = next_combination(alphabet_.begin(),
                                      alphabet_.begin() + n_combs_,
                                      alphabet_.end());
            // --------------------------------------------------
            if (!result)
            {
                ++n_combs_;
                if (n_combs_ > n_joints_)
                { reset(); result = true; }
            }
            // --------------------------------------------------
        } while (one_more);
        // --------------------------------------------------
        return result;
    }
};

//------------------------------------------------------------------------------
bool LearnMoves::rundownNextControl(IN OUT Control    &controls,
                                    IN OUT std::vector<int> &lasts_changes,
                                    IN OUT frames_t   &velosity)
{
    bool result = true;
    // ---------------------------------
    if (controls.size() != lasts_changes.size())
        CERROR(_T("rundownNextControl: not equal sizes controls and lasts_changes"));
    // ---------------------------------
    auto it = controls.begin();
    for (auto last_change : lasts_changes)
    {
        if (last_change)
        {
            // ---------------------------------
            if (last_change < 0)
            {
                if (it->lasts >= velosity)
                { it->lasts -= velosity; }
                else
                {
                    velosity = it->lasts;
                    it->lasts = 0U;
                }

                // {
                //   auto  m_op = muscleOpposite (it->muscle);
                //   auto it_op = boost::range::find (controls, m_op);
                //   // ---------------------------------
                //   it_op->last += (velosity - it->last);
                //   it->last = 0U;
                //   // ---------------------------------
                //   if ( it_op->start < it->start )
                //   {
                //     it->start = (it_op->last + 1U);
                //     it_op->start = 0U;
                //   }
                //   else // it_op->start > it->start
                //   {
                //     it_op->start = (it->last + 1U);
                //     it->start = 0U;
                //   }
                // } // end else

                auto  m_op = RoboI::muscleOpposite(it->muscle);
                auto it_op = boost::range::find(controls, m_op);

                if (it->start < it_op->start)
                { it_op->start = (it->lasts + 1U); }
            }
            else // last_change > 0
            {
                it->lasts += velosity;
                // ---------------------------------
                auto  m_op = RoboI::muscleOpposite(it->muscle);
                auto it_op = boost::range::find(controls, m_op);

                if (it->start < it_op->start)
                { it_op->start = (it->lasts + 1U); }
                // ---------------------------------
            } // end else
            // ---------------------------------
            result = false;
        } // end if
        ++it;
    } // end for
    // ---------------------------------
    return result;
}
//------------------------------------------------------------------------------
Robo::distance_t LearnMoves::rundownAllDirs(IN const Point &aim)
{
    auto p = _store.getClosestPoint(aim, annealing);
    if (!p.first)
        CERROR(_T("rundownFull: Empty adjacency"));
    // -----------------------------------------------
    distance_t distance, next_distance;
    next_distance = distance = bg::distance(aim, p.second.hit);
    // -----------------------------------------------
    Control controls{ p.second.controls };
    rundownControls(controls);
    // -----------------------------------------------
    frames_t velosity = 0;
    frames_t velosity_prev = 0;
    // -----------------------------------------------
    std::vector<int>  lasts_changes(_robo.musclesCount());
    // -----------------------------------------------
    RundownFullIncrementor   increm(_robo.jointsCount());
    // -----------------------------------------------
    while (distance > _target.precision())
    {
        velosity = frames_t(floor(distance / _target.precision() + 0.5));
        velosity = (velosity) ? velosity : 1U;
        // -----------------------------------------------
        /* Восстановить прошлое управление */
        for (auto &last_change : lasts_changes)
        {
            if (last_change != 0)
            { last_change = -last_change; }
        }
        rundownNextControl(controls, lasts_changes, velosity_prev);
        // -----------------------------------------------
        /* Взять новое сочетание */
        bool increm_result = increm.next(lasts_changes);
        // -----------------------------------------------
        rundownNextControl(controls, lasts_changes, velosity);

        velosity_prev = velosity;
        // -----------------------------------------------
        next_distance = actionRobo(aim, controls);
        //++rundown_complexity;
        if (next_distance < _target.precision())
        {
            CINFO(aim << _T(" reached"));
            break;
        }
        // -----------------------------------------------
        while (next_distance < distance)
        {
            distance = next_distance;
            // -----------------------------------------------
            frames_t velosity_new = frames_t(floor((distance) / _target.precision() + 0.5));
            velosity_new = (velosity_new) ? velosity_new : 1U;

            if (velosity_new != velosity)
            {
                velosity = velosity_new;
                for (auto &l : lasts_changes)
                { l = static_cast<int>(boost::math::sign(l) * velosity); }
            }
            // -----------------------------------------------
            rundownNextControl(controls, lasts_changes, velosity);

            velosity_prev = velosity;
            // -----------------------------------------------
            next_distance = actionRobo(aim, controls);
            // ++rundown_complexity;
            // -----------------------------------------------
            if (next_distance < _target.precision())
            {
                CINFO(aim << _T(" reached"));
                break;
            }
            if (next_distance >= distance)
            {
                increm.reset();
                increm_result = false;
            }
        }
        // -----------------------------------------------
        // if ( next_distance > side )
        if (increm_result)
        {
            CINFO(_T("rundownAllDirs FAIL ") << distance);
            break; /* FAIL */
        }
    } // end while

    CINFO("rundownAllDirs precision: " << distance);
    //CINFO("rundown complexity: " << rundown_complexity);
    return distance;
}

//------------------------------------------------------------------------------

