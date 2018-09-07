#include "StdAfx.h"
#include "RoboPos.h"
#include "RoboLearnMoves.h"


using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;
//------------------------------------------------------------------------------
void RoboPos::LearnMoves::gradientControls(IN const Point   &aim, IN  double d_d,
                                           IN const Control &inits_controls,
                                           IN const Control &lower_controls,
                                           IN const Control &upper_controls,
                                           OUT      Control &controls)
{
    auto  iter = controls.begin();
    // -----------------------------------------------
    for (joint_t joint = 0; joint < _robo.jointsCount(); ++joint)
    {
        auto mo = RoboI::muscleByJoint(joint, true);
        auto mc = RoboI::muscleByJoint(joint, false);
        
        auto it_mo_i = br::find(inits_controls, mo);
        auto it_mo_l = br::find(lower_controls, mo);
        auto it_mo_g = br::find(upper_controls, mo);

        auto it_mc_i = br::find(inits_controls, mc);
        auto it_mc_l = br::find(lower_controls, mc);
        auto it_mc_g = br::find(upper_controls, mc);
        
        int last_mo_i = (it_mo_i != inits_controls.end()) ? (it_mo_i->lasts) : 0U;
        int last_mo_l = (it_mo_l != lower_controls.end()) ? (it_mo_l->lasts - last_mo_i) : 0U;
        int last_mo_g = (it_mo_g != upper_controls.end()) ? (it_mo_g->lasts - last_mo_i) : 0U;
        
        int last_mc_i = (it_mc_i != inits_controls.end()) ? (it_mc_i->lasts) : 0U;
        int last_mc_l = (it_mc_l != lower_controls.end()) ? (it_mc_l->lasts - last_mc_i) : 0U;
        int last_mc_g = (it_mc_g != upper_controls.end()) ? (it_mc_g->lasts - last_mc_i) : 0U;
        // -----------------------------------------------
        const auto int_normalizer = int(d_d / _precision); /* velosity */ // 400;
        
        
        // -----------------------------------------------
        int direction_o = 0;
        int direction_c = 0;
        
        if (last_mo_l || last_mo_g)
        {
            double  d = 0.;
            if (last_mo_l + last_mo_g)
            {
                d = d_d / (last_mo_l + last_mo_g);
                // direction_o = ((d_d * int_normalizer) / (last_mo_l + last_mo_g));
                direction_o = int(d * int_normalizer);
            }
        
            if (direction_o == 0)
            {
                // BACKWARD MOVEMENT
                if (d < 0.)
                { ++direction_c; /* = (direction_o) ? (direction_o + 1) : 50; */ }
                else if (d > 0.)
                { --direction_o; /* = (direction_c < 0) ? -1 : 1; */ }
            }
        }
        
        if (last_mc_l || last_mc_g)
        {
            double  d = 0.;
            if (last_mc_l + last_mc_g)
            {
                d = d_d / (last_mc_l + last_mc_g);
                // direction_c = ((d_d * int_normalizer) / (last_mc_l + last_mc_g));
                direction_c = int(d * int_normalizer);
            }
        
            if (direction_c == 0)
            {
                // BACKWARD MOVEMENT
                if (d < 0.)
                { ++direction_o; /* = (direction_o) ? (direction_o + 1) : 50; */ }
                else if (d > 0.)
                { --direction_c; /* = (direction_c < 0) ? -1 : 1; */ }
            }
        }
        
        frames_t last_o = 0U;
        frames_t last_c = 0U;
        
        if (direction_o < 0 || (direction_o >= 0 && last_mo_i > direction_o))
        { last_o += (last_mo_i - direction_o); }
        else if ((direction_o >= 0 && last_mo_i < direction_o))
        {
            last_o = 0U;
            last_c += (last_mo_i + (direction_o - last_mo_i));
        }
        
        if (direction_c < 0 || (direction_c >= 0 && last_mc_i > direction_c))
        { last_c += (last_mc_i - direction_c); }
        else if ((direction_c >= 0 && last_mc_i <= direction_c))
        {
            last_c = 0U;
            last_o += (last_mc_i + (direction_c - last_mc_i));
        }
        
        if (last_o != last_c)
        {
            frames_t start_o = (last_o > last_c) ? 0U : (last_c + 1U);
            frames_t start_c = (last_o < last_c) ? 0U : (last_o + 1U);
        
            if (last_o) controls.append({ mo, start_o, last_o }); //{ controls.insert(iter, Control(mo, start_o, last_o)); }
            if (last_c) controls.append({ mc, start_c, last_c }); //{ controls.insert(iter, Control(mc, start_c, last_c)); }
        }
    }
}

//------------------------------------------------------------------------------
size_t RoboPos::LearnMoves::gradientMethod_admixture(IN const Point &aim)
{
    size_t gradient_complexity = 0U;
    // -----------------------------------------------
    _robo.reset();
    Point base_pos = _robo.position();
    Point pos; // hand_position
    // -----------------------------------------------
    double distance = boost_distance(base_pos, aim),
        new_distance = distance;
    // -----------------------------------------------
    do
    {
        // -----------------------------------------------
        Record  rec;
        Control lower_controls, upper_controls;
        double  lower_distance, upper_distance;
        // -----------------------------------------------
        if (!weightedMeanULAdjs(aim, &rec,
                                lower_controls, upper_controls,
                                lower_distance, upper_distance))
        {
            break;
        }
        // -----------------------------------------------
        const Control &inits_controls = rec.controls;
        double  d_d = (upper_distance - lower_distance);
        // -----------------------------------------------
        double d = boost_distance(rec.hit, aim);
        if (_precision > d)
        { break; }
        // -----------------------------------------------
        if (new_distance > d)
        { new_distance = d; }
        // -----------------------------------------------
        else
        {
            gradient_complexity += weightedMean(aim, pos);

            d = boost_distance(pos, aim);
            if (_precision > d)
            { break; }
            else if (new_distance > d)
            { continue; }
            else
            {
                gradient_complexity += rundownMDir(aim, pos);

                d = boost_distance(pos, aim);
                if (_precision > d)
                { break; }
                else if (new_distance > d)
                { continue; }
                else
                {
                    gradient_complexity += gradientMethod(aim);

                    auto p = _store.getClosestPoint(aim, _stage3_params.side);
                    if (!p.first)
                        throw std::runtime_error{ "gradientMethod_admixture: Empty adjacency" };
                    pos = p.second.hit;

                    d = boost_distance(pos, aim);
                    if (_precision > d)
                    { break; }
                    else if (new_distance > d)
                    { continue; }
                    else
                    {
                        /* FAIL */
                        break;
                    } // end else
                } // end else
            } // end else
        } // end else
        // -----------------------------------------------
        Control controls;
        gradientControls(aim, d_d,
                         inits_controls,
                         lower_controls,
                         upper_controls,
                         controls);
        // -----------------------------------------------
        if (actionRobo(aim, controls, pos))
        { ++gradient_complexity; }
        // -----------------------------------------------
        d = boost_distance(pos, aim);
        // -----------------------------------------------
        if (new_distance > d)
        { new_distance = d; }
        if (d > _stage3_params.side)
        { /* FAIL */
            break;
        }
        // -----------------------------------------------
        if (distance > new_distance)
        { distance = new_distance; }
        // -----------------------------------------------
    } while (_precision < distance);
    // -----------------------------------------------
    tcout << _T("_precision: ") << distance << std::endl;
    tcout << _T("gradient admix complexity: ")
          << gradient_complexity
          << std::endl << std::endl;
    // -----------------------------------------------
    return  gradient_complexity;
}
//------------------------------------------------------------------------------
const Record* RoboPos::LearnMoves::gradientClothestRecord(IN const adjacency_ptrs_t &range,
                                                          IN const Point            &aim,
                                                          IN const HitPosRelToAim   *pHitPosPred,
                                                          IN OUT   visited_t        *pVisited)
{
    const Record *pRecMin = NULL;
    // ------------------------
    size_t h;
    double dr, dm;
    // ------------------------
    for ( const auto &pRec : range )
    {
        if (pVisited)
        {
            RecordHasher rh;
            h = rh(*pRec);
        }
        // ------------------------
        dr = boost_distance(pRec->hit, aim);
        if ((!pVisited || pVisited->find(h) == pVisited->end())
            && (!pHitPosPred || (*pHitPosPred) (*pRec, aim))
            && (!pRecMin || dr < dm))
        {
            pRecMin = pRec; // &(*pRec);
            dm = dr;
        }
    }
    // ------------------------
    return pRecMin;
}
//------------------------------------------------------------------------------
bool RoboPos::LearnMoves::gradientSomeClothestRecords(IN  const Point &aim,
                                                      OUT Record *pRecClose,
                                                      OUT Record *pRecLower,
                                                      OUT Record *pRecUpper,
                                                      IN OUT visited_t *pVisited)
{
    if (!pRecClose) { return  false; }
    // ------------------------------------------------
    Point min(aim.x - _stage3_params.side, aim.y - _stage3_params.side),
          max(aim.x + _stage3_params.side, aim.y + _stage3_params.side);
    // ------------------------------------------------
    adjacency_ptrs_t range;
    _store.adjacencyRectPoints<adjacency_ptrs_t, ByP>(range, min, max);
    // ------------------------------------------------
    HitPosRelToAim cmp_l = [](const Record &p, const Point &aim) { return  (p.hit.x < aim.x) && (p.hit.y < aim.y); };
    HitPosRelToAim cmp_g = [](const Record &p, const Point &aim) { return  (p.hit.x > aim.x) && (p.hit.y > aim.y); };
    // ------------------------------------------------
    const Record *pRec = gradientClothestRecord(range, aim, NULL, pVisited);
    if (!pRec) { return false; }

    RecordHasher rh;
    size_t h = rh(*pRec);
    pVisited->insert(h);
    // ===========
    *pRecClose = *pRec;
    // ===========
    // ------------------------------------------------
    if (pRecLower && pRecUpper)
    {
        // ------------------------------------------------
        pRec = gradientClothestRecord(range, aim, &cmp_l, pVisited);
        if (!pRec) { return false; }
        // ===========
        *pRecLower = *pRec;
        // ===========
        // ------------------------------------------------
        pRec = gradientClothestRecord(range, aim, &cmp_g, pVisited);
        if (!pRec) { return false; }
        // ===========
        *pRecUpper = *pRec;
        // ===========
        // ------------------------------------------------
    }
    // ------------------------------------------------
    return true;
}
//------------------------------------------------------------------------------
size_t RoboPos::LearnMoves::gradientMethod(IN const Point &aim)
{
    size_t  gradient_complexity = 0U;
    // -----------------------------------------------

    std::set<size_t> visited;
    // -----------------------------------------------
    double distance = boost_distance(_base_pos, aim);
    double new_distance = 0.;
    // -----------------------------------------------
    _robo.reset();
    do
    {
        Record  rec_close, rec_lower, rec_upper;
        if (!gradientSomeClothestRecords(aim, &rec_close,
                                         &rec_lower,
                                         &rec_upper,
                                         &visited))
        { 
            /* FAIL */
            break;
        }
        Point hit = rec_close.hit;

        new_distance = boost_distance(hit, aim);
        if (new_distance < distance)
            distance = new_distance;

        if (_precision > new_distance) { break; }

        Control inits_controls{ rec_close.controls };
        // -----------------------------------------------
        double  lower_distance = boost_distance(aim, rec_lower.hit);
        double  upper_distance = boost_distance(aim, rec_upper.hit);

        double  d_d = (upper_distance - lower_distance);
        // -----------------------------------------------
        Control controls;
        gradientControls(aim, d_d,
                         inits_controls,
                         rec_lower.controls,
                         rec_upper.controls,
                         controls);
        // -----------------------------------------------
        if (actionRobo(aim, controls, hit))
        { ++gradient_complexity; }
        // -----------------------------------------------
        double d = boost_distance(hit, aim);
        // -----------------------------------------------
        if (_precision > d)
        { break; }
        // -----------------------------------------------
        else if (new_distance > d)
        { new_distance = d; }
        // -----------------------------------------------
        else
        {
            visited.clear();
            rundownFull(aim, hit);

            d = boost_distance(hit, aim);
            if (_precision > d)
            { break; }
            else if (new_distance > d)
            { continue; }
            else
            {
                /* FAIL */
                break;
            } // end else
        } // end else
        // -----------------------------------------------
        if (distance > new_distance)
        { distance = new_distance; }
        // -----------------------------------------------
        CDEBUG(_T("prec: ") << distance);
    } while (_precision < distance);
    // -----------------------------------------------
    tcout << _T("precision: ") << distance << std::endl;
    tcout << _T("grad_complex ") << gradient_complexity
          << std::endl << std::endl;
    // -----------------------------------------------
    return gradient_complexity;
}
//------------------------------------------------------------------------------

