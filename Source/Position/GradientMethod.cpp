#include "StdAfx.h"
#include "Position.h"

namespace Positions
{
  using namespace std;
  using namespace HandMoves;
  //------------------------------------------------------------------------------
  void  LearnMovements::gradientControls (IN  const Point &aim, IN  double  d_d,
                                          IN  const HandMoves::controling_t &inits_controls,
                                          IN  const HandMoves::controling_t &lower_controls,
                                          IN  const HandMoves::controling_t &upper_controls,
                                          OUT       HandMoves::controling_t &controls)
  {
    auto  iter = controls.begin ();
    // -----------------------------------------------
    for ( auto j : hand.joints_ )
    {
      auto mo = muscleByJoint (j, true);
      auto mc = muscleByJoint (j, false);

      auto it_mo_i = boost::range::find (inits_controls, mo);
      auto it_mo_l = boost::range::find (lower_controls, mo);
      auto it_mo_g = boost::range::find (upper_controls, mo);

      auto it_mc_i = boost::range::find (inits_controls, mc);
      auto it_mc_l = boost::range::find (lower_controls, mc);
      auto it_mc_g = boost::range::find (upper_controls, mc);

      int last_mo_i = (it_mo_i != inits_controls.end ()) ? (it_mo_i->last) : 0U;
      int last_mo_l = (it_mo_l != lower_controls.end ()) ? (it_mo_l->last - last_mo_i) : 0U;
      int last_mo_g = (it_mo_g != upper_controls.end ()) ? (it_mo_g->last - last_mo_i) : 0U;

      int last_mc_i = (it_mc_i != inits_controls.end ()) ? (it_mc_i->last) : 0U;
      int last_mc_l = (it_mc_l != lower_controls.end ()) ? (it_mc_l->last - last_mc_i) : 0U;
      int last_mc_g = (it_mc_g != upper_controls.end ()) ? (it_mc_g->last - last_mc_i) : 0U;
      // -----------------------------------------------
      const int  int_normalizer = (d_d / precision); /* velosity */ // 400;
      

      // -----------------------------------------------
      int  direction_o = 0U;
      int  direction_c = 0U;

      if ( last_mo_l || last_mo_g )
      {
        double  d = 0.;
        if ( last_mo_l + last_mo_g )
        {
          d = d_d / (last_mo_l + last_mo_g);
          // direction_o = ((d_d * int_normalizer) / (last_mo_l + last_mo_g));
          direction_o = d * int_normalizer;
        }

        if ( direction_o == 0 )
        {
          // BACKWARD MOVEMENT
          if ( d < 0. )
          { ++direction_c; /* = (direction_o) ? (direction_o + 1) : 50; */ }
          else if ( d > 0. )
          { --direction_o; /* = (direction_c < 0) ? -1 : 1; */ }
        }
      }

      if ( last_mc_l || last_mc_g )
      {
        double  d = 0.;
        if ( last_mc_l + last_mc_g )
        {
          d = d_d / (last_mc_l + last_mc_g);
          // direction_c = ((d_d * int_normalizer) / (last_mc_l + last_mc_g));
          direction_c = d * int_normalizer;
        }

        if ( direction_c == 0 )
        {
          // BACKWARD MOVEMENT
          if ( d < 0. )
          { ++direction_o; /* = (direction_o) ? (direction_o + 1) : 50; */ }
          else if ( d > 0.)
          { --direction_c; /* = (direction_c < 0) ? -1 : 1; */ }
        }
      }

      Hand::frames_t last_o = 0U;
      Hand::frames_t last_c = 0U;

      if ( direction_o < 0 || (direction_o >= 0 && last_mo_i > direction_o) )
      { last_o += (last_mo_i - direction_o); }
      else if ( (direction_o >= 0 && last_mo_i < direction_o) )
      {
        last_o = 0U;
        last_c += (last_mo_i + (direction_o - last_mo_i));
      }

      if ( direction_c < 0 || (direction_c >= 0 && last_mc_i > direction_c) )
      { last_c += (last_mc_i - direction_c); }
      else if ( (direction_c >= 0 && last_mc_i <= direction_c) )
      {
        last_c = 0U;
        last_o += (last_mc_i + (direction_c - last_mc_i));
      }

      if ( last_o != last_c )
      {
        Hand::frames_t start_o = (last_o > last_c) ? 0U : (last_c + 1U);
        Hand::frames_t start_c = (last_o < last_c) ? 0U : (last_o + 1U);

        if ( last_o ) { controls.insert (iter, Hand::Control (mo, start_o, last_o)); }
        if ( last_c ) { controls.insert (iter, Hand::Control (mc, start_c, last_c)); }
      }
    }

  }
  //------------------------------------------------------------------------------
  size_t  LearnMovements::gradientMethod_admixture (IN const Point &aim, IN bool verbose)
  {
    size_t  gradient_complexity = 0U;
    // -----------------------------------------------
    hand.SET_DEFAULT;
    Point hand_pos_base = hand.position;
    Point hand_position;
    // -----------------------------------------------
    double distance = boost_distance (hand_pos_base, aim),
      new_distance = distance;
    // -----------------------------------------------
    do
    {
      // -----------------------------------------------
      Record        rec;
      controling_t  lower_controls, upper_controls;
      double        lower_distance, upper_distance;
      // -----------------------------------------------
      if ( !weightedMeanULAdjs (aim, &rec,
                                lower_controls, upper_controls,
                                lower_distance, upper_distance) )
      { break; }
      // -----------------------------------------------
      const controling_t  &inits_controls = rec.controls;
      double  d_d = (upper_distance - lower_distance);
      // -----------------------------------------------
      double d = boost_distance (rec.hit, aim);
      if ( precision > d )
      { break; }
      // -----------------------------------------------
      if ( new_distance > d )
      { new_distance = d; }
      // -----------------------------------------------
      else
      {
        gradient_complexity += weightedMean (aim, hand_position, verbose);

        d = boost_distance (hand_position, aim);
        if ( precision > d )
        { break; }
        else if ( new_distance > d )
        { continue; }
        else
        {
          gradient_complexity += rundownMain (aim, hand_position, verbose);

          d = boost_distance (hand_position, aim);
          if ( precision > d )
          { break; }
          else if ( new_distance > d )
          { continue; }
          else
          { 
            gradient_complexity += gradientMethod (aim, verbose);
            
            auto &rec = store.ClothestPoint (aim, side);
            hand_position = rec.hit;
            
            d = boost_distance (hand_position, aim);
            if ( precision > d )
            { break; }
            else if ( new_distance > d )
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
      controling_t  controls;
      gradientControls (aim, d_d,
                        inits_controls,
                        lower_controls,
                        upper_controls,
                        controls);
      // -----------------------------------------------
      if ( handAct (aim, controls, hand_position) )
      { ++gradient_complexity; }
      // -----------------------------------------------
      d = boost_distance (hand_position, aim);
      // -----------------------------------------------
      if ( new_distance > d )
      { new_distance = d; }
      if ( d > side )
      { /* FAIL */
        break;
      }
      // -----------------------------------------------
      if ( distance > new_distance )
      { distance = new_distance; }
      // -----------------------------------------------
    } while ( precision < distance );
    // -----------------------------------------------
    if ( verbose )
    {
      tcout << _T ("precision: ") << distance << std::endl;
      tcout << _T ("gradient admix complexity: ")
            <<      gradient_complexity
            <<      std::endl << std::endl;
    }
    // -----------------------------------------------
    return  gradient_complexity;
  }
  //------------------------------------------------------------------------------
  const Record*  LearnMovements::gradientClothestRecord  (IN const HandMoves::adjacency_ptrs_t &range,
                                                          IN const  Point  &aim,
                                                          IN const  func_t *pPred,
                                                          IN OUT visited_t *pVisited)
  {
    const Record *pRecMin = NULL;
    // ------------------------
    size_t h;
    double  dr, dm;
    // ------------------------
    for ( auto pRec : range )
    // for ( const auto &pRec : range )
    {
      if ( pVisited )
      { RecordHasher rh;
        h = rh (*pRec);
      }
      // ------------------------
      dr = boost_distance (pRec->hit, aim);
      if ( (!pVisited || pVisited->find (h) == pVisited->end ())
        && (!pPred    || (*pPred) (*pRec, aim))
        && (!pRecMin  || dr < dm) )
      { pRecMin = pRec; // &(*pRec);
        dm = dr;
      }
    }
    // ------------------------
    return pRecMin;
  };
  //------------------------------------------------------------------------------
  bool     LearnMovements::gradientClothestRecords (IN  const Point &aim,
                                                    OUT HandMoves::Record *pRecClose,
                                                    OUT HandMoves::Record *pRecLower,
                                                    OUT HandMoves::Record *pRecUpper,
                                                    IN OUT visited_t      *pVisited)
  {
    if ( !pRecClose )  { return  false; }
    // ------------------------------------------------
    Point  min (aim.x - side, aim.y - side),
           max (aim.x + side, aim.y + side);
    // ------------------------------------------------
    adjacency_ptrs_t range;
    store.adjacencyRectPoints<adjacency_ptrs_t, ByP> (range, min, max);
    // ------------------------------------------------
    func_t  cmp_l = [](const Record &p, const Point &aim)
    { return  (p.hit.x < aim.x) && (p.hit.y < aim.y); };
    func_t  cmp_g = [](const Record &p, const Point &aim)
    { return  (p.hit.x > aim.x) && (p.hit.y > aim.y); };
    // ------------------------------------------------
    const Record *pRec = gradientClothestRecord (range, aim, NULL, pVisited);
    if ( !pRec ) { return false; }

    RecordHasher rh;
    size_t h = rh (*pRec);
    pVisited->insert (h);
    // ===========
    *pRecClose = *pRec;
    // ===========
    // ------------------------------------------------
    if ( pRecLower && pRecUpper )
    {
      // ------------------------------------------------
      pRec = gradientClothestRecord (range, aim, &cmp_l, pVisited);
      if ( !pRec ) { return false; }
      // ===========
      *pRecLower = *pRec;
      // ===========
      // ------------------------------------------------
      pRec = gradientClothestRecord (range, aim, &cmp_g, pVisited);
      if ( !pRec ) { return false; }
      // ===========
      *pRecUpper = *pRec;
      // ===========
      // ------------------------------------------------
    }
    // ------------------------------------------------
    return true;
  }
  //------------------------------------------------------------------------------
  size_t  LearnMovements::gradientMethod (IN const Point &aim, IN bool verbose)
  {
    size_t  gradient_complexity = 0U;
    // -----------------------------------------------

    std::set<size_t>  visited;
    // -----------------------------------------------
    double distance = boost_distance (hand_pos_base, aim);
    double new_distance = 0.;
    // -----------------------------------------------
    hand.SET_DEFAULT;
    do
    {
      Record  rec_close, rec_lower, rec_upper;
      if ( !gradientClothestRecords (aim, &rec_close,
                                          &rec_lower,
                                          &rec_upper,
                                          &visited) )
      { /* FAIL */
        break;
      }
      Point hand_position = rec_close.hit;

      new_distance = boost_distance (hand_position, aim);
      if ( new_distance < distance )
        distance = new_distance;

      if ( precision > new_distance ) { break; }

      HandMoves::controling_t  inits_controls{ rec_close.controls };
      // -----------------------------------------------
      double  lower_distance = boost_distance (aim, rec_lower.hit);
      double  upper_distance = boost_distance (aim, rec_upper.hit);

      double  d_d = (upper_distance - lower_distance);
      // -----------------------------------------------
      HandMoves::controling_t  controls;
      gradientControls (aim, d_d,
                        inits_controls,
                        rec_lower.controls,
                        rec_upper.controls,
                        controls);
      // -----------------------------------------------
      if ( handAct (aim, controls, hand_position) )
      { ++gradient_complexity; }
      // -----------------------------------------------
      double d = boost_distance (hand_position, aim);
      // -----------------------------------------------
      if ( precision > d )
      { break; }
      // -----------------------------------------------
      else if ( new_distance > d  )
      { new_distance = d; }
      // -----------------------------------------------
      else
      {
        visited.clear ();
        
        rundownFull (aim, hand_position, verbose);

        d = boost_distance (hand_position, aim);
        if ( precision > d )
        { break; }
        else if ( new_distance > d )
        { continue; }
        else
        {
          /* FAIL */
          break;
        } // end else
      } // end else
      // -----------------------------------------------
      if ( distance > new_distance )
      { distance = new_distance; }
      // -----------------------------------------------
#ifdef _DEBUG_PRINT
      tcout << _T ("prec: ") << best_distance << std::endl;
#endif // _DEBUG_PRINT

    } while ( precision < distance );
    // -----------------------------------------------
    if ( verbose )
    {
      tcout << _T ("precision: ") << distance << std::endl;
      tcout << _T ("gradient complexity: ")
            <<      gradient_complexity
            <<   std::endl << std::endl;
    }
    // -----------------------------------------------
    return gradient_complexity;
  }
};
//------------------------------------------------------------------------------

