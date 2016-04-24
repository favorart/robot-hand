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
        double  d = d_d / (last_mo_l + last_mo_g);
        // direction_o = ((d_d * int_normalizer) / (last_mo_l + last_mo_g));
        direction_o = d * int_normalizer;

        if ( direction_o == 0 )
        {
          // BACKWARD MOVEMENT
          if ( d < 0. )
          { ++direction_c; /* = (direction_o) ? (direction_o + 1) : 50; */ }
          else
          { --direction_o; /* = (direction_c < 0) ? -1 : 1; */ }
        }
      }

      if ( last_mc_l || last_mc_g )
      {
        double d = d_d / (last_mc_l + last_mc_g);
        // direction_c = ((d_d * int_normalizer) / (last_mc_l + last_mc_g));
        direction_c = d * int_normalizer;

        if ( direction_c == 0 )
        {
          // BACKWARD MOVEMENT
          if ( d < 0. )
          { ++direction_o; /* = (direction_o) ? (direction_o + 1) : 50; */ }
          else
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

      Hand::frames_t start_o = (last_o > last_c) ? 0U : (last_c + 1U);
      Hand::frames_t start_c = (last_o < last_c) ? 0U : (last_o + 1U);

      if ( last_o ) { controls.insert (iter, Hand::Control (mo, start_o, last_o)); }
      if ( last_c ) { controls.insert (iter, Hand::Control (mc, start_c, last_c)); }
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
    
    int iter = 0;
    // -----------------------------------------------
    do
    {
      // -----------------------------------------------
      Record        rec;
      controling_t  lower_controls, upper_controls;
      double        lower_distance, upper_distance;
      // -----------------------------------------------
      if ( !w_meansULAdjs (aim, &rec,
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
      if ( new_distance <= d )
      {
        // controling_t  controls;
        gradient_complexity += w_means (aim, /* controls, hand_position,*/ verbose);

        d = boost_distance (hand_position, aim);
        if ( precision > d )
        { break; }
        else if ( new_distance > d )
        { continue; }
        else // ( new_distance <= d )
        {
          // controls.clear ();
          gradient_complexity += rundown (aim, /*controls,*/ hand_position, verbose);

          d = boost_distance (hand_position, aim);
          if ( precision > d )
          { break; }
          else if ( new_distance > d )
          { continue; }
          else if ( distance == new_distance )
          { 
            // gradient_complexity += rundownMethod (aim, /*controls, hand_position,*/ verbose);
            //  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            //  d = boost_distance (hand_position, aim);
            //  if ( precision > d )
            //  { break; }
            //  else if ( new_distance > d )
            //  { continue; }
            ++iter;
            if ( iter > 4 )
              break;
          }
        }
      }
      else { new_distance = d; }
      // -----------------------------------------------
      controling_t  controls;
      gradientControls (aim, d_d,
                        inits_controls,
                        lower_controls,
                        upper_controls,
                        controls);
      // -----------------------------------------------
      hand_act (aim, controls, hand_position);
      ++gradient_complexity;
      // -----------------------------------------------
      d = boost_distance (hand_position, aim);
      // -----------------------------------------------
      if ( new_distance > d )
      { new_distance = d; }
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
  typedef std::function<bool (const Record &, const Point &)> func_t;
  Record*  GradientMin (IN const HandMoves::adjacency_refs_t &range, IN const Point &aim,
                        IN OUT std::set<size_t> *visited = NULL, IN const func_t *func = NULL)
  {
    Record *rmin = NULL;
    double  dr, dm;

    size_t h;
    for ( auto &rec : range )
    {
      if ( visited )
      {
        RecordHasher rh;
        h = rh (*rec);
      }

      dr = boost_distance (rec->hit, aim);
      if ( (!visited || visited->find (h) == visited->end ())
          && (!func || (*func) (*rec, aim))
          && (!rmin || dr < dm) )
      {
        rmin = &(*rec);
        dm = dr;
      }
    }
    return  rmin;
  };
  //------------------------------------------------------------------------------
  bool  ClothestRecords (IN HandMoves::Store &store, IN const Point &aim,
                         OUT Record *rec0, OUT Record *rec1, OUT Record *rec2,
                         IN std::set<size_t> *visited = NULL)
  {
    if ( !rec0 )
    { return  false; }
    // ------------------------------------------------
    Point  min (aim.x - 0.1, aim.y - 0.1),
      max (aim.x + 0.1, aim.y + 0.1);
    // ------------------------------------------------
    adjacency_refs_t range;
    store.adjacencyRectPoints<adjacency_refs_t, ByP> (range, min, max);
    // ------------------------------------------------
    func_t  cmp_l = [](const Record &p, const Point &aim)
    { return  (p.hit.x < aim.x) && (p.hit.y < aim.y); };
    func_t  cmp_g = [](const Record &p, const Point &aim)
    { return  (p.hit.x > aim.x) && (p.hit.y > aim.y); };
    // ------------------------------------------------
    Record *pRec = GradientMin (range, aim, visited, NULL);
    if ( !pRec ) { return false; }

    RecordHasher rh;
    size_t h = rh (*pRec);
    visited->insert (h);
    // ===========
    *rec0 = *pRec;
    // ===========
    // ------------------------------------------------
    if ( rec1 && rec2 )
    {
      // ------------------------------------------------
      pRec = GradientMin (range, aim, visited, &cmp_l);
      if ( !pRec ) { return false; }
      // ===========
      *rec1 = *pRec;
      // ===========
      // ------------------------------------------------
      pRec = GradientMin (range, aim, visited, &cmp_g);
      if ( !pRec ) { return false; }
      // ===========
      *rec2 = *pRec;
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
    hand.SET_DEFAULT;
    Point hand_pos_base = hand.position;
    // -----------------------------------------------
    Point hand_position; // = rec.hit;

    std::set<size_t>  visited;
    // -----------------------------------------------
    double distance = boost_distance (hand_pos_base, aim);
    double new_distance = 0.;
    while ( precision < distance )
    {
      Record  rec0, rec_lower, rec_upper;
      if ( !ClothestRecords (store, aim, &rec0, &rec_lower, &rec_upper, &visited) )
      { return gradient_complexity; }

      new_distance = boost_distance (rec0.hit, aim);
      if ( new_distance < distance )
        distance = new_distance;

      HandMoves::controling_t  inits_controls = rec0.controls;
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
      hand_act (aim, controls, hand_position);
      ++gradient_complexity;
      // -----------------------------------------------
      double d = boost_distance (hand_position, aim);
      if ( d > side ) //== new_distance )
      {
        visited.clear ();

        Record rec0;
        ClothestRecords (store, aim, &rec0, NULL, NULL, &visited);
        if ( rec0.controls.size () )
        {
          Point hand_position = rec0.hit; // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
          rundownMethod (aim /*, rec0.controls, hand_position*/);
          d = boost_distance (hand_position, aim);
        }

        if ( d == new_distance )
        { break; }
      }

      if ( new_distance > 0.1 )
      {
        visited.clear ();
        // continue;
      }

      if ( new_distance > distance && d > 0.2 ) // new_distance )
      { break; }

      new_distance = d;
      if ( distance > new_distance )
      { distance = new_distance; }
      // -----------------------------------------------
#ifdef _DEBUG_PRINT
      tcout << _T ("prec: ") << best_distance << std::endl;
#endif // _DEBUG_PRINT
    } // end while
    
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
  //------------------------------------------------------------------------------
};
