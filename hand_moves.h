#include "StdAfx.h"

#ifdef  _HAND_MOVES_H_
// #define  _HAND_MOVES_H_
// 
// #include "hand.h"
// #include "target.h"
// #include "my_window.h"
// #include "hand_status.h"
// 
// 
// //------------------------------------------------------------------------------
// typedef class HandMoves  hm;
// typedef std::list<hs>    hm_list;
// class HandMoves
// { //---aggregation------------------------------
//   Target* tg_;
// 		bool    sync;
// 		void    Sync () {}
// 
//   //--------------------------------------------
//   struct cmpPointPtrsByDistance
//   { friend class Point;
//     bool operator() (const Point* ptg, const Point* p) const
//     { 
//      return ( !ptg->Hit (*p,1e-3) && *ptg < *p );
//     }
//   };
// 
//   typedef std::multimap<const Point*, hm_list,
//    cmpPointPtrsByDistance>  hm_map;
//   //--------------------------------------------
//   hm_map  hmoves; // n*m * attemps_quantity
// 
// public:
// 		explicit HandMoves (Target *T);
// 
// 		HandStatus* Get (const Point      *aim);
// 		void        Set (const HandStatus &hs ); // Point* == Point* !!!
// 		void        Set (const Point  &hand,
// 		                 uint_t   count_moves,
// 	                   bool   control [hs::maxMovesCount][Hand::hydEnginesCount],
// 										 ulong_t  time  [hs::maxMovesCount] );
//     void        Set (const Point& hand, uint_t count_moves,
//                      Hand::MusclesEnum  hyd[hs::maxMovesCount],
//                      ulong_t              last[hs::maxMovesCount]);
// 
// 		void  Dump () const;
// 		void  Gain ();
// 		//--------------------------------------------
// 		friend class HandStatus;
// 		friend class Target;
// };
// //------------------------------------------------------------------------------
// void  test_cover  (hm &Hm, Hand &H, double radius);
// void  test_random (hm &Hm, Hand &H, uint_t  tries);
#endif // _HAND_MOVES_H_
