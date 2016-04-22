#include "StdAfx.h"

#ifndef  _LIN_OPER_H_
#define  _LIN_OPER_H_

#include "WindowHeader.h"
#include "target.h"
#include "Store.h"

//------------------------------------------------------------------------------
namespace Positions
{
  class LinearOperator
  {
    Point  min_, max_;
    std::vector<double>  xCoefs, yCoefs;

    static const int     n_muscles;
    static const double normilizer;

    const int  LSO = 0;
    const int  LSC = 1;
    const int  LEO = 2;
    const int  LEC = 3;

    //------------------------------------------------------------------------------
    void createJointControl (IN HandMoves::controling_t &controls,
                             IN int *solution, IN size_t opn, IN size_t cls,
                             IN Hand::MusclesEnum Opn, IN Hand::MusclesEnum Cls);

  public:
    //------------------------------------------------------------------------------
    LinearOperator () {}
    LinearOperator (IN  HandMoves::Store &store,
                    IN  const Point &aim,
                    IN  double radius,
                    OUT HandMoves::controling_t &controls,
                    IN  bool verbose = false) throw (...);

    void  predict (IN  const Point &aim,
                   OUT HandMoves::controling_t &controls,
                   IN  bool verbose = false);
  };
};
//------------------------------------------------------------------------------
#endif // _LIN_OPER_H_
