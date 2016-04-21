
#if   HAND_VER == 1
#include "Hand.h"
using namespace OldHand;
#elif HAND_VER == 2
#include "Hand.h"
using namespace NewHand;
#include "HandMuscles.h"
#endif

#include "Position.h"  
#include "Store.h"
//------------------------------------------------------------------------------
namespace SimplexMethod
{
  extern "C"
  {
    int  calculate (IN  int nCols /* variables in the model */,
                    IN  int nRows,
                    IN  double** rows,
                    IN  double*  rights,
                    IN  double*  objectives,
                    OUT int* answer,
                    IN  int verbose);
  };
};
//------------------------------------------------------------------------------

// extern "C"
//{
#include "qr_solve.hpp"
//};

extern "C"
{
#define N_MUSCLES 4

  int  muscles_interpolate (IN  int m, int n,
                            IN  double *matrix,
                            IN  double *xes,
                            IN  double *yes,
                            IN  double *rights,
                            OUT int    *answers,
                            OUT double *xCoefs,
                            OUT double *yCoefs,
                            IN  int     verbose)
  {
    int result = 0;
    //======================================================
    double *x_coefs = qr_solve (m, n, matrix, xes);
    double *y_coefs = qr_solve (m, n, matrix, yes);
    //======================================================
    if ( !x_coefs || !y_coefs )
    {
      result = 1;
      goto RESULT;
    }

    for ( int j = 0; j < n; ++j )
      xCoefs[j] = x_coefs[j];
    for ( int j = 0; j < n; ++j )
      yCoefs[j] = y_coefs[j];

    if ( verbose )
    {
      printf ("\n\n");
      for ( int j = 0; j < n; ++j )
        printf ("%.4lf ", x_coefs[j]);
      printf ("\n");

      for ( int j = 0; j < n; ++j )
        printf ("%.4lf ", y_coefs[j]);
      printf ("\n\n");

      // --- checking ------------
      double sum_x = 0.;
      double sum_y = 0.;
      for ( int j = 0; j < n; ++j )
      {
        sum_x += matrix[j] * x_coefs[j];
        sum_y += matrix[j] * y_coefs[j];
      }

      printf ("%.4lf == %.4lf\n", sum_x, xes[0]);
      printf ("%.4lf == %.4lf\n", sum_y, yes[0]);
      printf ("\n");
    } // end if verbose
      //--------------------------------
    double *maxtix[] = { x_coefs, y_coefs };
    double objects[] = { 1., 1.,  1., 1.  };

    if ( SimplexMethod::calculate (n, 2, maxtix, rights, objects, answers, verbose) )
    {
      result = 1;
      goto RESULT;
    }

    if ( verbose )
    {
      double sum_x = 0.;
      double sum_y = 0.;
      for ( int j = 0; j < n; ++j )
      {
        sum_x += matrix[j] * answers[j];
        sum_y += matrix[j] * answers[j];
      }
      printf ("%lf == %lf\n", sum_x, rights[0]);
      printf ("%lf == %lf\n", sum_y, rights[1]);
    }

RESULT:;
    if ( x_coefs ) free (x_coefs);
    if ( y_coefs ) free (y_coefs);

    return result;
  }
};
//------------------------------------------------------------------------------
namespace Positions
{
  using namespace HandMoves;
  //------------------------------------------------------------------------------
  const int    LinearOperator::n_muscles  = 4;
  const double LinearOperator::normilizer = 150.;
  //------------------------------------------------------------------------------
  LinearOperator::LinearOperator (IN  HandMoves::Store &store,
                                  IN  const Point &aim,
                                  IN  double radius,
                                  OUT HandMoves::controling_t &controls,
                                  IN  bool verbose) throw (...):
    min_ (Point (aim.x - radius, aim.y - radius)),
    max_ (Point (aim.x + radius, aim.y + radius))
  {

    HandMoves::adjacency_t     range;
    store.adjacencyRectPoints<adjacency_t, ByP> (range, min_, max_);

    if ( range.size () > 2 )
    { // throw new std::exception ("solveQR: no points in adjacency"); }

      int  m = static_cast<int> (range.size ());
      int  n = LinearOperator::n_muscles;
      
      std::vector<double>  matrix (m * n);
      std::vector<double>  xes (m);
      std::vector<double>  yes (m);

      int i = 0, j = 0;
      for ( auto &rec : range )
      {
        xes[i] = rec.hit.x * LinearOperator::normilizer;
        yes[i] = rec.hit.y * LinearOperator::normilizer;
        
        if ( verbose )
        { tcout << _T ("controls: ") << rec.controls.size () << std::endl; }

        j = 0;
        for ( const Hand::Control &control : rec.controls )
        {
               if ( control.muscle & Hand::ShldrOpn )
          {
            matrix[i * n + LSO] = static_cast<double> (control.last);
            if ( verbose )
            { tcout << _T ("so ") << control.last << ' ' << matrix[i * n + LSO] << std::endl; }
          }
          else if ( control.muscle & Hand::ShldrCls )
          {
            matrix[i * n + LSC] = static_cast<double> (control.last);
            if ( verbose )
            { tcout << _T ("sc ") << control.last << ' ' << matrix[i * n + LSC] << std::endl; }
          }
          else if ( control.muscle & Hand::ElbowOpn )
          {
            matrix[i * n + LEO] = static_cast<double> (control.last);
            if ( verbose )
            { tcout << _T ("eo ") << control.last << ' ' << matrix[i * n + LEO] << std::endl; }
          }
          else if ( control.muscle & Hand::ElbowCls )
          {
            matrix[i * n + LEC] = static_cast<double> (control.last);
            if ( verbose )
            { tcout << _T ("ec ") << control.last << ' ' << matrix[i * n + LEC] << std::endl; }
          }

          ++j;
        }

        if ( verbose )
        { tcout << std::endl; }

        ++i;
      }

      if ( verbose )
      {
        auto old_prec = tcout.precision (4);

        for ( int j = 0; j < n; ++j )
        { tcout << _T ("\t") << j << _T ("\t"); }
        tcout << _T (" | x  \t| y") << std::endl;

        for ( int i = 0; i < m; ++i )
        {
          for ( int j = 0; j < n; ++j )
            tcout << matrix[i * n + j] << _T (" ");
            // printf ("%.4lf ", matrix[i * n + j]);
          // printf (" | %.4lf | %.4lf\n", xes[i], yes[i]);
          tcout << _T (" | ") << xes[i] << _T (" | ") << yes[i] << std::endl;
        }
        tcout.precision (old_prec);
      }
      
      int   solution[LinearOperator::n_muscles];
      double  rights[] = { aim.x * 100., aim.y * 100. };

      xCoefs.resize (LinearOperator::n_muscles);
      yCoefs.resize (LinearOperator::n_muscles);

      if ( !muscles_interpolate (m, n, matrix.data (), xes.data (), yes.data (),
                                 rights, solution, xCoefs.data (), yCoefs.data (),
                                 static_cast<int> (verbose)) )

      {
        // for ( int i = 0; i < 4; ++i )
        //   if ( solution[i] < 0 ) throw std::exception ("!!!");

        createJointControl (controls, solution, LSO, LSC, Hand::ShldrOpn, Hand::ShldrCls);
        createJointControl (controls, solution, LEO, LEC, Hand::ElbowOpn, Hand::ElbowCls);
      }
    } // end if
  }
  //------------------------------------------------------------------------------
  void LinearOperator::createJointControl (IN HandMoves::controling_t &controls,
                                           IN int *solution, IN size_t opn, IN size_t cls,
                                           IN Hand::MusclesEnum Opn, IN Hand::MusclesEnum Cls)
  {
    if ( solution[opn] && solution[cls] )
    {
      if ( solution[opn] > solution[cls] )
      {
        controls.push_back (Hand::Control (Opn, 0U, solution[opn]));
        controls.push_back (Hand::Control (Cls, solution[opn] + 1U, solution[cls]));
      }
      else
      {
        controls.push_back (Hand::Control (Opn, 0U, solution[cls]));
        controls.push_back (Hand::Control (Cls, solution[cls] + 1U, solution[opn]));
      }
    }
    else if ( solution[opn] )
    { controls.push_back (Hand::Control (Opn, 0U, solution[opn])); }
    else if ( solution[cls] )
    { controls.push_back (Hand::Control (Cls, 0U, solution[cls])); }
  }
  //------------------------------------------------------------------------------
  void  LinearOperator::predict (IN  const Point &aim,
                                 OUT HandMoves::controling_t &controls, 
                                 IN  bool verbose)
  {
    if ( min_.x <= aim.x && min_.y <= aim.y
      && max_.x >= aim.x && max_.y >= aim.y )
    {
      int   solution[LinearOperator::n_muscles];
      double objects[] = { 1., 1., 1., 1. };
      double  rights[] = { aim.x * LinearOperator::normilizer,
                           aim.y * LinearOperator::normilizer };
      double *maxtix[] = { xCoefs.data (), yCoefs.data () };

      if ( !SimplexMethod::calculate (LinearOperator::n_muscles, 2, maxtix, 
                                      rights, objects, solution, static_cast<int> (verbose)) )
      {
        createJointControl (controls, solution, LSO, LSC, Hand::ShldrOpn, Hand::ShldrCls);
        createJointControl (controls, solution, LEO, LEC, Hand::ElbowOpn, Hand::ElbowCls);

        // objects[0] = 2;
      } // end if
    } // end if
  }
  //------------------------------------------------------------------------------
};
//------------------------------------------------------------------------------