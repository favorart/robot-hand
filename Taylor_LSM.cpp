
#if   HAND_VER == 1
#include "Hand.h"
using namespace OldHand;
#elif HAND_VER == 2
#include "NewHand.h"
using namespace NewHand;
#include "HandMuscles.h"
#endif

//------------------------------------------------------------------------------
// #include "MyWindow.h"
// #include "target.h"
#include "HandMovesStore.h"

#include "llsq.hpp"
#include "qr_solve.hpp"



namespace Positions
{
#define N_MSCL_LASTS 4
#define N_MSCL_START 2

  using namespace HandMoves;

  typedef std::array<double, (N_MSCL_LASTS + N_MSCL_START)> lin_op_t;

#define LSO 0
#define LSC 1
#define LEO 2
#define LEC 3

#define BSO 4
#define BEO 5

  void  LinearOperator::solveQR1 (HandMoves::Store &store, const Point &aim, double side) throw (...)
  {
    HandMoves::adjacency_t     range;
    store.adjacencyRectPoints (range, Point (aim.x - side, aim.y - side),
                                      Point (aim.x + side, aim.y + side));

    if ( range.size () > 2 )
    { // throw new std::exception ("solveQR: no points in adjacency"); }

      int m = (int) range.size ();
      int n = (N_MSCL_LASTS + N_MSCL_START);

      std::vector<double> A (m*n);
      std::vector<double> B (m);
      std::vector<double> C (m);

      int i = 0;
      for ( auto &rec : range )
      {
        B[i] = rec.hit.y;
        C[i] = rec.hit.x;

        int j = 0;
        printf ("controls: %d\n", (int) rec.controls ().size ());
        for ( const Hand::Control &control : rec.controls () )
        {
          if ( control.muscle & Hand::ShldrOpn )
          {
            A[i * n + LSO] = (double) control.last;
            tcout << _T ("so ") << control.last << ' ' << A[i * n + LSO] << std::endl;
          }
          else if ( control.muscle & Hand::ShldrCls )
          {
            A[i * n + LSC] = (double) control.last;
            tcout << _T ("sc ") << control.last << ' ' << A[i * n + LSC] << std::endl;
          }
          else if ( control.muscle & Hand::ElbowOpn )
          {
            A[i * n + LEO] = (double) control.last;
            tcout << _T ("eo ") << control.last << ' ' << A[i * n + LEO] << std::endl;
          }
          else if ( control.muscle & Hand::ElbowCls )
          {
            A[i * n + LEC] = (double) control.last;
            tcout << _T ("ec ") << control.last << ' ' << A[i * n + LEC] << std::endl;
          }
          ++j;
        }
        tcout << std::endl;
        ++i;
      }

      for ( int i = 0; i < m; ++i )
      {
        for ( int j = 0; j < n; ++j )
          printf ("%.4lf ", A[i * n + j]);
        // tcout << A[i * n + j] << _T (" ");
        printf (" | %.4lf\n", B[i]);
        // tcout << _T (" | ") << B[i] << std::endl;
      }

      double *coefs1 = qr_solve (m, n, A.data (), C.data ());
      double *coefs2 = qr_solve (m, n, A.data (), B.data ());

      tcout << std::endl << std::endl;

      for ( int j = 0; j < n; ++j )
        printf ("%.4lf ", coefs1[j]);
      printf ("\n\n");

      for ( int j = 0; j < n; ++j )
        printf ("%.4lf ", coefs2[j]);
      printf ("\n");

      std::vector<double> cc (2 * n);
      for ( int i = 0; i < n; ++i )
        cc[i] = coefs1[i];
      for ( int i = 0; i < n; ++i )
        cc[n + i] = coefs2[i];


      double x[] = { aim.x, aim.y };

      // double *coefs_ = qr_solve (m, n, cc.data (), x);

      double sum = 0.;
      for ( int j = 0; j < n; ++j )
        sum = A[j] * coefs1[j];
      tcout << sum << ' ' << C[0] << '\n';


      delete[] coefs1;
      delete[] coefs2;

      // for ( int j = 0; j < n; ++j )
      //   printf ("%d ", (int) coefs_[j]);
      // printf ("\n");

      int solution[4];
      for ( int i = 0; i < 4; ++i )
        if ( solution[i] < 0 )
          throw new std::exception ("!!!");

      controling_t cntrls;
      auto createJointControl = [](controling_t &controls, int *solution, size_t opn, size_t cls)
      {
        if ( solution[LSO] && solution[LSC] )
        {
          if ( solution[LSO] > solution[LSC] )
          {
            controls.push_back (Hand::Control (Hand::ShldrOpn, 0U, solution[opn]));
            controls.push_back (Hand::Control (Hand::ShldrCls, solution[opn] + 1U, solution[cls]));
          }
          else
          {
            controls.push_back (Hand::Control (Hand::ShldrOpn, 0U, solution[cls]));
            controls.push_back (Hand::Control (Hand::ShldrCls, solution[cls] + 1U, solution[opn]));
          }
        }
        else if ( solution[LSO] )
        { controls.push_back (Hand::Control (Hand::ShldrOpn, 0U, solution[opn])); }
        else if ( solution[LSC] )
        { controls.push_back (Hand::Control (Hand::ShldrOpn, 0U, solution[cls])); }
      };

      createJointControl (cntrls, solution, LSO, LSC);
      createJointControl (cntrls, solution, LEO, LEC);

      // double *T = new double[];
    }
  }
  void  LinearOperator::solveQR (HandMoves::Store &store, const Point &aim, double side) throw (...)
  {
    HandMoves::adjacency_t  range;
    store.adjacencyRectPoints (range, Point (aim.x - side, aim.y - side),
                                      Point (aim.x + side, aim.y + side));
  
    if ( range.size () > 2 )
    { // throw new std::exception ("solveQR: no points in adjacency"); }

      int m = (int) range.size ();
      int n = (N_MSCL_LASTS + N_MSCL_START);

      std::vector<double> A (m*n);
      std::vector<double> B (m);
      std::vector<double> C (m);
      // double *A = new double[m * n];
      // double *B = new double[m];

      int i = 0;
      for ( auto &rec : range )
      {
        B[i] = rec.hit.y;
        //A[i * n + 6] = rec.hit.x;
        C[i] = rec.hit.x;

        int j = 0;
        printf ("controls: %d\n", (int) rec.controls ().size ());
        for ( const Hand::Control &control : rec.controls () )
        {
          if ( control.muscle & Hand::ShldrOpn )
          {
            A[i * n + LSO] = (double) control.last;
            tcout << _T ("so ") << control.last << ' ' << A[i * n + LSO] << std::endl;

            if ( !control.start )
            { // so = true;
              A[i * n + BSO] = true;
              tcout << _T ("bso ") << true << ' ' << A[i * n + 4] << std::endl;

            }
          }
          else if ( control.muscle & Hand::ShldrCls )
          {
            A[i * n + LSC] = (double) control.last;
            tcout << _T ("sc ") << control.last << ' ' << A[i * n + LSC] << std::endl;

            if ( !control.start )
            { // sc = false;
              A[i * n + BSO] = false;
              tcout << _T ("bsc ") << false << ' ' << A[i * n + 4] << std::endl;

            }
          }
          else if ( control.muscle & Hand::ElbowOpn )
          {
            A[i * n + LEO] = (double) control.last;
            tcout << _T ("eo ") << control.last << ' ' << A[i * n + LEO] << std::endl;

            if ( !control.start )
            { // eo = true;
              A[i * n + BEO] = true;
              tcout << _T ("beo ") << true << ' ' << A[i * n + 5] << std::endl;

            }
          }
          else if ( control.muscle & Hand::ElbowCls )
          {
            A[i * n + LEC] = (double) control.last;
            tcout << _T ("ec ") << control.last << ' ' << A[i * n + LEC] << std::endl;

            if ( !control.start )
            { // ec = false;
              A[i * n + BEO] = false;
              tcout << _T ("bec ") << false << ' ' << A[i * n + 5] << std::endl;

            }
          }

          ++j;
        }
        ++i;
      }

      for ( int i = 0; i < m; ++i )
      {
        for ( int j = 0; j < n; ++j )
          printf ("%.4lf ", A[i * n + j]);
        // tcout << A[i * n + j] << _T (" ");
        printf (" | %.4lf\n", B[i]);
        // tcout << _T (" | ") << B[i] << std::endl;
      }
      double *coefs1 = qr_solve (m, n, A.data (), C.data ());
      double *coefs2 = qr_solve (m, n, A.data (), B.data ());

      tcout << std::endl << std::endl;

      for ( int j = 0; j < n; ++j )
        printf ("%.4lf ", coefs1[j]);
      printf ("\n\n");

      for ( int j = 0; j < n; ++j )
        printf ("%.4lf ", coefs2[j]);
      printf ("\n");

      std::vector<double> cc (2 * n);
      for ( int i = 0; i < n; ++i )
        cc[i] = coefs1[i];
      for ( int i = 0; i < n; ++i )
        cc[n + i] = coefs2[i];


      double x[] = { aim.x, aim.y };

      // double *coefs_ = qr_solve (m, n, cc.data (), x);

      double sum = 0.;
      for ( int j = 0; j < n; ++j )
        sum = A[j] * coefs1[j];
      tcout << sum << ' ' << C[0] << '\n';


      delete[] coefs1;
      delete[] coefs2;

      // for ( int j = 0; j < n; ++j )
      //   printf ("%d ", (int) coefs_[j]);
      // printf ("\n");

      // int min_so = 
      //   int min_eo = 
      //   int min_
      // for ( int i = )


      //controling_t cntrls;
      //if ( coefs_[BSO] > 0.5 )
      //{
      //  cntrls.push_back (Hand::Control (Hand::ShldrOpn, 0U, coefs_[LSO]));
      //  if ( coefs_[LSC] > EPS )
      //    cntrls.push_back (Hand::Control (Hand::ShldrCls, coefs_[LSO] + 1U, coefs_[LSC]));
      //}
      //else
      //{
      //  cntrls.push_back (Hand::Control (Hand::ShldrCls, 0U, coefs_[LSC]));
      //  if ( coefs_[LSC] > EPS )
      //    cntrls.push_back (Hand::Control (Hand::ShldrOpn, coefs_[LSC] + 1U, coefs_[LSO]));
      //}

      //if ( coefs_[BSO] > 0.5 )
      //{
      //  cntrls.push_back (Hand::Control (Hand::ElbowOpn, 0U, coefs_[LEO]));
      //  if ( coefs_[LSC] > EPS )
      //    cntrls.push_back (Hand::Control (Hand::ElbowCls, coefs_[LEO] + 1U, coefs_[LEC]));
      //}
      //else
      //{
      //  cntrls.push_back (Hand::Control (Hand::ElbowCls, 0U, coefs_[LEC]));
      //  if ( coefs_[LSC] > EPS )
      //    cntrls.push_back (Hand::Control (Hand::ElbowOpn, coefs_[LEC] + 1U, coefs_[LEO]));
      //}

      //// double *T = new double[];
    }
  }

  void  constructLinearOperator (Hand::frames_t last_so, Hand::frames_t last_sc,
                                 Hand::frames_t last_eo, Hand::frames_t last_ec,
                                 bool eo,
                                 bool so,
                                 lin_op_t LinOp)
  {
    /* int N, the number of data values */
    const int n = (N_MSCL_LASTS + N_MSCL_START);
    const int m = 2;

    double x[n];
    double y[n];
    double a;
    double b;

    /* LLSQ solves a linear least squares problem matching a line to data */
    llsq (n, x, y, a, b);
    /* A formula for a line of the form Y = A * X + B is sought, which           */
    /* will minimize the root-mean-square error to N data points ( X[I], Y[I] ); */
    
    //    Input, double X[N], Y[N], the coordinates of the data points.
    //    Output, double &A, &B, the slope and Y-intercept of the least-squares
    //    approximant to the data.

    double A[2 * n];
    double B[] = { 0., 0. };
    /* QR_SOLVE solves a linear system in the least squares sense */
    double* C = qr_solve (m, n, A, B);

    //****************************************************************************80
    //
    //    If the matrix A has full column rank, then the solution X should be the
    //    unique vector that minimizes the Euclidean norm of the residual.
    //
    //    If the matrix A does not have full column rank, then the solution is
    //    not unique; the vector X will minimize the residual norm, but so will
    //    various other vectors.
    //
    //  Parameters:
    //
    //    Input, int M, the number of rows of A.
    //
    //    Input, int N, the number of columns of A.
    //
    //    Input, double A[M*N], the matrix.
    //
    //    Input, double B[M], the right hand side.
    //
    //    Output, double QR_SOLVE[N], the least squares solution.

    return;
  }

};
//------------------------------------------------------------------------------