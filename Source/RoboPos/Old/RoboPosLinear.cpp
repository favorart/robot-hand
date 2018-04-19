#include "StdAfx.h"
//#include "qr_solve.hpp"
//
//#include "RoboPos.h"  
//#include "RoboMovesStore.h"
////------------------------------------------------------------------------------
////extern "C" {
//#define N_MUSCLES 4
//
//int SimplexMethod(IN  int nCols /* variables in the model */,
//                  IN  int nRows,
//                  IN  double** rows,
//                  IN  double*  rights,
//                  IN  double*  objectives,
//                  OUT int* answer,
//                  IN  int verbose);
////------------------------------------------------------------------------------
//int muscles_interpolate(IN  int m, int n,
//                        IN  double *matrix,
//                        IN  double *xes,
//                        IN  double *yes,
//                        IN  double *rights,
//                        OUT int    *answers,
//                        OUT double *xCoefs,
//                        OUT double *yCoefs,
//                        IN  int     verbose)
//{
//    int result = 0;
//    //======================================================
//    double *x_coefs = qr_solve(m, n, matrix, xes);
//    double *y_coefs = qr_solve(m, n, matrix, yes);
//    //======================================================
//    if (!x_coefs || !y_coefs)
//    {
//        result = 1;
//        goto RESULT;
//    }
//
//    for (int j = 0; j < n; ++j)
//        xCoefs[j] = x_coefs[j];
//    for (int j = 0; j < n; ++j)
//        yCoefs[j] = y_coefs[j];
//
//    if (verbose)
//    {
//        printf("\n\n");
//        for (int j = 0; j < n; ++j)
//            printf("%.4lf ", x_coefs[j]);
//        printf("\n");
//
//        for (int j = 0; j < n; ++j)
//            printf("%.4lf ", y_coefs[j]);
//        printf("\n\n");
//
//        // --- checking ------------
//        double sum_x = 0.;
//        double sum_y = 0.;
//        for (int j = 0; j < n; ++j)
//        {
//            sum_x += matrix[j] * x_coefs[j];
//            sum_y += matrix[j] * y_coefs[j];
//        }
//
//        printf("%.4lf == %.4lf\n", sum_x, xes[0]);
//        printf("%.4lf == %.4lf\n", sum_y, yes[0]);
//        printf("\n");
//    } // end if verbose
//      //--------------------------------
//    double *maxtix[] = { x_coefs, y_coefs };
//    double objects[] = { 1., 1.,  1., 1. };
//
//    if (SimplexMethod(n, 2, maxtix, rights, objects, answers, verbose))
//    {
//        result = 1;
//        goto RESULT;
//    }
//
//    if (verbose)
//    {
//        double sum_x = 0.;
//        double sum_y = 0.;
//        for (int j = 0; j < n; ++j)
//        {
//            sum_x += matrix[j] * answers[j];
//            sum_y += matrix[j] * answers[j];
//        }
//        printf("%lf == %lf\n", sum_x, rights[0]);
//        printf("%lf == %lf\n", sum_y, rights[1]);
//    }
//
//RESULT:;
//    if (x_coefs) free(x_coefs);
//    if (y_coefs) free(y_coefs);
//
//    return result;
//}
////}
//
//
//using namespace Robo;
//using namespace RoboPos;
//using namespace RoboMoves;
////------------------------------------------------------------------------------
//const double RoboPos::LinearOperator::normilizer = 150.;
////------------------------------------------------------------------------------
//RoboPos::LinearOperator::LinearOperator(IN RoboMoves::Store &store, IN  const Point &aim,
//                                        IN double radius, IN unsigned n_muscles) :
//    min_(Point(aim.x - radius, aim.y - radius)),
//    max_(Point(aim.x + radius, aim.y + radius)),
//    n_muscles(n_muscles)
//{
//    adjacency_t range;
//    store.adjacencyRectPoints<adjacency_t, ByP>(range, min_, max_);
//
//    if (range.size() > 2)
//    { // throw std::exception ("solveQR: no points in adjacency"); }
//
//        int  m = static_cast<int> (range.size());
//        int  n = LinearOperator::n_muscles;
//
//        std::vector<double>  matrix(m * n);
//        std::vector<double>  xes(m);
//        std::vector<double>  yes(m);
//
//        int i = 0, j = 0;
//        for (auto &rec : range)
//        {
//            xes[i] = rec.hit.x * LinearOperator::normilizer;
//            yes[i] = rec.hit.y * LinearOperator::normilizer;
//
//            if (verbose)
//            { tcout << _T("controls: ") << rec.controls.size() << std::endl; }
//
//            j = 0;
//            for (const Control &control : rec.controls)
//            {
//                ///if (control.muscle == Hand::ShldrOpn)
//                ///{
//                ///    matrix[i * n + LSO] = static_cast<double> (control.last);
//                ///    if (verbose)
//                ///    { tcout << _T("so ") << control.last << ' ' << matrix[i * n + LSO] << std::endl; }
//                ///}
//                ///else if (control.muscle & Hand::ShldrCls)
//                ///{
//                ///    matrix[i * n + LSC] = static_cast<double> (control.last);
//                ///    if (verbose)
//                ///    { tcout << _T("sc ") << control.last << ' ' << matrix[i * n + LSC] << std::endl; }
//                ///}
//                ///else if (control.muscle & Hand::ElbowOpn)
//                ///{
//                ///    matrix[i * n + LEO] = static_cast<double> (control.last);
//                ///    if (verbose)
//                ///    { tcout << _T("eo ") << control.last << ' ' << matrix[i * n + LEO] << std::endl; }
//                ///}
//                ///else if (control.muscle & Hand::ElbowCls)
//                ///{
//                ///    matrix[i * n + LEC] = static_cast<double> (control.last);
//                ///    if (verbose)
//                ///    { tcout << _T("ec ") << control.last << ' ' << matrix[i * n + LEC] << std::endl; }
//                ///}
//
//                ++j;
//            }
//
//            if (verbose)
//            { tcout << std::endl; }
//
//            ++i;
//        }
//
//        if (verbose)
//        {
//            auto old_prec = tcout.precision(4);
//
//            for (int j = 0; j < n; ++j)
//            { tcout << _T("\t") << j << _T("\t"); }
//            tcout << _T(" | x  \t| y") << std::endl;
//
//            for (int i = 0; i < m; ++i)
//            {
//                for (int j = 0; j < n; ++j)
//                    tcout << matrix[i * n + j] << _T(" ");
//                // printf ("%.4lf ", matrix[i * n + j]);
//              // printf (" | %.4lf | %.4lf\n", xes[i], yes[i]);
//                tcout << _T(" | ") << xes[i] << _T(" | ") << yes[i] << std::endl;
//            }
//            tcout.precision(old_prec);
//        }
//
//        int   solution[LinearOperator::n_muscles];
//        double  rights[] = { aim.x * 100., aim.y * 100. };
//
//        xCoefs.resize(LinearOperator::n_muscles);
//        yCoefs.resize(LinearOperator::n_muscles);
//
//        if (!muscles_interpolate(m, n, matrix.data(), xes.data(), yes.data(),
//                                 rights, solution, xCoefs.data(), yCoefs.data(),
//                                 static_cast<int> (verbose)))
//
//        {
//            // for ( int i = 0; i < 4; ++i )
//            //   if ( solution[i] < 0 ) throw std::exception ("!!!");
//
//            ///createJointControl(controls, solution, LSO, LSC, Hand::ShldrOpn, Hand::ShldrCls);
//            ///createJointControl(controls, solution, LEO, LEC, Hand::ElbowOpn, Hand::ElbowCls);
//        }
//    } // end if
//}
////------------------------------------------------------------------------------
//void RoboPos::LinearOperator::jointControl(OUT Robo::Control & controls, IN joint_t joint, IN const int *solution)
//{
//    muscle_t mOpn = RoboI::muscleByJoint(joint, true);
//    muscle_t mCls = RoboI::muscleByJoint(joint, false);
//
//    if (solution[mOpn] && solution[mCls])
//    {
//        if (solution[mOpn] > solution[mCls])
//        {
//            controls.append({ mOpn, 0U, frames_t(solution[mOpn]) });
//            controls.append({ mCls, frames_t(solution[mOpn] + 1U), frames_t(solution[mCls]) });
//        }
//        else
//        {
//            controls.append({ mOpn, 0U, frames_t(solution[mCls]) });
//            controls.append({ mCls, frames_t(solution[mCls] + 1U), frames_t(solution[mOpn]) });
//        }
//    }
//    else if (solution[mOpn])
//    { controls.append({ mOpn, 0U, frames_t(solution[mOpn]) }); }
//    else if (solution[mCls])
//    { controls.append({ mCls, 0U, frames_t(solution[mCls]) }); }
//}
////------------------------------------------------------------------------------
//Robo::Control RoboPos::LinearOperator::predict(IN const Point &aim, IN bool verbose)
//{
//    if (min_.x <= aim.x && min_.y <= aim.y
//     && max_.x >= aim.x && max_.y >= aim.y)
//    {
//        std::vector<int> solution(n_muscles);
//        std::vector<double> objects(n_muscles, 1.);
//        double  rights[] = { aim.x * LinearOperator::normilizer,
//                             aim.y * LinearOperator::normilizer };
//        double *maxtix[] = { xCoefs.data(), yCoefs.data() };
//
//        Robo::Control controls;
//        if (!SimplexMethod(n_muscles, 2u, maxtix, rights, &objects[0], &solution[0], static_cast<int>(verbose)))
//            for(joint_t j = 0; j < n_muscles / 2; ++j)
//                jointControl(controls, j, &solution[0]);
//        return controls;
//    }
//    throw std::runtime_error("Invalid LinearOperator");
//}
////------------------------------------------------------------------------------