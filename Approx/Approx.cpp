#include "RoboMovesStore.h"
#include "RoboPosApprox.h"

using namespace RoboMoves;
using namespace RoboPos;
using namespace Utils;
using namespace Eigen;


int main()
{
    std::cout << std::setprecision(3) << std::left;
    //Store store("Hand-v3-robo-moves-2018.04.26-10.10.bin");
    MatrixXd X(7,2);
    X <<
        0., 0.,
        0., 1.,
        1., 0.,
        1., 1.,
        0., 2.,
        2., 0.,
        2., 2.;
    MatrixXd Y(7, 2);
    Y <<
        0., +0.,
        1., +5.,
        1., -5.,
        0., +0.,
        1., +9.,
        1., -9.,
        0., +0.;

    Approx approx(X, Y);
    //approx.solveQ();

    //for (RowVectorXd::InnerIterator it(Y.rowwise()); it; ++it)
    //    approx.predict(*it);

    //Control c{};

    //tfstream ofs("approx.less.txt", std::ios_base::out);
    ////boost::archive::text_oarchive toa(ofs);
    ////toa & approx;
    //for (int i = 0 ; i < X.rows(); ++i)
    //{
    //    VectorXd x = X.row(i);
    //    Point p = approx.predict(x);
    //    VectorXd y = Y.row(i);
    //    ofs << y[0] << ' ' << y[1] << ' ' << p << std::endl;
    //}


    //double noize = 12.;
    //MatrixXd ma(X.rows(), X.rows()), mq(X.rows(), 2);
    //
    //for (auto i : boost::irange(0, X.rows()))
    //    for (auto j : boost::irange(0, X.rows()))
    //    {
    //        std::cout << X.row(i) << " - " << X.row(j) << " ==> " 
    //                  << ((i == j) ? noize : approx.K_function(X.row(i) - X.row(j))) << std::endl;
    //
    //        ma.row(i)[j] = ((i == j) ? noize : approx.K_function(X.row(i) - X.row(j)));
    //    }
    //
    //mq.col(0) = ma.colPivHouseholderQr().solve(Y.col(0));
    //mq.col(1) = ma.colPivHouseholderQr().solve(Y.col(1));
    //
    //std::cout << mq << std::endl << std::endl;
    //
    //
    //
    //for (int i = 0 ; i < X.rows(); ++i)
    //{
    //    VectorXd x = X.row(i);
    //    //Point p = approx.predict(x);
    //    VectorXd y = Y.row(i);
    //
    //    double y1 = 0, y2 = 0;
    //    for (auto i = 0; i < X.rows(); ++i)
    //    {
    //        std::cout << x << " - " << X.row(i) << " ==> "
    //            << approx.K_function(x - X.row(i)) << std::endl;
    //
    //        y1 += mq.col(0)[i] * approx.K_function(x - X.row(i));
    //        y2 += mq.col(1)[i] * approx.K_function(x - X.row(i));
    //    }
    //
    //    std::cout << y[0] << ' ' << y[1] << " = " << y1 << ' ' << y2 << std::endl;
    //        //p.x << ' ' << p.y << std::endl;
    //}

    //for (int i = 0; i < X.rows(); ++i)
    //{
    //    VectorXd x = X.row(i);
    //    Point p = approx.predict(x);
    //}
    
    approx.predict(X);

    MatrixXd pred(7, 2);
    pred <<
        0.0, 0.5,
        0.5, 1.0,
        0.5, 0.5,
        0.0, 1.5,
        1.5, 0.0,
        1.5, 1.5,
        1.5, 1.7;

    approx.predict(pred);

    //for (int i = 0; i < pred.rows(); ++i)
    //{
    //    VectorXd x = pred.row(i);
    //    Point p = approx.predict(x);
    //}

    //for (int i = 0; i < pred.rows(); ++i)
    //{
    //    VectorXd x = pred.row(i);
    //    //Point p = approx.predict(x);
    //    //VectorXd y = Y.row(i);
    //
    //    double y1 = 0, y2 = 0;
    //    for (auto i = 0; i < X.rows(); ++i)
    //    {
    //        std::cout << x << " - " << X.row(i) << " ==> "
    //            << approx.K_function(x - X.row(i)) << std::endl;
    //
    //        y1 += mq.col(0)[i] * approx.K_function(x - X.row(i));
    //        y2 += mq.col(1)[i] * approx.K_function(x - X.row(i));
    //    }
    //
    //    std::cout << " = " << y1 << ' ' << y2 << std::endl;
    //    //p.x << ' ' << p.y << std::endl;
    //}

    return 0;
}

