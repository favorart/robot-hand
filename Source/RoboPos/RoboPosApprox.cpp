#include "Robo.h"
#include "RoboMovesStore.h"
#include "RoboPosApprox.h"


using namespace Eigen;
using namespace RoboPos;
using namespace Robo;

VectorXd RoboPos::Approx::convertToRow(const Robo::Control &controls) const
{
    VectorXd res(_max_controls_count * Approx::control_size);
    res.fill(Robo::MInvalid);

    for (auto &a : controls)
    {
        auto j = a.muscle * Approx::control_size;
        res[j + 0] = double(a.muscle);
        res[j + 1] = double(a.start);
        res[j + 2] = double(a.lasts);
    }
    for (Robo::muscle_t m = 0; m < _max_controls_count; ++m)
    {
        auto j = m * Approx::control_size;
        if (res[j] == Robo::MInvalid)
        {
            for (auto i = j; i < j + Approx::control_size; ++i)
                res[i] = 0.;
        }
    }
    return res;
}

void RoboPos::Approx::constructXY(const RoboMoves::Store &store)
{
    int i = 0;
    for (auto &rec : store)
    {
        _mX.row(i) = convertToRow(rec.controls);
        _mY.row(i) = Eigen::Vector2d(rec.hit.x, rec.hit.y);
        ++i;
    }

    constructXY();
}


Eigen::MatrixXd RoboPos::Approx::calcKFunction(Eigen::MatrixXd &X) const
{
    VectorXd vK = X.colwise().squaredNorm();
    //std::cout << "vK=" << std::endl << vK << std::endl;
    //std::cout << "_vK=" << std::endl << _vK << std::endl;

    auto vK_rows_train = _vK.rows();
    auto vK_rows_prdct = vK.size();

    MatrixXd mK(vK_rows_prdct, vK_rows_train);
    for (auto i : boost::irange(0, vK_rows_prdct))
        for (auto j : boost::irange(0, vK_rows_train))
            mK(i, j) = _vK[j] + vK[i];
    //std::cout << "mK=" << mK.rows() << ", " << mK.cols() << std::endl; // << mK << std::endl;
    //std::cout << "_nmX=" << _nmX.row(0) << std::endl;

    auto mD = MatrixXd{ mK - 2. * X.transpose() * _nmX };
    //std::cout << "mD1=" << mD.col(0) << std::endl;
    //std::cout << "mD=" << std::endl << mD << std::endl;
    auto mI = MatrixXd{ mD.unaryExpr([](double c) { return (c <= 0); }) };
    //std::cout << "mI=" << std::endl << mI << std::endl;

    for (auto i : boost::irange(0, mD.rows()))
        for (auto j : boost::irange(0, mD.cols()))
            if (mI(i, j))
                mD(i, j) = 1.;
    //std::cout << "mD=" << std::endl << mD << std::endl;

    mD = mD.unaryExpr([t=_sizing()](double c) { return c * (std::log(c) + t); });

    for (auto i : boost::irange(0, mD.rows()))
        for (auto j : boost::irange(0, mD.cols()))
            if (i == j && _train)
                mD(i, i) = _noize(i);
            else if (mI(i, j))
                mD(i, j) = 0.;
    //std::cout << "mD=" << std::endl << mD << std::endl;
    //std::cout << "mD2=" << mD.col(0) << std::endl;
    return mD;
}

void RoboPos::Approx::constructXY()
{
    if (!_mX.rows() || !_mX.cols() || !_mY.rows() || !_mY.cols())
        throw std::runtime_error{ "Approx does not applied train data to construct" };

    _train = true;
    _vNorm = (_mX.colwise().maxCoeff() - _mX.colwise().minCoeff()) * sqrt(double(_mX.rows()));
    _vNorm = _vNorm.unaryExpr([](double c) { return (c == 0) ? 1. : 1. / c; });
    //std::cout << "vNorm=" << std::endl << _vNorm << std::endl;

    _nmX = _vNorm.asDiagonal() * _mX.transpose();
    //std::cout << "nmX=" << nmX.rows() << ", " << nmX.cols() << std::endl << nmX << std::endl;
    _vK = _nmX.colwise().squaredNorm();
    //std::cout << "_vK=" << _vK.rows() << ", " << _vK.cols() << std::endl << _vK << std::endl;

    MatrixXd mD = calcKFunction(_nmX);

    MatrixXd mA(mD.rows() + 1, mD.cols() + 1);
    mA << mD, MatrixXd::Ones(mD.rows(), 1), MatrixXd::Ones(1, mD.cols() + 1);
    mA(mD.rows(), mD.cols()) = 0;
    //std::cout << "mA=" << mA.rows() << ", " << mA.cols() << std::endl;// << mA << std::endl;
    //----------------------------------------------
    boost::this_thread::interruption_point();
    //----------------------------------------------
    MatrixXd mY(_mY.rows() + 1, _mY.cols());
    mY << _mY, MatrixXd::Ones(1, _mY.cols());
    //std::cout << "mY= " << mY.rows() << ", " << mY.cols() << std::endl;// << _mY << std::endl;
    _mQ = mA.colPivHouseholderQr().solve(mY);
    //std::cout << "mQ= " << _mQ.rows() << ", " << _mQ.cols() << std::endl;// << _mQ << std::endl;
    _constructed = true;
    _train = false;
}

Eigen::MatrixXd RoboPos::Approx::predict(Eigen::MatrixXd &X) const
{
    if (!_constructed)
        throw std::runtime_error{ "Approx does not constructed train data to predict"};
    if (X.cols() != _max_controls_count * Approx::control_size)
        throw std::runtime_error{ "Invalid predict data" };

    //std::cout << "Norm= " << std::endl << _vNorm << std::endl;
    MatrixXd nmX = _vNorm.asDiagonal() * X.transpose();
    //std::cout << "nmX= " << std::endl << nmX.transpose() << std::endl;
    MatrixXd mD = calcKFunction(nmX);

    MatrixXd mA(mD.rows(), mD.cols() + 1);
    mA << mD, MatrixXd::Ones(mD.rows(), 1);
    //std::cout << "mA= " << /*mA.rows() << ", " << mA.cols() << std::endl <<*/ mA.col(0) << std::endl;
    mA *= _mQ;
    //std::cout << "mA= " << mA.rows() << ", " << mA.cols() << std::endl << mA << std::endl;
    return mA;
}


Point RoboPos::Approx::predict(Eigen::VectorXd &v) const
{
    MatrixXd mX(1, v.size());
    mX << v;
    //std::cout << "mX= " << mX.rows() << ", " << mX.cols() << std::endl << mX << std::endl;
    MatrixXd mA = predict(mX);
    //std::cout << "mA= " << mA.rows() << ", " << mA.cols() << std::endl; // << mA << std::endl;
    //return { y_function(x, _mQ.col(0)), y_function(x,  _mQ.col(1)) };
    return Point{ mA(0,0), mA(0,1) };
}

Point RoboPos::Approx::predict(const Robo::Control &controls) const
{
    VectorXd x = convertToRow(controls);
    return predict(x);
}

bool RoboPos::Approx::clarify(const Robo::Control &controls, Point hit)
{
    VectorXd x = convertToRow(controls);
    Vector2d y = { hit.x, hit.y };
    return clarify(x, y);
}

bool RoboPos::Approx::clarify(const Eigen::VectorXd &x, const Eigen::Vector2d &y)
{
    /// ??? https://www.encyclopediaofmath.org/index.php/Sequential_approximation
    return false;
}
