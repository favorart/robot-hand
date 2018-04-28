#include "Robo.h"
#include "RoboMovesStore.h"
#include "RoboPosApprox.h"


using namespace Eigen;
using namespace RoboPos;
using namespace Robo;

VectorXd RoboPos::Approx::convertRow(const Robo::Control &controls) const
{
    VectorXd res(_max_controls_count * Approx::control_size, Robo::MInvalid);

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

double RoboPos::Approx::noize(int i) const { return 0.5; }

void RoboPos::Approx::constructXY(const RoboMoves::Store &store)
{
    int i = 0;
    for (auto &rec : store)
    {
        _mX.row(i) = convertRow(rec.controls);
        _mY.row(i) = Eigen::Vector2d(rec.hit.x, rec.hit.y);
        ++i;
    }

    constructXY();
}


double fuck(double c)
{
    return double(std::log(c) - 50);
}


void RoboPos::Approx::constructXY()
{
    _vNorm = (_mX.rowwise().maxCoeff() - _mX.rowwise().minCoeff()) * sqrt(double(_mX.cols()));
    _vNorm = _vNorm.unaryExpr([](double c) { return (c == 0) ? 1. : 1. / c; });
    //std::cout << "vNorm=" << std::endl << _vNorm << std::endl;

    _mX = _vNorm.asDiagonal() * _mX;
    //std::cout << "mX=" << std::endl << _mX << std::endl;

    _mk = _mX.colwise().squaredNorm();
    //std::cout << "_mk=" << std::endl << _mk << std::endl;

    MatrixXd mk(_mk.rows(), _mk.rows());
    for (auto i : boost::irange(0, _mk.rows()))
        for (auto j : boost::irange(0, _mk.rows()))
            mk(i, j) = _mk[i] + _mk[j];
    std::cout << "_mk=" << std::endl << _mk << std::endl;

    auto md = MatrixXd{ mk - 2. * _mX.transpose() * _mX };
    //std::cout << "md=" << std::endl << md << std::endl;

    auto mt = MatrixXd{ md.unaryExpr([](double c) { return (c <= 0); }) };
    //std::cout << "mt=" << std::endl << mt << std::endl;

    for (auto i : boost::irange(0, md.cols()))
        for (auto j : boost::irange(0, md.rows()))
            if (mt(j, i))
                md(j,i) = 1.;
    //std::cout << "md=" << std::endl << md << std::endl;

    md = md.unaryExpr([t=_t](double c) { return c * (std::log(c) - t); });

    for (auto i : boost::irange(0, md.cols()))
        for (auto j : boost::irange(0, md.rows()))
            if (i == j)
                md(i, i) = noize();
            else if (mt(j, i))
                md(j, i) = 0.;
    //std::cout << "md=" << std::endl << md << std::endl;

    MatrixXd mA(md.rows() + 1, md.cols() + 1);
    mA << md, MatrixXd::Ones(md.rows(), 1),
          MatrixXd::Ones(1, md.cols() + 1);
    mA(md.rows(), md.cols()) = 0;
    //std::cout << "mA=" << std::endl << mA << std::endl;

    //ob = np.linalg.inv(md)
    //mY = np.vstack(( rez.transpose(), np.zeros((1,rez.shape[0])) ))
    //mQ = np.linalg.solve(md, mY)

    MatrixXd mY(_mY.cols() + 1, _mY.rows());
    mY << _mY.transpose(), MatrixXd::Zero(1, _mY.rows());

    //std::cout << "mY= " << std::endl << mY << std::endl;
    _mQ = mA.colPivHouseholderQr().solve(mY);
    //std::cout << "mQ= " << std::endl << _mQ << std::endl;
    _constructed = true;
}

Eigen::MatrixXd RoboPos::Approx::predict(Eigen::MatrixXd &X) const
{
    std::cout << "X=" << std::endl << X << std::endl;
    auto tx = _vNorm.asDiagonal() * X;
    std::cout << "tx=" << std::endl << tx << std::endl;

    auto this_mk = tx.colwise().squaredNorm();
    std::cout << "this_mk=" << std::endl << this_mk << std::endl;
    std::cout << "_mk=" << std::endl << _mk << std::endl;

    auto n = std::max(this_mk.rows(), this_mk.cols());
    MatrixXd mk(_mk.rows(), n);
    for (auto i : boost::irange(0, _mk.rows()))
        for (auto j : boost::irange(0, n))
            mk(i, j) = _mk[i] + this_mk[j];
    std::cout << "mk=" << std::endl << mk << std::endl;
    
    auto md = MatrixXd{ mk - 2. * tx.transpose() * tx };
    std::cout << "md=" << std::endl << md << std::endl;

    auto mt = MatrixXd{ md.unaryExpr([](double c) { return (c <= 0); }) };
    std::cout << "mt=" << std::endl << mt << std::endl;

    for (auto i : boost::irange(0, md.cols()))
        for (auto j : boost::irange(0, md.rows()))
            if (mt(j, i))
                md(j, i) = 1.;
    std::cout << "md=" << std::endl << md << std::endl;

    md = md.unaryExpr([t = _t](double c) { return c * (std::log(c) - t); });

    for (auto i : boost::irange(0, md.cols()))
        for (auto j : boost::irange(0, md.rows()))
            if (i == j)
                md(i, i) = noize();
            else if (mt(j, i))
                md(j, i) = 0.;
    std::cout << "md=" << std::endl << md << std::endl;

    MatrixXd mA(md.rows(), md.cols() + 1);
    mA << md, MatrixXd::Ones(md.rows(), 1);

    mA *= _mQ;
    mA.transposeInPlace();

    std::cout << "var= " << std::endl << mA << std::endl;
    return mA;
}

Point RoboPos::Approx::predict(Eigen::VectorXd &v) const
{
    //return { y_function(x, _mQ.col(0)), y_function(x,  _mQ.col(1)) };
    return Point{};
}

Point RoboPos::Approx::predict(const Robo::Control &controls) const
{
    VectorXd x = convertRow(controls);
    return predict(x);
}

bool RoboPos::Approx::clarify(const Robo::Control &controls, Point hit)
{
    VectorXd x = convertRow(controls);
    Vector2d y = { hit.x, hit.y };
    return clarify(x, y);
}

bool RoboPos::Approx::clarify(const Eigen::VectorXd &x, const Eigen::VectorXd &y)
{
    /// ???
    return false;
}

bool RoboPos::Approx::solveQ()
{
    if (!_constructed)
    {
        CWARN("solveQ() of non-constructed Approx");
        return false;
    }

    try
    {
        MatrixXd mA(_mX.rows(), _mX.rows());
        
        for (auto i : boost::irange(0, _mX.rows()))
            for (auto j : boost::irange(0, _mX.rows()))
                mA.row(i)[j] = (i == j) ? /* K_function(_mX.row(i) - _mX.row(j)) + */ noize() :
                                             K_function(_mX.row(i) - _mX.row(j));
        
        _mQ.col(0) = mA.colPivHouseholderQr().solve(_mY.col(0));
        _mQ.col(1) = mA.colPivHouseholderQr().solve(_mY.col(1));
        return true;
    }
    catch (const std::exception& e)
    {
        CERROR(e.what());
        _mQ.setZero();
        return false;
    }
}

