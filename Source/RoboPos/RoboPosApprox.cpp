#include "RoboPosApprox.h"
#include "RoboMovesRecord.h"

using namespace Eigen;
using namespace RoboPos;
using namespace Robo;

//------------------------------------------------------------------------------
RoboPos::Approx::Approx(size_t store_size, size_t max_n_controls, Noize noize, Sizing sizing) :
    _max_n_controls(max_n_controls),
    _mX(store_size, Approx::control_size * _max_n_controls),
    _mY(store_size, Approx::point_size),
    _mQ(store_size, Approx::point_size),
    _vNorm(store_size),
    _vK(store_size),
    _nmX(Approx::control_size * _max_n_controls, store_size),
    _noize(noize), _sizing(sizing)
{}

//------------------------------------------------------------------------------
RoboPos::Approx::Approx(Eigen::MatrixXd &X, Eigen::MatrixXd &Y) :
    _max_n_controls(X.cols() / Approx::control_size),
    _mX(X),
    _mY(Y),
    _mQ(X.rows(), Approx::point_size),
    _vNorm(X.rows()),
    _vK(X.rows()),
    _nmX(Approx::control_size * _max_n_controls, X.rows()),
    _noize(noize), _sizing(sizing)
{
    if ((X.cols() % Approx::control_size) > 0 ||
        X.rows() != Y.rows() ||
        Y.cols() != Approx::point_size)
        throw std::runtime_error{ "Invalid Controls Matrix" };
    constructXY();
}

//------------------------------------------------------------------------------
void RoboPos::Approx::resize(size_t store_size, size_t max_n_controls)
{
    CINFO(" sz="  << store_size << " nc=" << _max_n_controls <<
          " _mX=" << _mX.rows() << "x" << _mX.cols() <<
          " _mY=" << _mY.rows() << "x" << _mY.cols());

    if (size_t(_mX.rows()) != store_size || size_t(_mX.cols()) != (Approx::control_size * max_n_controls))
        _mX.resize(store_size, Approx::control_size * max_n_controls);
    if (size_t(_mY.rows()) != store_size)
        _mY.resize(store_size, Approx::point_size);
    if (size_t(_mQ.rows()) != store_size)
        _mQ.resize(store_size, Approx::point_size);
    if (size_t(_vNorm.size()) != store_size)
        _vNorm.resize(store_size);
    if (size_t(_vK.size()) != store_size)
        _vK.resize(store_size);
    if (size_t(_nmX.rows()) != (Approx::control_size * max_n_controls) || size_t(_nmX.cols()) != store_size)
        _nmX.resize(Approx::control_size * max_n_controls, store_size);
    _constructed = false;
}

//------------------------------------------------------------------------------
void RoboPos::Approx::chg_params(Noize noize, Sizing sizing)
{
    _noize = noize;
    _sizing = sizing;
    _constructed = false;
    _train = false;
}


//------------------------------------------------------------------------------
RowVectorXd RoboPos::Approx::convertToRow(const Robo::Control &controls) const
{
    if (controls.size() > _max_n_controls)
        CERROR("convertToRow: controls " << controls.size() << " is too long " << _max_n_controls);

    RowVectorXd res(_max_n_controls * Approx::control_size);
    res.fill(double(Robo::MInvalid));

    int j = 0;
    muscle_t m = 0;
    frames_t prev_start = Robo::LastsInfinity; // -1;
    frames_t prev_lasts = Robo::LastsInfinity; // -1;
    for (auto &a : controls)
    {
        // если границы движения совпадают у нескольких мускулов,
        // объединить их в одно действие
        if ((a.start == prev_start) && (a.lasts == prev_lasts))
        {
            m |= (1 << a.muscle);
            prev_start = a.start;
            prev_lasts = a.lasts;

            res[j + 0] = double(m);
            res[j + 1] = double(a.start);
            res[j + 2] = double(a.lasts);
        }
        else
        {
            m = (1 << a.muscle);
            prev_start = a.start;
            prev_lasts = a.lasts;

            res[j + 0] = double(1 << a.muscle);
            res[j + 1] = double(a.start);
            res[j + 2] = double(a.lasts);
        }
        j += Approx::control_size;
    }
    return res;
}

//------------------------------------------------------------------------------
void RoboPos::Approx::insert(const Robo::Control &controls, Point hit, size_t index)
{
    _mX.row(index) = convertToRow(controls);
    _mY.row(index) = Vector2d(hit.x, hit.y);
    _constructed = false;
}

//------------------------------------------------------------------------------
void RoboPos::Approx::clear()
{
    _constructed = false;
    _train = false;
    //_max_n_controls;
    _mX = {}; _mY = {}; _mQ = {};
    _vNorm = {};
    _vK = {};
    _nmX = {};
    //_noize;
    //_sizing;
}


//------------------------------------------------------------------------------
Eigen::MatrixXd RoboPos::Approx::calcKFunction(Eigen::MatrixXd &X) const
{
    VectorXd vK = X.colwise().squaredNorm();
    //std::cout << "vK=" << std::endl << vK << std::endl;
    //std::cout << "_vK=" << std::endl << _vK << std::endl;

    auto vK_rows_train = _vK.rows();
    auto vK_rows_prdct = vK.size();

    MatrixXd mK(vK_rows_prdct, vK_rows_train);
    for (auto i : boost::irange<frames_t>(0, vK_rows_prdct))
        for (auto j : boost::irange<frames_t>(0, vK_rows_train))
            mK(i, j) = _vK[j] + vK[i];
    //std::cout << "mK=" << mK.rows() << ", " << mK.cols() << std::endl; // << mK << std::endl;
    //std::cout << "_nmX=" << _nmX.row(0) << std::endl;

    auto mD = MatrixXd{ mK - 2. * X.transpose() * _nmX };
    //std::cout << "mD1=" << mD.col(0) << std::endl;
    //std::cout << "mD=" << std::endl << mD << std::endl;
    auto mI = MatrixXd{ mD.unaryExpr([](double c) { return (c <= 0); }) };
    //std::cout << "mI=" << std::endl << mI << std::endl;

    for (auto i : boost::irange<frames_t>(0, mD.rows()))
        for (auto j : boost::irange<frames_t>(0, mD.cols()))
            if (mI(i, j))
                mD(i, j) = 1.;
    //std::cout << "mD=" << std::endl << mD << std::endl;

    mD = mD.unaryExpr([t=_sizing()](double c) { return c * (std::log(c) + t); });

    for (auto i : boost::irange<frames_t>(0, mD.rows()))
        for (auto j : boost::irange<frames_t>(0, mD.cols()))
            if (i == j && _train)
                mD(i, i) = _noize(i);
            else if (mI(i, j))
                mD(i, j) = 0.;
    //std::cout << "mD=" << std::endl << mD << std::endl;
    //std::cout << "mD2=" << mD.col(0) << std::endl;
    return mD;
}

//------------------------------------------------------------------------------
void RoboPos::Approx::constructXY()
{
    if (!_mX.rows() || !_mX.cols() || !_mY.rows() || !_mY.cols())
        CERROR("Approx does not applied train data to construct");

    CINFO("matix X=" << _mX.rows() << "x" << _mX.cols() << " Y=" << _mY.rows() << "x" << _mY.cols());

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


//------------------------------------------------------------------------------
Eigen::MatrixXd RoboPos::Approx::predict(Eigen::MatrixXd &X) const
{
    if (!_constructed)
        throw std::runtime_error{ "Approx does not constructed train data to predict"};
    if (size_t(X.cols()) != _max_n_controls * Approx::control_size)
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

//------------------------------------------------------------------------------
Point RoboPos::Approx::predict(Eigen::RowVectorXd &v) const
{
    MatrixXd mX(1, v.size());
    mX << v;
    //std::cout << "mX= " << mX.rows() << ", " << mX.cols() << std::endl << mX << std::endl;
    MatrixXd mA = predict(mX);
    //std::cout << "mA= " << mA.rows() << ", " << mA.cols() << std::endl; // << mA << std::endl;
    //return { y_function(x, _mQ.col(0)), y_function(x,  _mQ.col(1)) };
    return Point{ mA(0,0), mA(0,1) };
}

//------------------------------------------------------------------------------
Point RoboPos::Approx::predict(const Robo::Control &controls) const
{
    RowVectorXd x = convertToRow(controls);
    return predict(x);
}


//------------------------------------------------------------------------------
bool RoboPos::Approx::clarify(const Robo::Control &controls, Point hit)
{
    RowVectorXd x = convertToRow(controls);
    RowVector2d y = { hit.x, hit.y };
    return clarify(x, y);
}

//------------------------------------------------------------------------------
bool RoboPos::Approx::clarify(const Eigen::RowVectorXd &x, const Eigen::RowVector2d &y)
{
    /// ??? https://www.encyclopediaofmath.org/index.php/Sequential_approximation
    return false;
}
