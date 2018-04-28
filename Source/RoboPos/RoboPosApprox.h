#pragma once

//#include "LAPACK"
//#include <cvm.h>
#include <Eigen/Eigen>
#include "EigenSerializePlugin.h"

#include "Robo.h"
#include "RoboMovesStore.h"


namespace RoboPos
{
//using Pt = std::pair<double, double>;
//using Row = std::vector<double>;
//Row substract(const Row &a, const Row &b) const
//{
//    if (a.size() != b.size())
//        throw std::logic_error("Invalid Row");
//
//    Row res(a.size());
//    for (auto i = 0u; i < a.size(); ++i)
//    {
//        res[i] = a[i] - b[i];
//    }
//    return res;
//}
//double sq_norm(const Row &row) const
//{
//    double res = 0;
//    for (double x : row)
//        res += x * x;
//    return res;
//}


class Approx
{
public:
    static const size_t control_size = 3;
    static const size_t point_size = 2;
    static const int MLN = 1000000;
    
    const double _t = 50. /*10 * MLN*/, _n = 50. /*10 * MLN*/;
    const double _c_k = 17; // калибровочный коэффициент
    bool _constructed = false;

    const size_t _max_controls_count;
    // X.rows() = Y.rows() = store.size()
    // X.cols() = control_size * _max_controls_count;
    // Y.cols() = point_size;
    Eigen::MatrixXd _mX, _mY, _mQ;
    // интерполируемая функция
    // X - aргументы функции,
    // Y - значения функции
    Eigen::VectorXd _vNorm, _mk;

    double noize(int i = 0) const;
    Eigen::VectorXd convertRow(const Robo::Control&) const;

    double K_function(const Eigen::VectorXd &x) const
    {
        double sq_norm = x.squaredNorm();
        return _c_k * (sq_norm * std::log(sq_norm / _t) + _n);
    }
    double y_function(const Eigen::VectorXd &x, const Eigen::VectorXd &q) const
    {
        double res = 0;
        for (auto i = 0; i < _mX.rows(); ++i)
            res += q[i] * K_function(x - _mX.row(i));
        return res;
    }
    
    Approx() = delete;
    Approx(size_t store_size, size_t max_controls_count) :
        _max_controls_count(max_controls_count),
        _mX(store_size, Approx::control_size * _max_controls_count),
        _mY(store_size, Approx::point_size),
        _mQ(store_size, store_size),
        _vNorm(Approx::control_size * _max_controls_count),
        _mk(Approx::control_size * _max_controls_count),
        _constructed(false)
    {}
    Approx(Eigen::MatrixXd &X, Eigen::MatrixXd &Y) :
        _max_controls_count(X.rows()),
        _mX(X),
        _mY(Y),
        _mQ(X.rows(), X.rows()),
        _vNorm(Approx::control_size * _max_controls_count),
        _mk(Approx::control_size * _max_controls_count),
        _constructed(false)
    { constructXY(); }

    Eigen::MatrixXd predict(Eigen::MatrixXd&) const;

    Point predict(Eigen::VectorXd&) const;
    Point predict(const Robo::Control&) const;
    bool  clarify(const Robo::Control&, Point);
    bool  clarify(const Eigen::VectorXd &x, const Eigen::VectorXd &y);

    void constructXY();
    void constructXY(const RoboMoves::Store &store);
    bool solveQ();

    template <class Archive>
    void serialize(Archive &ar, unsigned version)
    { ar & _mX & _mY & _mq & _max_controls_count; }
};
}
