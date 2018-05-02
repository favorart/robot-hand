#pragma once

//#include <LAPACK>
//#include <cvm.h>
#include <Eigen/Eigen>
#include "EigenSerializePlugin.h"

#include "Robo.h"
#include "RoboMovesStore.h"


namespace RoboPos
{
/// Аппроксимирующая функция - точка остановы по управлениям
class Approx
{
    static const size_t control_size = 3; ///< muscle, start, lasts
    static const size_t point_size = 2; ///< x, y
    
    bool _constructed = false;

    const size_t _max_controls_count;
    /// интерполируемая функция: X - aргументы функции, Y - значения функции
    Eigen::MatrixXd _mX, _mY, _mQ;
    Eigen::VectorXd _vNorm, _vK;
    /// нормализованная транспонированная матрица обучающих управлений
    Eigen::MatrixXd _nmX;

    double sizing() const; ///< калибровочный коэффициент
    double noize(int i = 0) const; ///< шум
    Eigen::VectorXd convertRow(const Robo::Control&) const;

    double K_function(const Eigen::VectorXd &x) const
    {
        double sq_norm = x.squaredNorm();
        return /*_c_k * */(sq_norm * std::log(sq_norm / sizing()) /*+ _n*/);
    }
    double y_function(const Eigen::VectorXd &x, const Eigen::VectorXd &q) const
    {
        double res = 0;
        for (auto i = 0; i < _mX.rows(); ++i)
            res += q[i] * K_function(x - _mX.row(i));
        return res;
    }

    Eigen::MatrixXd calcKFunction(Eigen::MatrixXd &X, bool predict) const;

    void constructXY();
public:
    Approx() = delete;
    Approx(size_t store_size, size_t max_controls_count) :
        _max_controls_count(max_controls_count),
        _mX(store_size, Approx::control_size * _max_controls_count),
        _mY(store_size, Approx::point_size),
        _mQ(store_size, Approx::point_size),
        _vNorm(store_size),
        _vK(store_size),
        _constructed(false),
        _nmX(Approx::control_size * _max_controls_count, store_size)
    {}
    Approx(Eigen::MatrixXd &X, Eigen::MatrixXd &Y) :
        _max_controls_count(X.cols() / Approx::control_size),
        _mX(X),
        _mY(Y),
        _mQ(X.rows(), Approx::point_size),
        _vNorm(X.rows()),
        _vK(X.rows()),
        _constructed(false),
        _nmX(Approx::control_size * _max_controls_count, X.rows())
    { 
        if ((X.cols() % Approx::control_size) > 0 || 
             X.rows() != Y.rows() ||
             Y.cols() != Approx::point_size)
            throw std::runtime_error{ "Invalid Controls Matrix" };
        constructXY();
    }

    Eigen::MatrixXd predict(Eigen::MatrixXd&) const;

    Point predict(Eigen::VectorXd&) const;
    Point predict(const Robo::Control&) const;

    bool  clarify(const Robo::Control&, Point);
    bool  clarify(const Eigen::VectorXd &x, const Eigen::Vector2d &y);

    void constructXY(const RoboMoves::Store &store);

    template <class Archive>
    void serialize(Archive &ar, unsigned version)
    { ar & _mX & _mY & _mq & _max_controls_count; }
};
}
