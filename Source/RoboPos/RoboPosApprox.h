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
/// http://www.machinelearning.ru/wiki/index.php?title=Обучение_с_учителем._Многомерная_интерполяции_и_аппроксимация._Обобщение_на_основе_теории_случайных_функций._Вариант_точного_решения.
class Approx
{
    static const size_t control_size = 3; ///< muscle, start, lasts
    static const size_t point_size = 2; ///< x, y
    
    bool _constructed = false;
    bool _train = false;

    const size_t _max_controls_count;
    /// интерполируемая функция: X - aргументы функции, Y - значения функции
    Eigen::MatrixXd _mX, _mY, _mQ;
    Eigen::VectorXd _vNorm, _vK;
    /// нормализованная транспонированная матрица обучающих управлений
    Eigen::MatrixXd _nmX;
    
    std::function<double(size_t)> _noize;
    std::function<double()> _sizing;

    double K_function(const Eigen::VectorXd &x) const
    {
        double sq_norm = x.squaredNorm();
        return /* _vK* */(sq_norm * std::log(sq_norm) + _sizing());
    }
    double y_function(const Eigen::VectorXd &x, const Eigen::VectorXd &q) const
    {
        double res = 0;
        for (auto i = 0; i < _mX.rows(); ++i)
            res += q[i] * K_function(x - _mX.row(i));
        return res;
    }

    Eigen::MatrixXd calcKFunction(Eigen::MatrixXd &X) const;

    void constructXY();
public:
    bool constructed() const { return _constructed; }
    /// Calibrating factor (greater is more precise)
    static double sizing() { return 4.; }
    /// Шум - этот коэффициент сообщает, насколько можно доверять управлению с номером i
    static double noize(size_t i = 0) { return 0.00000001; }

    Approx() = delete;
    /// Construct from store
    Approx(size_t store_size, size_t max_controls_count,
           std::function<double(size_t)> noize = Approx::noize,
           std::function<double()> sizing = Approx::sizing) :
        _max_controls_count(max_controls_count),
        _mX(store_size, Approx::control_size * _max_controls_count),
        _mY(store_size, Approx::point_size),
        _mQ(store_size, Approx::point_size),
        _vNorm(store_size),
        _vK(store_size),
        _nmX(Approx::control_size * _max_controls_count, store_size),
        _noize(noize), _sizing(sizing)
    {}
    /// Construct from converted matrices
    Approx(Eigen::MatrixXd &X, Eigen::MatrixXd &Y) :
        _max_controls_count(X.cols() / Approx::control_size),
        _mX(X),
        _mY(Y),
        _mQ(X.rows(), Approx::point_size),
        _vNorm(X.rows()),
        _vK(X.rows()),
        _nmX(Approx::control_size * _max_controls_count, X.rows()),
        _noize(noize), _sizing(sizing)
    { 
        if ((X.cols() % Approx::control_size) > 0 || 
             X.rows() != Y.rows() ||
             Y.cols() != Approx::point_size)
            throw std::runtime_error{ "Invalid Controls Matrix" };
        constructXY();
    }
    /// Convert a raw controls of RoboI to an aligned row of doubles
    Eigen::VectorXd convertToRow(const Robo::Control&) const;
    /// Predict the end-point for a pack of controls at once
    /// \param[in]  XTest  matrix of aligned rows of doubles
    /// \return matrix, each row is a point (x,y)
    Eigen::MatrixXd predict(Eigen::MatrixXd&) const;
    /// Predict the end-point for an aligned row of doubles
    Point predict(Eigen::VectorXd&) const;
    /// Predict the end-point for a raw control
    Point predict(const Robo::Control&) const;
    /// Exact interpolation by one more point
    bool  clarify(const Robo::Control&, Point);
    /// Exact interpolation by one more point converted
    bool  clarify(const Eigen::VectorXd&, const Eigen::Vector2d&);
    /// Construct the train date from a store
    void constructXY(const RoboMoves::Store&);
    /// save/load
    template <class Archive>
    void serialize(Archive &ar, unsigned version)
    { ar & _mX & _mY & _mq & _max_controls_count; }
};
}
