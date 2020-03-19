#pragma once

//#include <LAPACK>
//#include <cvm.h>
#include <Eigen/Eigen>
#include "EigenSerializePlugin.h"

#include "Robo.h"
#include "RoboControl.h"

namespace RoboPos {
//------------------------------------------------------------------------------
/// Аппроксимирующая функция - точка остановы по управлениям
/// http://www.machinelearning.ru/wiki/index.php?title=Обучение_с_учителем._Многомерная_интерполяции_и_аппроксимация._Обобщение_на_основе_теории_случайных_функций._Вариант_точного_решения.
class Approx
{
    static const size_t control_size = 3; ///< muscle, start, lasts
    static const size_t point_size = 2; ///< x, y
    
    bool _constructed = false;
    bool _train = false;

    const size_t _max_n_controls;
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
    friend void myConstructXY(Approx&, void*);
public:
    using Noize = std::function<double(size_t)>;
    using Sizing = std::function<double()>;
    /// Calibrating factor (greater is more precise)
    static double sizing() { return 4.; }
    /// Шум - этот коэффициент сообщает, насколько можно доверять управлению с номером i
    static double noize(size_t i = 0) { return 0.00000001; }

    /// Construct from store
    Approx(size_t store_size, size_t max_n_controls, 
           Noize noize = Approx::noize,
           Sizing sizing = Approx::sizing);
    /// Construct from converted matrices
    Approx(Eigen::MatrixXd &X, Eigen::MatrixXd &Y);

    /// If predictor is configurated
    bool constructed() const { return _constructed; }
    /// Convert a raw controls of RoboI to an aligned row of doubles
    Eigen::VectorXd convertToRow(const Robo::Control&) const;
    /// Apply aligned row in currect index
    void insert(const Robo::Control&, Point, size_t index);

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

    /// Construct the train data from any iterable container
    template <typename Iterator>
    void  constructXY(const Iterator begin, const Iterator end)
    {
        int i = 0;
        for (Iterator it = begin; it != end; ++it, ++i)
            insert(it->controls, it->hit, i);
        constructXY();
    }
    /// Construct the train data from any source, with filtering
    template <typename ApproxFilter>
    void  constructXY(ApproxFilter &next)
    {
        for (size_t i = 0; true; ++i)
        {
            auto res = next();
            if (!res)
                break;
            insert(res->controls, res->hit, i);
        }
        constructXY();
    }
    /// save/load
    template <class Archive>
    void serialize(Archive &ar, unsigned version)
    { ar & _mX & _mY & _mq & _max_controls_count; }
};
} // RoboPos
