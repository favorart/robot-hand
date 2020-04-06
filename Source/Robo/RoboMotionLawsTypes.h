#pragma once

#include "RoboInputs.h"
#include "RoboMotionLaws.h"

//#include "lagrange_interp_1d.hpp"

void inputAngles(const tstring &filename, std::vector<double> &angles);
void outputAngles(const tstring &filename, const std::vector<double> &angles);


namespace Robo {
namespace MotionLaws {
//------------------------------------------------------------------------------
//  Основные движения.
//  Для примера реализованы две функции работы мускулов руки:
//  они учитывают ускорение и замедление руки во время движения.
//------------------------------------------------------------------------------
class ContinuousAcceleration : public JointMoveLawI
{
    /*  Моделируется нелинейное ускорение руки во время работы мускулов,
     *  отрезком функции f(i) = (1 - cos(i * PI / (n - 1))) / 2, где i меняется [0, n-1].
     *      . . .
     */
public:
    ContinuousAcceleration() : JointMoveLawI(JointMoveLawI::Law::Move) {}
    void generate(IterVecDoubles first, size_t count, double left, double right, double) const override
    {
        IterVecDoubles iter = first;
        size_t n = (count - 1);
        double summary = 0.;
        for (size_t i = 0U; i <= n; ++i)
        {
            /* diff velosity on start and end */
            *iter = std::max(Epsilont, (1. - cos(i * M_PI / n)) * 0.5);
            summary += *iter; /* calc for normalization */
            ++iter;
        }
        normalize(first, count, left, right, summary);
    }
};

//------------------------------------------------------------------------------
class ContinuousDeceleration : public JointMoveLawI
{
    /*  Инерция.
     *  Кадры после получения сигнала остановки идут в порядке убывания
     *  расстояния, начиная с величины равной или меньшей максимальной
     *  скорости развитой рукой в кадрах основного движения.
     *  Таким образом, после прекращения мышечного усилия скорость
     *  движения руки уменьшается не мгновенно.
     *       . . .
     */
public:
    ContinuousDeceleration() : JointMoveLawI(JointMoveLawI::Law::Stop) {}
    void generate(IterVecDoubles first, size_t count, double left, double right, double max_velosity) const override
    {
        IterVecDoubles iter = first;
        size_t n = (count - 1);
        if (count <= 1)
        {
            *first = right * 3 / 2;
            return;
        }
        double a = left, b = right, v = max_velosity;
        // --------------------------------
        double norm_sum = 0.;
        for (size_t i = 0U; i <= n; ++i)
        {
            /* diff velosity on start and end */
            *iter = std::max(Epsilont, (1. - cos((double(i) / n + 1.) * M_PI)) * 0.5);
            /* calc normalization */
            norm_sum += *iter;
            ++iter;
        }
        /* apply normalization */
        for (size_t i = 0U; i <= n; ++i)
        {
            *first = *first * (b - (n + 1) * a) / norm_sum + a;
            if (*first > v)  *first = v;
            ++first;
        }
    }
};

//------------------------------------------------------------------------------
class ContinuousAccelerationThenStabilization : public JointMoveLawI
{
    /*  Моделируется нелинейное ускорение на протяжении первой части работы мускула,
     *  длина этой части задаётся параметром AccelerationLong ϵ [0.0, 1.0].
     *  Затем скорость движения руки стабилизируется до полного раскрытия ...
     */
    double accelerationLong_{};
public:
    // --------------------------------
    ContinuousAccelerationThenStabilization(double dStableRatio = 0.45)
        : JointMoveLawI(JointMoveLawI::Law::Move)
    {
        if (0. > dStableRatio || dStableRatio > 1.)
            throw std::logic_error{ "Invalid Hand law" };
        accelerationLong_ = 1. - dStableRatio;
    }
    void generate(IterVecDoubles first, size_t count, double left, double right, double) const override
    {
        IterVecDoubles iter = first;
        // --------------------------------
        size_t  n = count;
        size_t  m = static_cast<size_t> (count * accelerationLong_);
        // --------------------------------
        double summary = 0.;
        for (size_t i = 0U; i <= m; ++i)
        {
            /* diff velosity on start and end */
            double d = (1. - cos(i * M_PI / m)) * 0.5;
            *iter = std::max(d, Epsilont);
            summary += *iter; /* calculating a normalization */
            ++iter;
        }
        for (size_t i = (m + 1U); i < n; ++i)
        {
            /* constant after getting the full speed */
            *iter = 1.;
            /* continue calculating a normalization */
            summary += *iter;
            ++iter;
        }
        // --------------------------------
        for (size_t i = 0U; i < n; ++i)
        {
            /* apply the normalization */
            double d = *first * (right - (n + 1) * left) / summary + left;
            *first = d;
            ++first;
        }
    }
};

//------------------------------------------------------------------------------
class ContinuousFastAcceleration : public JointMoveLawI
{
    /*  Моделируется нелинейное ускорение руки во время работы мускулов,
     *  отрезком функции f(i) = ((log (i / (n - 1)) + 3.) * 0.33),
     *  где i меняется [0, n-1] ...
     */
public:
    ContinuousFastAcceleration() : JointMoveLawI(JointMoveLawI::Law::Move) {}
    void generate(IterVecDoubles first, size_t count, double left, double right, double) const override
    {
        IterVecDoubles iter = first;
        size_t n = count;
        double summary = 0.;
        for (size_t i = 0; i < n; ++i)
        {
            *iter = std::max(Epsilont, log(double(i) / n + 2.) * 0.2);
            summary += *iter;
            ++iter;
        }
        normalize(first, count, left, right, summary);
    }
};

//------------------------------------------------------------------------------
class ContinuousSlowAcceleration : public JointMoveLawI
{
    /*  Моделируется нелинейное ускорение руки во время работы мускулов,
     *  отрезком функции f(i) = (exp(i / (n-1) - 1.2) * 2. - 0.6),
     *  где i меняется [0, n-1] ...
     */
public:
    ContinuousSlowAcceleration() : JointMoveLawI(JointMoveLawI::Law::Move) {}
    void generate(IterVecDoubles first, size_t count, double left, double right, double) const override
    {
        IterVecDoubles iter = first;
        size_t n = count;
        double summary = 0.;
        for (size_t i = 1; i <= n; ++i)
        {
            *iter = std::max(Epsilont, exp(double(i) / (n + 1) - 1.2) * 2. - 0.6);
            summary += *iter;
            ++iter;
        }
        normalize(first, count, left, right, summary);
    }
};

//------------------------------------------------------------------------------
class PhisicalAcceleration // : public JointMoveLawI
{
    using IterVecDoubles = std::vector<double>::iterator;
    /* Зависимость от сил, масс, трения и моментов инерции */

    double  EngineForce;   /* Сила привода двигателя */
    double  MaxVelosity;   /* Максимальная развиваемая моментальная скорость */
    double  Acceleration;  /* Ускорение придаваемое за счёт усилия двигателя */

    double  SectionMass;   /* Масса звена и всего, что удерживается сочленением */
    double  JountMass;     /* Масса сочленения */
    double  JountFriction; /* Трение в сочленении */
public:
    virtual void generate(IterVecDoubles /*first*/, size_t /*count*/, double /*left*/, double /*right*/, double) const /*override*/
    {}
};

//------------------------------------------------------------------------------
class PhisicalDeceleration // : public JointMoveLawI
{
    using IterVecDoubles = std::vector<double>::iterator;
    /* ??? Коэффициент взаимодействия приводов */
    double FrictionForce;  /* Сила трения */
    double InrtiaMoment;   /* Момент инерции */
public:
    // template <typename ForwardIterator>
    virtual void generate(IterVecDoubles /*first*/, size_t /*count*/, double /*left*/, double /*right*/, double /*max_velosity*/) const /*override*/
    {}
};

//------------------------------------------------------------------------------
class MangoAcceleration : public JointMoveLawI
{
    tstring filename;
public:
    MangoAcceleration(const tstring &filename) : JointMoveLawI(JointMoveLawI::Law::Move), filename(filename) {}
    void generate(IterVecDoubles first, size_t count, double left, double right, double) const override
    {
        std::vector<double> angles;
        inputAngles(filename, angles);
        // --------------------------------
        IterVecDoubles iter = first;
        // --------------------------------
        size_t  n = count;
        size_t  m = angles.size();
        // --------------------------------
        // std::vector<double>  new_frames (n);
        // double n_frame = 1.;
        // for ( auto &frame : new_frames )
        // { frame = n_frame * m / n;  n_frame += 1.; }
        // // --------------------------------
        // double*  new_angles = lagrange_value_1d (static_cast<int> (m), frames.data (), angles.data (),
        //                                          static_cast<int> (n), new_frames.data ());
        // --------------------------------
        double summary = 0.;
        for (size_t i = 0U; i < n; ++i)
        {
            double d = angles[size_t(i * m / n)];
            summary += d;
            *iter = d;
            ++iter;
        }
        // --------------------------------
        // delete[] new_angles;
        // --------------------------------
        normalize(first, count, left, right, summary);
        // --------------------------------
        // Mango::generate() outputAngles(filename, angles);
    }
};

//------------------------------------------------------------------------------
class MangoDeceleration : public JointMoveLawI
{
    tstring filename;
public:
    MangoDeceleration(const tstring &filename) : JointMoveLawI(JointMoveLawI::Law::Stop), filename(filename) {}
    void generate(IterVecDoubles first, size_t count, double left, double right, double max_velosity) const override
    {
        std::vector<double> angles;
        inputAngles(filename, angles);
        //------------------------------------------------------
        IterVecDoubles iter = first;
        //------------------------------------------------------
        size_t  n = count;
        size_t  m = angles.size();
        double  v = max_velosity;
        //------------------------------------------------------
        // std::vector<double>  new_frames (n);
        // size_t  n_frame = 1U;
        // for ( auto &frame : new_frames )
        // { frame = size_t (n_frame * m / n);  n_frame += 1.; }
        // --------------------------------
        // double*  new_angles = lagrange_value_1d (static_cast<int> (10), frames.data (), angles.data (),
        //                                          static_cast<int> (3), new_frames.data ());
        //------------------------------------------------------
        double summary = 0.;
        for (size_t i = 0U; i < n; ++i)
        {
            double d = angles[size_t(i * m / n)]; // new_angles[i];

            summary += d;
            *iter = d;
            ++iter;
        }
        //------------------------------------------------------
        // delete[] new_angles;
        //------------------------------------------------------
        iter = first;
        for (size_t i = 0U; i < n; ++i) /* apply normalization */
        {
            *iter = *iter * (right - n * left) / summary + left;
            if (*iter > v)
                *iter = v;
            ++iter;
        }
        //------------------------------------------------------
        // Mango::generate() outputAngles(filename, angles);
    }
};

//------------------------------------------------------------------------------
class ElbowMoveLaw : public JointMoveLawI
{
    /*  Каждый сустав может иметь свою разбивку по кадрам -
     *  величину пройдённого расстояния или «закон движения» ...
     */
public:
    ElbowMoveLaw() : JointMoveLawI(JointMoveLawI::Law::Move) {}
    void generate(IterVecDoubles /*first*/, size_t /*count*/, double /*left*/, double /*right*/, double) const override
    {}
};
} // MotionLaws
} // Robo
