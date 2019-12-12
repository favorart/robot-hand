#include "RoboMotionLaws.h"

namespace Robo {
namespace MotionLaws {
//------------------------------------------------------------------------------
class ContinuousAccelerationThenStabilization : public JointMoveLawI
{
    /*  Моделируется нелинейное ускорение на протяжении первой части работы мускула,
     *  длина этой части задаётся параметром AccelerationLong ϵ [0.0, 1.0].
     *  Затем скорость движения руки стабилизируется до полного раскрытия.
     *  ...
     */
    double accelerationLong_;
public:
    // --------------------------------
    ContinuousAccelerationThenStabilization(double dStableRatio = 0.45)
    {
        if (0. > dStableRatio || dStableRatio > 1.)
            throw std::logic_error{ "Invalid Hand law" };
        accelerationLong_ = 1. - dStableRatio;
    }
    virtual void  generate(IterVecDoubles first, size_t frames_count,
                           double left_border, double right_border) const
    {
        IterVecDoubles iter = first;
        // --------------------------------
        size_t  n = frames_count;
        size_t  m = static_cast<size_t> (frames_count * accelerationLong_);
        double  a = left_border,
                b = right_border;
        // --------------------------------
        double norm_sum = 0.;
        for (size_t i = 0U; i <= m; ++i)
        {
            /* diff velosity on start and end */
            double d = (1. - cos(i * M_PI / m)) * 0.5;
            *iter = std::max(d, Epsilont);
            norm_sum += *iter; /* calculating a normalization */
            ++iter;
        }
        for (size_t i = (m + 1U); i < n; ++i)
        { 
            /* constant after getting the full speed */
            *iter = 1.;
            /* continue calculating a normalization */
            norm_sum += *iter;
            ++iter;
        }
        // --------------------------------
        for (size_t i = 0U; i < n; ++i)
        {
            /* apply the normalization */
            double d = *first * (b - n * a) / norm_sum + a;
            *first = d;
            ++first;
        }
        // --------------------------------
    }
};
//------------------------------------------------------------------------------
class ContinuousFastAcceleration : public JointMoveLawI
{
    /*  Моделируется нелинейное ускорение руки во время работы мускулов,
     *  отрезком функции f(i) = ((log (i / (n - 1)) + 3.) * 0.33),
     *  где i меняется [0, n-1]. . .
     */
public:
    virtual void  generate(IterVecDoubles first, size_t frames_count,
                           double left_border, double right_border) const
    {
        IterVecDoubles iter = first;
        size_t  n = (frames_count - 1U);
        double  a = left_border,
                b = right_border;
        // --------------------------------
        double norm_sum = 0.;
        for (size_t i = 1U; i <= n; ++i)
        {
            double d = (log(double(i) / n + 2.)) * 0.2;
            *iter = std::max(d, Epsilont);
            norm_sum += *iter;
            ++iter;
        }
        /* apply normalization */
        for (size_t i = 0U; i <= n; ++i)
        {
            *first = *first * (b - (n + 1) * a) / norm_sum + a;
            ++first;
        }
        // --------------------------------
    }
};
class ContinuousSlowAcceleration : public JointMoveLawI
{
    /*  Моделируется нелинейное ускорение руки во время работы мускулов,
     *  отрезком функции f(i) = (exp(i / (n-1) - 1.2) * 2. - 0.6),
     *  где i меняется [0, n-1]. . .
     */
public:
    virtual void  generate(IterVecDoubles first, size_t frames_count,
                           double left_border, double right_border) const
    {
        IterVecDoubles iter = first;
        size_t  n = (frames_count - 1U);
        double  a = left_border, b = right_border;
        // --------------------------------
        double norm_sum = 0.;
        for (size_t i = 1U; i <= n; ++i)
        {
            double d = (exp(double(i) / (n - 1) - 1.2) * 2. - 0.6);
            *iter = std::max(d, Epsilont);
            norm_sum += *iter;
            ++iter;
        }
        /* apply normalization */
        for (size_t i = 0U; i <= n; ++i)
        {
            *first = *first * (b - (n + 1) * a) / norm_sum + a;
            ++first;
        }
        // --------------------------------
    }
};
//------------------------------------------------------------------------------
/* Зависимость от сил, масс, трения и моментов инерции */
class PhisicalAcceleration
{
    double  EngineForce;   /* Сила привода двигателя */
    double  MaxVelosity;   /* Максимальная развиваемая моментальная скорость */
    double  Acceleration;  /* Ускорение придаваемое за счёт усилия двигателя */

    double  SectionMass;   /* Масса звена и всего, что удерживается сочленением */
    double  JountMass;     /* Масса сочленения */
    double  JountFriction; /* Трение в сочленении */
public:
    virtual void  generate(IterVecDoubles first, size_t frames_count,
                           double left_border, double right_border) const
    {}
};
class PhisicalDeceleration
{
    /* ??? Коэффициент взаимодействия приводов */
    double FrictionForce;  /* Сила трения */
    double InrtiaMoment;   /* Момент инерции */
public:
    // template <typename ForwardIterator>
    virtual void  generate(IterVecDoubles first, size_t frames_count,
                           double left_border, double right_border,
                           double max_velosity) const
    {}
};
//------------------------------------------------------------------------------
}
}

//#include "lagrange_interp_1d.hpp"

void inputAngles(const tstring &filename, std::vector<double> &angles);
void outputAngles(const tstring &filename, const std::vector<double> &angles);


namespace Robo {
namespace MotionLaws {
//------------------------------------------------------------------------------
class MangoAcceleration : public JointMoveLawI
{
    tstring filename;
public:
    // --------------------------------
    MangoAcceleration(const tstring &filename) : filename(filename) {}
    // --------------------------------
    virtual void  generate(IterVecDoubles first, size_t frames_count,
                           double left_border, double right_border) const
    {
        std::vector<double> angles;
        inputAngles(filename, angles);
        // --------------------------------
        IterVecDoubles iter = first;
        // --------------------------------
        size_t  n = frames_count;
        size_t  m = angles.size();
        double  a = left_border,
            b = right_border;
        // --------------------------------
        // std::vector<double>  new_frames (n);
        // double n_frame = 1.;
        // for ( auto &frame : new_frames )
        // { frame = n_frame * m / n;  n_frame += 1.; }
        // // --------------------------------
        // double*  new_angles = lagrange_value_1d (static_cast<int> (m), frames.data (), angles.data (),
        //                                          static_cast<int> (n), new_frames.data ());
        // --------------------------------
        double  norm_sum = 0.;
        for (size_t i = 0U; i < n; ++i)
        {
            double d = angles[size_t(i * m / n)];

            norm_sum += d;
            *iter = d;
            ++iter;
        }
        // --------------------------------
        // delete[] new_angles;
        // --------------------------------
        iter = first;
        for (size_t i = 0U; i < n; ++i)
        { /* apply the normalization */
            *iter = *iter * (b - n * a) / norm_sum + a;
            ++iter;
        }
        // --------------------------------
        // Mango::generate() outputAngles(filename, angles);
    }
};
class MangoDeceleration : public JointStopLawI
{
    tstring  filename;
public:
    MangoDeceleration(const tstring &filename) : filename(filename) {}
    // --------------------------------
    virtual void generate(IterVecDoubles first, size_t frames_count,
                          double left_border, double right_border,
                          double max_velosity) const
    {
        std::vector<double> angles;
        inputAngles(filename, angles);
        //------------------------------------------------------
        IterVecDoubles iter = first;
        //------------------------------------------------------
        size_t  n = frames_count;
        size_t  m = angles.size();
        double  a = left_border,
            b = right_border,
            v = max_velosity;
        //------------------------------------------------------
        // std::vector<double>  new_frames (n);
        // size_t  n_frame = 1U;
        // for ( auto &frame : new_frames )
        // { frame = size_t (n_frame * m / n);  n_frame += 1.; }
        // --------------------------------
        // double*  new_angles = lagrange_value_1d (static_cast<int> (10), frames.data (), angles.data (),
        //                                          static_cast<int> (3), new_frames.data ());
        //------------------------------------------------------
        double  norm_sum = 0.;
        for (size_t i = 0U; i < n; ++i)
        {
            double d = angles[size_t(i * m / n)]; // new_angles[i];

            norm_sum += d;
            *iter = d;
            ++iter;
        }
        //------------------------------------------------------
        // delete[] new_angles;
        //------------------------------------------------------
        iter = first;
        for (size_t i = 0U; i < n; ++i)
        { /* apply normalization */
            *iter = *iter * (b - n * a) / norm_sum + a;
            if (*iter > v)  *iter = v;
            ++iter;
        }
        //------------------------------------------------------
        // Mango::generate() outputAngles(filename, angles);
    }
};
//------------------------------------------------------------------------------
}
}

