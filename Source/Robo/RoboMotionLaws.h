#pragma once

#ifndef  _MOTION_LAWS_H_
#define  _MOTION_LAWS_H_
//------------------------------------------------------------------------------
namespace Robo {
namespace MotionLaws {

using IterVecDoubles = std::vector<double>::iterator;
//------------------------------------------------------------------------------
/*  Здесь указан интерфейс, в котором эмулятор руки принимает закон движения мускула.
 *  Открывающий и закрывающий мускулы одного сустава получают одинаковую разбивку по
 *  кадрам и одинаковые граничные условия.
 */
class JointMoveLawI
{
public:
    // --------------------------------
    virtual void  generate(IterVecDoubles first, size_t frames_count,
                           double left_border, double right_border) const = 0;

    template <typename ForwardIterator>
    /* virtual */ void  generate(ForwardIterator first, size_t frames_count,
                                 double left_border, double right_border) const = 0;
};

class JointStopLawI
{
public:
    // --------------------------------
    virtual void  generate(IterVecDoubles first, size_t frames_count,
                           double left_border, double right_border,
                           double max_velosity) const = 0;

    template <typename ForwardIterator>
    /* virtual */ void  generate(ForwardIterator first, size_t frames_count,
                                 double left_border, double right_border,
                                 double max_velosity) const = 0;
};

//------------------------------------------------------------------------------
struct JointMotionLaw
{
    std::shared_ptr<JointMoveLawI> moveLaw;
    std::shared_ptr<JointStopLawI> stopLaw;

    double stopDistanceRatio{ 0.55 }; // 55% от общего пробега
    uint8_t type{ 0xFF };
    tstring typeName{};
    tstring param{};

    JointMotionLaw(JointMoveLawI *pMoveLaw, JointStopLawI *pStopLaw) :
        moveLaw(pMoveLaw), stopLaw(pStopLaw)
    {}
    JointMotionLaw(JointMoveLawI *pMoveLaw, JointStopLawI *pStopLaw,
                   tstring typeName, uint8_t type, tstring param) :
        moveLaw(pMoveLaw), stopLaw(pStopLaw),
        typeName(typeName), type(type), param(param)
    {}
};

//------------------------------------------------------------------------------
/*  Основные движения.
 *  Для примера реализованы две функции работы мускулов руки:
 *  они учитывают ускорение и замедление руки во время движения.
 */
class ContinuousAcceleration : public JointMoveLawI
{
    /*  Моделируется нелинейное ускорение руки во время работы мускулов,
     *  отрезком функции f(i) = (1 - cos(i * PI / (n - 1))) / 2, где i меняется [0, n-1].
     *      . . .
     */
public:
    // --------------------------------
    // template <typename ForwardIterator>
    virtual void  generate(IterVecDoubles first, size_t frames_count,
                           double left_border, double right_border) const
    {
        IterVecDoubles  iter = first;
        size_t  n = (frames_count - 1U);
        double a = left_border, b = right_border;
        // --------------------------------
        double  norm_sum = 0.;

        for (size_t i = 0U; i <= n; ++i)
        { /* diff velosity on start and end */
            *iter = (1. - cos(i * M_PI / n)) * 0.5;
            /* calc normalization */
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
class ContinuousDeceleration : public JointStopLawI
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
    // --------------------------------
    // template <typename ForwardIterator>
    virtual void generate(IterVecDoubles first, size_t frames_count,
                          double left_border, double right_border,
                          double max_velosity) const
    {
        IterVecDoubles  iter = first;
        size_t  n = (frames_count - 1U);
        if (frames_count <= 1)
        {
            *first = right_border * 3 / 2;
            return;
        }
        double a = left_border, b = right_border, v = max_velosity;
        //------------------------------------------------------
        double  norm_sum = 0.;
        for (size_t i = 0U; i <= n; ++i)
        {
            /* diff velosity on start and end */
            double  d = static_cast<double> (i) / n;
            *iter = (1. - cos((d + 1.) * M_PI)) * 0.5;
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
        //------------------------------------------------------
    }
};
//------------------------------------------------------------------------------
class ElbowMoveLaw : public JointMoveLawI
{
    /*  Каждый сустав может иметь свою разбивку по кадрам -
     *  величину пройдённого расстояния или «закон движения».
     *     . . .
     */
public:
    // --------------------------------
    // template <typename ForwardIterator>
    virtual void  generate(IterVecDoubles first, size_t frames_count,
                           double left_border, double right_border) const {}
};

}
}
//------------------------------------------------------------------------------
#endif // _MOTION_LAWS_H_
