#include "StdAfx.h"
#include "HandMotionLaws.h"

#ifndef   _MOTION_LAWS_CUSTOM_H_
#define   _MOTION_LAWS_CUSTOM_H_
//------------------------------------------------------------------------------
namespace NewHand
{
  namespace MotionLaws
  {
    //------------------------------------------------------------------------------
    class ContinuousAccelerationThenStabilization : public JointMoveLaw
    {
      /*  Моделируется нелинейное ускорение на протяжении первой части работы мускула,
      *  длина этой части задаётся параметром AccelerationLong ϵ [0.0, 1.0].
      *  Затем скорость движения руки стабилизируется до полного раскрытия.
      *  ...
      */
      double  AccelerationLong;
    public:
      // --------------------------------
      ContinuousAccelerationThenStabilization (double AccelerationLong=0.45)
      {
        if ( 1. >= AccelerationLong && AccelerationLong >= 0. )
        { this->AccelerationLong = AccelerationLong; }
      }
      // --------------------------------
      // template <typename ForwardIterator>
      virtual void  generate (VectorDoublesIterator first, size_t frames_count,
                              double left_border, double right_border) const
      {
        VectorDoublesIterator  iter = first;
        // --------------------------------
        size_t  n = frames_count;
        size_t  m = static_cast<size_t> (frames_count * AccelerationLong);
        double  a = left_border,
          b = right_border;
        // --------------------------------
        double  norm_sum = 0.;
        for ( size_t i = 0U; i <= m; ++i )
        { /* diff velosity on start and end */
          double d = (1. - cos (i * M_PI / m)) * 0.5;
          /* calculating a normalization */
          norm_sum += d;
          *iter = d;
          ++iter;
        }
        for ( size_t i = (m + 1U); i < n; ++i )
        { /* constant after getting the full speed */
          *iter = 1.;
          /* continue calculating a normalization */
          norm_sum += *iter;
          ++iter;
        }
        // --------------------------------
        // double sum1 = 0.;
        for ( size_t i = 0U; i < n; ++i )
        { /* apply the normalization */
          double d = *first * (b - n * a) / norm_sum + a;
          *first = d;
          ++first;
        }
        // --------------------------------
      }
    };
    //------------------------------------------------------------------------------
    class ContinuousFastAcceleration : public JointMoveLaw
    {
      /*  Моделируется нелинейное ускорение руки во время работы мускулов,
      *  отрезком функции f(i) = ((log (i / (n - 1)) + 3.) * 0.33),
      *  где i меняется [0, n-1]. . .
      */
    public:
      // --------------------------------
      // template <typename ForwardIterator>
      virtual void  generate (VectorDoublesIterator first, size_t frames_count,
                              double left_border, double right_border) const
      {
        VectorDoublesIterator  iter = first;
        size_t  n = (frames_count - 1U);
        double  a = left_border, b = right_border;
        // --------------------------------
        double  norm_sum = 0.;
        *iter = EPS;
        for ( size_t i = 1U; i <= n; ++i )
        {
          double d = (log (double (i) / n + 3.)) * 0.33;
          *iter = (d > 0) ? d : EPS;
          norm_sum += *iter;
          ++iter;
        }
        /* apply normalization */
        for ( size_t i = 0U; i <= n; ++i )
        {
          *first = *first * (b - (n + 1) * a) / norm_sum + a;
          ++first;
        }
        // --------------------------------
      }
    };
    class ContinuousSlowAcceleration : public JointMoveLaw
    {
      /*  Моделируется нелинейное ускорение руки во время работы мускулов,
      *  отрезком функции f(i) = (exp(i / (n-1) - 1.2) * 2. - 0.6),
      *  где i меняется [0, n-1]. . .
      */
    public:
      // --------------------------------
      // template <typename ForwardIterator>
      virtual void  generate (VectorDoublesIterator first, size_t frames_count,
                              double left_border, double right_border) const
      {
        VectorDoublesIterator  iter = first;
        size_t  n = (frames_count - 1U);
        double  a = left_border, b = right_border;
        // --------------------------------
        double  norm_sum = 0.;
        *iter = EPS;
        for ( size_t i = 1U; i <= n; ++i )
        {
          *iter = (exp (double (i) / (n - 1) - 1.2) * 2. - 0.6);
          norm_sum += *iter;
          ++iter;
        }
        /* apply normalization */
        for ( size_t i = 0U; i <= n; ++i )
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
      double  EngineForce;     /* Сила привода двигателя */
      double  MaxVelosity;     /* Максимальная развиваемая моментальная скорость */
      double  Acceleration;    /* Ускорение придаваемое за счёт усилия двигателя */

      double  SectionMass;     /* Масса звена и всего, что удерживается сочленением */
      double    JountMass;     /* Масса сочленения */
      double    JountFriction; /* Трение в сочленении */
    public:
      // template <typename ForwardIterator>
      virtual void  generate (VectorDoublesIterator first, size_t frames_count,
                              double left_border, double right_border) const {}
    };
    class PhisicalDeceleration
    {
      /* ??? Коэффициент взаимодействия приводов */
      double FrictionForce;    /* Сила трения */
      double InrtiaMoment;     /* Момент инерции */
    public:
      // template <typename ForwardIterator>
      virtual void  generate (VectorDoublesIterator first, size_t frames_count,
                              double left_border, double right_border,
                              double max_velosity) const
      {}
    };
    //------------------------------------------------------------------------------
    class MangoAcceleration : public JointMoveLaw
    {
    public:
      // --------------------------------
      MangoAcceleration (tstring &filename)
      {}
      // --------------------------------
      // template <typename ForwardIterator>
      virtual void  generate (VectorDoublesIterator first, size_t frames_count,
                              double left_border, double right_border) const
      {
        VectorDoublesIterator  iter = first;
        // --------------------------------
        size_t  n = frames_count;
        // size_t  m = static_cast<size_t> (frames_count * AccelerationLong);
        double  a = left_border,
          b = right_border;
        // --------------------------------
        double  norm_sum = 0.;
        // for ( size_t i = 0U; i <= m; ++i )
        // { /* diff velosity on start and end */
        //   double d = (1. - cos (i * M_PI / m)) * 0.5;
        //   /* calculating a normalization */
        //   norm_sum += d;
        //   *iter = d;
        //   ++iter;
        // }
        // for ( size_t i = (m + 1U); i < n; ++i )
        // { /* constant after getting the full speed */
        //   *iter = 1.;
        //   /* continue calculating a normalization */
        //   norm_sum += *iter;
        //   ++iter;
        // }
        // --------------------------------
        for ( size_t i = 0U; i < n; ++i )
        { /* apply the normalization */
          *first = *first * (b - (n + 1) * a) / norm_sum + a;
          ++first;
        }
        // --------------------------------
      }
    };
    class MangoDeceleration : public JointStopLaw
    {
    public:
      // --------------------------------
      MangoDeceleration (tstring &filename)
      {}
      // --------------------------------
      // template <typename ForwardIterator>
      virtual void generate (VectorDoublesIterator first, size_t frames_count,
                             double left_border, double right_border,
                             double max_velosity) const
      {
        VectorDoublesIterator  iter = first;
        size_t  n = (frames_count - 1U);
        double  a = left_border,
          b = right_border,
          v = max_velosity;
        //------------------------------------------------------
        double  norm_sum = 0.;
        // for ( size_t i = 0U; i <= n; ++i )
        // {
        //   /* diff velosity on start and end */
        //   double  d = static_cast<double> (i) / n;
        //   *iter = (1. - cos ((d + 1.) * M_PI)) * 0.5;
        //   /* calc normalization */
        //   norm_sum += *iter;
        //   ++iter;
        // }
        /* apply normalization */
        for ( size_t i = 0U; i <= n; ++i )
        {
          *first = *first * (b - (n + 1) * a) / norm_sum + a;
          if ( *first > v )  *first = v;
          ++first;
        }
        //------------------------------------------------------
      }
    };
  }
}
//------------------------------------------------------------------------------
#endif // _MOTION_LAWS_CUSTOM_H_
