﻿#include "StdAfx.h"


#ifndef  _MOTION_LAWS_H_
#define  _MOTION_LAWS_H_
//------------------------------------------------------------------------------
namespace NewHand
{
  namespace MotionLaws
  {
    /*  Здесь указан интерфейс, в котором эмулятор руки принимает закон движения мускула.
     *  Открывающий и закрывающий мускулы одного сустава: они имеют один максимальный
     *  угол раскрытия и получают одинаковую разбивку по кадрам.
     */
    typedef std::function<std::vector<double> (double, double, size_t)> MotionLaw;

    /*  РЕАЛИЗОВАНЫ 2 ФУНКЦИИ:
     *  Они учитывают ускорение и замедление руки во время движения.
     *  Моделируется отрезком функции  (1. - cos (i * M_PI / n)) * 0.5
     */

    /* Основное движение */
    std::vector<double>  generateJointMoveFrames (double left_border, double right_border, size_t frames_count);
    /*  Инерция - кадры, после получения сигнала остановки, должны идут по убыванию
     *  расстояния, покажется начиная с первого кадра, величина, которого меньше
     *  величины в текущем кадре, т.е. скорость движения руки должна уменьшаться
     *  после остановки двигателя.
     */
    std::vector<double>  generateJointStopFrames (double left_border, double right_border, size_t frames_count);

    /*  Каждый сустав может иметь свою разбивку по кадрам (величину пройдённого
     *  расстояния или «закон движения») - это можно сделать следующим образом:
     */
    // std::vector<double>  generateElbowMoveFrames (double, double, size_t);

    std::vector<double>  generateJointMoveFrames_cos (double a, double b, size_t n);
  }
}
//------------------------------------------------------------------------------
#endif // _MOTION_LAWS_H_