#pragma once

namespace Robo {
enum class Enviroment : short
{
    NOTHING = 0,
    /// internal phisical parameters
    MUTIAL_BLOCKING     = 1 << 0, ///<   1 - блокировка сустава, фиксировано раскрытие в моменты, когда сразу на 2х его приводах одинаковое усилие
    MUTIAL_DYNAMICS     = 1 << 1, ///<   2 - влияние суставов на соседние при работе, когда те не блокированы (момент инерции)
    OPPOSITE_HANDLE     = 1 << 2, ///<   4 - возможна одновременная работа сразу 2х приводов одного сустава
    /// 1. мгновенные изменения 
    MOMENTUM_CHANGES    = 1 << 3, ///<   8 - (в любой такт движения/остановки) единоразовое смещение, шум, который нужно отфильтровать
    /// 2. системные изменения
    SYSTEMATIC_CHANGES  = 1 << 4, ///<  16 - постепенно нарастающие изменения в законе движения одного или несколько мускулов (?сочленения)
    /// 3. граничные условия
    START_FRICTION      = 1 << 5, ///<  32 - несколько тактов в начале движение минимально, либо отсутствет, за ним взрывное ускорение
    EDGES               = 1 << 6, ///<  64 - удары и биения на самопересечениях и границах рабочей области
    
    WINDY               = 1 << 7, ///< 128 - случайное перемещение в первый такт движения мускулом
    WEATHER             = 1 << 8, ///< 256 - 
    _LAST_              = 10
};
} // Robo
using ENV = Robo::Enviroment;

#ifdef WIN32
#include <winnt.h>
DEFINE_ENUM_FLAG_OPERATORS(ENV)
#endif

namespace Robo {
inline bool anyE(ENV e) { return (e != ENV::NOTHING); }
inline bool containE(ENV e, ENV conds) { return ((e & conds) != ENV::NOTHING); }
//------------------------------------------------------
constexpr std::array<const TCHAR*, size_t(Robo::Enviroment::_LAST_)> enviroment_outputs =
{
    _T("NOTHING"), _T("MUTIAL_BLOCKING"), _T("MUTIAL_DYNAMICS"), _T("OPPOSITE_HANDLE"),
    _T("MOMENTUM_CHANGES"), _T("SYSTEMATIC_CHANGES"), _T("START_FRICTION"), _T("EDGES"), 
    _T("WINDY"), _T("WEATHER")
};
} // Robo


