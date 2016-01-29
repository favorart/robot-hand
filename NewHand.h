#include "StdAfx.h"
// #include "circle.h"
// #pragma once

#ifndef  _NEW_HAND_H_
#define  _NEW_HAND_H_
//------------------------------------------------------------------------------
namespace NewHand
{
  class Hand
  {
  public:         
    typedef uint32_t time_t;
    typedef uint32_t frames_t;
    //----------------------------------------------------
    typedef enum : uint8_t
    { /* Opn - open,
       * Cls - close
       */
      EmptyMov = 0,
      ClvclOpn = 1 << 0,
      ClvclCls = 1 << 1,
      ShldrOpn = 1 << 2,
      ShldrCls = 1 << 3,
      ElbowOpn = 1 << 4,
      ElbowCls = 1 << 5,
      WristOpn = 1 << 6,
      WristCls = 1 << 7
    } MusclesEnum;

    typedef enum : uint8_t
    {
      ClvclOpnIndex = 0U,
      ClvclClsIndex = 1U,
      ShldrOpnIndex = 2U,
      ShldrClsIndex = 3U,
      ElbowOpnIndex = 4U,
      ElbowClsIndex = 5U,
      WristOpnIndex = 6U,
      WristClsIndex = 7U,
      MusclesCount  = 8U
    } MusclesIndexEnum;

    //----------------------------------------------------
    typedef enum : uint8_t
    { Empty = 0,
      Clvcl = 1 << 0, // 1,  // ключица:   clavicle
      Shldr = 1 << 1, // 2,  // плечо:     sholder
      Elbow = 1 << 2, // 4,  // локоть:    elbow
      Wrist = 1 << 3  // 8   // запястье:  carpus , wrist
    } JointsEnum;

    typedef enum : uint8_t
    {
      ClvclIndex  = 0,
      ShldrIndex  = 1,
      ElbowIndex  = 2,
      WristIndex  = 3,
      JointsCount = 4
    } JointsIndexEnum;
    //----------------------------------------------------

    size_t   jointsCount;
    size_t  musclesCount;

    typedef std::array< JointsEnum,  JointsCount> j_array;
    typedef std::array<MusclesEnum, MusclesCount> m_array;

    // std::array< JointsEnum,  JointsCount>   joints_;
    // std::array<MusclesEnum, MusclesCount>  muscles_;

    std::vector< JointsEnum>  joints_;
    std::vector<MusclesEnum>  muscles_;

    MusclesIndexEnum   muscleIndex (MusclesEnum  muscle);
     JointsIndexEnum    jointIndex ( JointsEnum   joint);

    frames_t  maxMuscleLast (MusclesEnum  muscle);

  private:

    struct HandStatus
    {
      //---current position---------------------------------
      Point  curPosHand_, curPosArm_, curPosShldr_;

      double angleElbow_, angleShldr_, shiftClvcl_, angleWrist_;

      bool   moveEnd_;

      //---parameters of hydraulic force--------------------
      // std::array<size_t, JointsCount> curMoveFrame;
      // std::array<size_t, JointsCount> curStopFrame;

      std::array<frames_t, JointsCount> lastsMove;
      std::array<frames_t, JointsCount> lastsStop;
      std::array<frames_t, JointsCount> lasts_;

      // size_t ElbowLast_, ShldrLast_;
      // size_t lasts;
      // std::array<frames_t, jointsCount> lastMove;
      MusclesEnum  musclesMove_;
      //----------------------------------------------------
      std::array<double, JointsCount> prevFrame_;
      // double  velosity;
    };

    HandStatus hs;

    //---internal phisical parameters---------------------
    const std::array<uint_t, JointsCount>  maxMoveFrames;
    const std::array<uint_t, JointsCount>  minStopFrames;

    std::array<std::vector<double>, JointsCount>  framesMove;
    std::array<std::vector<double>, JointsCount>  framesStop;

    //---angle limits-------------------------------------
    const double  maxClvclShift;
    const uint_t  maxShldrAngle;
    const uint_t  maxElbowAngle;
    const uint_t  maxWristAngle;
    
    //---base position------------------------------------
    Point   hand_, arm_, sholder_, clavicle_;

    //----------------------------------------------------
    double   nextFrame (MusclesEnum muscle, frames_t &frame, bool atStop);
    bool   muscleFrame (MusclesEnum muscle, frames_t &frame, bool atStop);

    void   muscleMove (JointsIndexEnum jointIndex, MusclesEnum muscle, frames_t last, bool control);
    //----------------------------------------------------

  public:
    //----------------------------------------------------
    Hand (const Point &hand    = { -0.70, 1.00 }, const Point &arm      = { 0.10, 0.85 },
          const Point &sholder = {  0.75, 0.25 }, const Point &clavicle = { 0.75, 0.25 },
          const std::vector<JointsEnum /*, JointsCount */>  joints  = { Shldr, Elbow },
          const std::vector<MusclesEnum/*, MusclesCount*/>  muscles = { ShldrOpn, ShldrCls, ElbowOpn, ElbowCls }
         );

    void  draw (HDC hdc, HPEN hPen, HBRUSH hBrush) const;

    void  move (MusclesEnum muscle, frames_t last);
    void  move (MusclesEnum muscle, frames_t last, std::list<Point> &visited);
    void  step (MusclesEnum muscle=EmptyMov, frames_t last=0U);

    /* jOp = { Clvcl, Shldr, Elbow } < 100.0 % */
    void  set (JointsEnum joint, const std::array<double, Hand::JointsCount> &jointsOpenPercent);
    // void  set (MusclesEnum muscle, frames_t frame);
    void  reset ();

    /* Microsoft specific: C++ properties */
    __declspec(property(get = get_mend)) bool moveEnd;
    bool  get_mend () const { return hs.moveEnd_; }
    __declspec(property(get = get_posit)) const Point& position;
    const Point&  get_posit () const { return hs.curPosHand_; }
    __declspec(property(get = get_pos)) boost_point2_t pos;
    boost_point2_t  get_pos () const { return hs.curPosHand_; }
    //----------------------------------------------------
  };

  static const Hand::j_array   joints = { Hand::Clvcl, Hand::Shldr,
                                          Hand::Elbow, Hand::Wrist };

  static const Hand::m_array  muscles = { Hand::ClvclOpn, Hand::ClvclCls,
                                          Hand::ShldrOpn, Hand::ShldrCls,
                                          Hand::ElbowOpn, Hand::ElbowCls,
                                          Hand::WristOpn, Hand::WristCls };
                                          
  std::vector<double>  generateMoveFrames (double a, double b, size_t n);
  std::vector<double>  generateStopFrames (double a, double b, size_t n);
  //------------------------------------------------------------------------------
  Hand::MusclesEnum  operator| (Hand::MusclesEnum m, Hand::MusclesEnum k);
  Hand::MusclesEnum  operator& (Hand::MusclesEnum m, Hand::MusclesEnum k);
  Hand::MusclesEnum  operator^ (Hand::MusclesEnum m, Hand::MusclesEnum k);

  Hand::JointsEnum   operator| (Hand::JointsEnum  j, Hand::JointsEnum  k);
  Hand::JointsEnum   operator& (Hand::JointsEnum  j, Hand::JointsEnum  k);
  Hand::JointsEnum   operator^ (Hand::JointsEnum  j, Hand::JointsEnum  k);

  std::ostream&  operator<< (std::ostream &out, Hand::MusclesEnum m);
  std::ostream&  operator<< (std::ostream &out, Hand::JointsEnum  j);
  //------------------------------------------------------------------------------
  bool  muscleValidAtOnce (Hand::MusclesEnum muscle);

  const size_t  HandMovesCount = 26UL;
  Hand::MusclesEnum  selectHandMove (size_t choose);

  Hand::MusclesEnum  muscleByJoint (Hand::JointsEnum  joint, bool open);
  Hand::JointsEnum   jointByMuscle (Hand::MusclesEnum muscle);

#define SET_DEFAULT set(NewHand::Hand::Shldr | NewHand::Hand::Elbow, { 0., 70. })
};
//------------------------------------------------------------------------------
#endif // _HAND_H_

