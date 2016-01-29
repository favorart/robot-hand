#include "StdAfx.h"
// #include "circle.h"


#ifndef  _HAND_H_
#define  _HAND_H_
//------------------------------------------------------------------------------
namespace OldHand
{
  class Hand
  {
  public:
    //----------------------------------------------------
    typedef uint64_t time_t;
    // typedef ulong_t time_t;
    // typedef unsigned int time_t;

    typedef enum : ::uchar_t
    { /* Opn - open, Cls - close */
      EmptyMov = 0,
      ClvclOpn = 1 << 0,
      ClvclCls = 1 << 1,
      ShldrOpn = 1 << 2,
      ShldrCls = 1 << 3,
      ElbowOpn = 1 << 4,
      ElbowCls = 1 << 5,
      WristOpn = 1 << 6,
      WristCls = 1 << 7,
      MusclesCount = 8
    } MusclesEnum;

    //----------------------------------------------------  
    typedef enum : ::uchar_t
    {
      Empty = 0,
      Clvcl = 1 << 0, // 1 // ключица:   clavicle
      Shldr = 1 << 1, // 2 // плечо:     sholder
      Elbow = 1 << 2, // 4 // локоть:    elbow
      Wrist = 1 << 3, // 8 // запястье:  wrist , carpus
      JointsCount = 4
    } JointsEnum;

    uint_t   jointsCount;
    uint_t  musclesCount;

    typedef std::array< JointsEnum,  JointsCount> j_array;
    typedef std::array<MusclesEnum, MusclesCount> m_array;

    j_array  joints_;
    m_array muscles_;

    size_t muscleIndex (MusclesEnum  muscle);

    const uint_t  minJStopMoveFrames;
    const uint_t  maxClvclMoveFrames;
    const uint_t  maxShldrMoveFrames;
    const uint_t  maxElbowMoveFrames;
    const uint_t  maxWristMoveFrames;

    uint_t maxMuscleLast (MusclesEnum muscle = EmptyMov);

  private:
    //---internal phisical parameters---------------------
    time_t  time_;                            // descrete time
    Point   hand_, arm_, sholder_, clavicle_; // base position

    //---current position---------------------------------
    Point  curPosHand_, curPosArm_, curPosShldr_;
    double angleElbow_, angleShldr_, shiftClvcl_;
    bool   flagMovEnd_;

    //---parameters of hydraulic force--------------------
    typedef union
    { /* Microsoft specific:  Allow to declare
       * the struct in struct without the name!
       */
      struct
      {
        bool Clvcl : 1;
        bool Shldr : 1;
        bool Elbow : 1;
        bool Wrist : 1;
uint_t:3;  /* unused */
        bool Cmptb : 1;  /* bit 7 */
      };
      uchar_t raw;
    } joint_flags_t;

    time_t  timeBgn2OpenHyd_[MusclesCount];
    time_t  timeEnd2OpenHyd_[MusclesCount];
    time_t  tFrames2OpenHyd_[MusclesCount];

  public:

    //---angle limits-------------------------------------
    const double  maxClvclShift;
    const uint_t  maxShldrAngle;
    const uint_t  maxElbowAngle;

    //----------------------------------------------------
    mutable double current_velosity;

    std::vector<double>  c_frames, s_frames, e_frames;
    std::vector<double>  c_frstop, s_frstop, e_frstop;

    double  stepFrame (MusclesEnum hydNo, time_t time, bool atStop) const;
    //----------------------------------------------------
    void  muscle (uint_t no, bool control);
    bool  muscleMove (MusclesEnum hydNo, time_t time, bool atStop = false);

  public:
    //----------------------------------------------------
    //Hand ();
    Hand (const Point &hand = { -0.35, 1.0 }, const Point &arm = { 0.25, 0.7 },
          const Point &sholder = { 0.75, 0.25 }, const Point &clavicle = { 0.75, 0.25 },
          const std::vector<JointsEnum>  & joints_ = { Clvcl, Shldr, Elbow },
          const std::vector<MusclesEnum> &muscles_ = { ClvclOpn, ClvclCls, ShldrOpn,
                                                       ShldrCls, ElbowOpn, ElbowCls });
    
    void  draw (HDC hdc, HPEN hPen, HBRUSH hBrush) const;
    void  move (MusclesEnum muscle, time_t last);              // ??? PROGRESS
    void  move (MusclesEnum muscle, time_t last, std::list<Point> &visited);
    void  step (MusclesEnum muscle = EmptyMov);
    void  step (const bool control[MusclesCount]);

    void  reset (); /* clear, drop - сбрасывать */

    /* jOp = { Clvcl, Shldr, Elbow } < 100.0 % */
    void  set (JointsEnum joint, const std::array<double, Hand::JointsCount> &jOp);
    void  set (MusclesEnum muscle, uint_t frame); // ?????

    /* Microsoft specific: C++ properties */
    __declspec(property(get = get_mend)) bool moveEnd;
    bool  get_mend () const { return flagMovEnd_; }
    __declspec(property(get = get_posit)) const Point& position;
    const Point&  get_posit () const { return curPosHand_; }
    __declspec(property(get = get_pos)) boost_point2_t pos;
    boost_point2_t  get_pos () const { return curPosHand_; }

    struct PositionSensor
    { const static uint_t  amountAppliedPositions = 3U;
      Point   positions_[amountAppliedPositions];
      // Circle  lastDirection_;
    } ps_;
    // const Circle& direction () const { return ps_.lastDirection_; }
    //----------------------------------------------------
  };

  static const Hand::j_array   joints = { Hand::Clvcl, Hand::Shldr,
                                          Hand::Elbow, Hand::Wrist };

  static const Hand::m_array  muscles = { Hand::ClvclOpn, Hand::ClvclCls,
                                          Hand::ShldrOpn, Hand::ShldrCls,
                                          Hand::ElbowOpn, Hand::ElbowCls,
                                          Hand::WristOpn, Hand::WristCls };

  Hand::MusclesEnum  operator| (Hand::MusclesEnum m, Hand::MusclesEnum k);
  Hand::MusclesEnum  operator& (Hand::MusclesEnum h, Hand::MusclesEnum k);

  Hand::JointsEnum   operator| (Hand::JointsEnum  j, Hand::JointsEnum  k);
  Hand::JointsEnum   operator& (Hand::JointsEnum  j, Hand::JointsEnum  k);

  std::ostream&  operator<< (std::ostream &out, Hand::MusclesEnum m);
  std::ostream&  operator<< (std::ostream &out, Hand::JointsEnum  j);

  // std::istream&  operator>> (std::istream &in, Hand::MusclesEnum m);
  // std::istream&  operator>> (std::ostream &in, Hand::JointsEnum  j);
  //------------------------------------------------------------------------------
  bool  muscleValidAtOnce (Hand::MusclesEnum muscle);

  const uint_t  HandMovesCount = 26UL;
  Hand::MusclesEnum  selectHandMove (uint_t choose);

  Hand::MusclesEnum  muscleByJoint (Hand::JointsEnum  joint, bool open);
  Hand::JointsEnum   jointByMuscle (Hand::MusclesEnum muscle);


#define SET_DEFAULT set(Hand::Clvcl|Hand::Shldr|Hand::Elbow,{0,0,70})
};
/*
 * hand
 *=================================
 * // непрервный вывод
 * point output (int time);
 *---------------------------------
 * internal phisical parameters
 *---------------------------------
 * friction s&e joints
 * inertion in joints and muscles
 * time of full stop of muscle
 *
 * момент инерции - масса для вращательного движения
 * int inertia_moment;
 *
 * lenght of hand_
 * масса и распределение
 * mass of hand_
 * mass of sholder
 * mass of arm
 * mass of hand_
 *---------------------------------*/
#endif // _HAND_H_

