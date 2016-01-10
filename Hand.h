#include "StdAfx.h"
// #include "circle.h"


#ifndef  _HAND_H_
#define  _HAND_H_
//------------------------------------------------------------------------------
class Hand
{ //---internal phisical parameters---------------------
  ulong_t  time_;                            // descrete time
  Point    hand_, arm_, sholder_, clavicle_; // base position

	//---current position---------------------------------
	Point  curPosHand_,  curPosArm_, curPosShldr_;
  double angleElbow_, angleShldr_,  shiftClvcl_;
	bool   flagMovEnd_;

  //---draw constants-----------------------------------
	const static double  REllipse;
  const static double  WSholder;

public:
  //----------------------------------------------------
  const static uint_t   jointsCount = 3U;
	const static uint_t  musclesCount = 6U;

  typedef enum : ::uchar_t
  { EmptyMov = 0,
    ClvclOpn = 1 << 0,
    ClvclCls = 1 << 1,
    ShldrOpn = 1 << 2,
    ShldrCls = 1 << 3,
    ElbowOpn = 1 << 4,
    ElbowCls = 1 << 5,
    WristOpn = 1 << 6,
    WristCls = 1 << 7
  } MusclesEnum;
  // Opn - open, Cls - close
  // hydEnginesEnum;

  //----------------------------------------------------  
  typedef enum : ::uchar_t
  { Empty=0,
    Clvcl=1,
    Shldr=2,
    Elbow=4,
    Wrist=8
  }	JointsEnum;
   // запястье: carpus , wrist
	 // ключица: clavicle
	 // локоть: elbow
	 // плечо: sholder

  static const  JointsEnum   joints[ jointsCount];
  static const MusclesEnum  muscles[musclesCount];

  static size_t muscleIndex   (MusclesEnum  muscle);

  static MusclesEnum  muscleByJoint (JointsEnum joint, bool open);

  const uint_t  minJStopMoveFrames; // USING IN TESTS
  const uint_t  maxClvclMoveFrames; // USING IN TESTS
  const uint_t  maxShldrMoveFrames; // USING IN TESTS
  const uint_t  maxElbowMoveFrames; // USING IN TESTS
  const uint_t  maxWristMoveFrames; // USING IN TESTS

  uint_t timeMuscleWorking (MusclesEnum muscle=EmptyMov);

protected:
 	//---parameters of hydraulic force--------------------
  typedef union
  { /* Microsoft specific:  Allow to declare
     * the struct in struct without the name!
     */   
    struct
    { bool Clvcl:1;
      bool Shldr:1;
      bool Elbow:1;
      bool Wrist:1;
      uint_t    :3;  /* unused */
      bool Cmptb:1;  /* bit 7 */
    };
    uchar_t raw;
  } joint_flags_t;

	ulong_t  timeBgn2OpenHyd_[musclesCount];
  ulong_t  timeEnd2OpenHyd_[musclesCount];
  ulong_t  tFrames2OpenHyd_[musclesCount];

  //---angle limits-------------------------------------
	const double  maxClvclShift;
	const uint_t  maxShldrAngle;
	const uint_t  maxElbowAngle;
  
	//----------------------------------------------------  
  virtual double  stepFrame (MusclesEnum hydNo, ulong_t time, bool atStop) const;
  //----------------------------------------------------
  void  muscle     (uint_t no, bool control);
	bool  muscleMove (MusclesEnum hydNo, ulong_t time, bool atStop = false);

public:
  //----------------------------------------------------
	Hand ();

	virtual void  draw (const HDC &hdc, const HPEN &hPen) const;
  virtual void  move (MusclesEnum muscle, ulong_t last);              // ????PROGRESS
  virtual void  move (MusclesEnum muscle, ulong_t last, std::list<Point> &visited);
  virtual void  step (MusclesEnum muscle=EmptyMov);
	virtual void  step (const bool control[musclesCount]);

	virtual void  reset (); /* clear, drop - сбрасывать */
  virtual void  set   (const uchar_t jointOpenPercent[jointsCount]); // ??PARAMETERS
  virtual void  set   (MusclesEnum muscle, uint_t frame); // ?????

	        bool  isMoveEnd () const { return flagMovEnd_; }

  /* Microsoft specific: C++ properties */
  __declspec(property(get = get_pos)) const Point& position;
  const Point&  get_pos () const { return curPosHand_; }
    
    struct PositionSensor
  { const static uint_t  amountAppliedPositions = 3U;

    Point   positions_[amountAppliedPositions];
    // Circle  lastDirection_;
  } ps_;
  // const Circle& direction () const { return ps_.lastDirection_; }
  //----------------------------------------------------
  virtual ~Hand () {}
  //----------------------------------------------------
};

Hand::MusclesEnum  operator| (Hand::MusclesEnum m, Hand::MusclesEnum k);
Hand::JointsEnum   operator| (Hand::JointsEnum  j, Hand::JointsEnum  k);

std::ostream&  operator<< (std::ostream &out, Hand::MusclesEnum m);
std::ostream&  operator<< (std::ostream &out, Hand::JointsEnum  j);

// std::istream&  operator>> (std::istream &in, Hand::MusclesEnum m);
// std::istream&  operator>> (std::ostream &in, Hand::JointsEnum  j);
//------------------------------------------------------------------------------
bool  muscleValidAtOnce (Hand::MusclesEnum muscle);

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
*	масса и распределение
* mass of hand_
* mass of sholder
* mass of arm
* mass of hand_
*---------------------------------*/
#endif // _HAND_H_

