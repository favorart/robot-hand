#include "StdAfx.h"
#include "HandMotionLaw.h"


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

    std::vector< JointsEnum>  joints_;
    std::vector<MusclesEnum>  muscles_;
    
    MusclesIndexEnum   muscleIndex (MusclesEnum  muscle) const;
     JointsIndexEnum    jointIndex ( JointsEnum   joint) const;

    frames_t  maxMuscleLast (MusclesEnum  muscle);
    
  private:

    struct HandStatus
    {
      //---current position---------------------------------
      Point  curPosPalm_, curPosHand_, curPosArm_,
             curPosShldr_, curPosClvcl_;
      // TODO: make array

      double angleWrist_, angleElbow_,
             angleShldr_, shiftClvcl_;
      // TODO: make array

      bool   moveEnd_;

      //---parameters of hydraulic force--------------------
      std::array<frames_t, JointsCount> lastsMove;
      std::array<frames_t, JointsCount> lastsStop;
      std::array<frames_t, JointsCount> lasts_;

      MusclesEnum  musclesMove_;
      //----------------------------------------------------
      std::array<double, JointsCount> prevFrame_;
      //----------------------------------------------------
      double  velosity_;
    };
    HandStatus hs;

    bool drawPalm_;
    //---base position------------------------------------
    Point   palm_, hand_, arm_, sholder_, clavicle_;
    // TODO: make array

    //---angle limits-------------------------------------
    // const double  maxClvclShift;
    // const uint_t  maxShldrAngle;
    // const uint_t  maxElbowAngle;
    // const uint_t  maxWristAngle;

    // TODO: make array
    const std::array<size_t, JointsCount> maxJointAngles;
    const double  StopDistaceRatio;

    //---internal phisical parameters---------------------
    const std::array<uint_t, JointsCount>  maxMoveFrames;
    const std::array<uint_t, JointsCount>  minStopFrames;

    std::array<std::vector<double>, JointsCount>  framesMove;
    std::array<std::vector<double>, JointsCount>  framesStop;

    //----------------------------------------------------
    double    nextFrame (MusclesEnum muscle, frames_t frame, bool atStop);
    bool    muscleFrame (MusclesEnum muscle, frames_t frame, bool atStop);

    void    muscleFinal (JointsIndexEnum jointIndex);
    void    muscleBrake (JointsIndexEnum jointIndex);

    void    muscleMove  (JointsIndexEnum jointIndex, MusclesEnum muscle,
                         frames_t last, bool control);
    //----------------------------------------------------
    std::vector<MusclesEnum>  controls;
    void  createControls ();
    void  recursiveControlsAppend (MusclesEnum  muscles,
                                   JointsEnum   joints,
                                   size_t       cur_deep,
                                   size_t       max_deep);

  public:
    //----------------------------------------------------
    Hand (const Point &palm    = { -0.75, 1.05 },
          const Point &hand    = { -0.70, 1.00 }, const Point &arm      = { 0.10, 0.85 },
          const Point &sholder = {  0.75, 0.25 }, const Point &clavicle = { 0.75, 0.25 },
          const std::vector<JointsEnum>  &joints={ Elbow, Shldr }, // , Wrist, Clvcl 
          const std::vector<MotionLaws::MotionLaw> &genMoveFrames = 
          { MotionLaws::generateJointMoveFrames,
            MotionLaws::generateJointMoveFrames },
          const std::vector<MotionLaws::MotionLaw> &genStopFrames = 
          { MotionLaws::generateJointStopFrames,
            MotionLaws::generateJointStopFrames });

    void  draw (HDC hdc, HPEN hPen, HBRUSH hBrush) const;

    frames_t  move (IN MusclesEnum muscle, IN frames_t last);
    frames_t  move (IN MusclesEnum muscle, IN frames_t last, OUT std::list<Point> &visited);

    void  step (IN MusclesEnum muscle=EmptyMov, IN frames_t last=0U);

    /* jointsOpenPercent = { Clvcl, Shldr, Elbow, Wrist } < 100.0 % */
    void  set (JointsEnum joint, const std::array<double, Hand::JointsCount> &jointsOpenPercent);
    void  reset ();

    std::vector<const Point*>  points () const;

    MusclesEnum  selectControl (size_t choose)
    { return  (choose < controls.size ()) ? controls[choose] : Hand::EmptyMov; }
    MusclesEnum  selectControl ();
    // { return  controls[random (controls.size ())]; }
    
    /* Microsoft specific: C++ properties */
    __declspec(property(get = get_mend)) bool moveEnd;
    bool  get_mend () const { return hs.moveEnd_; }
    __declspec(property(get = get_posit)) const Point& position;
    const Point&  get_posit () const { return (drawPalm_) ? hs.curPosPalm_ : hs.curPosHand_; }
    __declspec(property(get = get_pos)) boost_point2_t pos;
    boost_point2_t  get_pos () const { return (drawPalm_) ? hs.curPosPalm_ : hs.curPosHand_; }
    __declspec(property(get = get_n_controls)) size_t controlsCount;
    size_t  get_n_controls () const { return controls.size (); }
    //----------------------------------------------------
  };
  //------------------------------------------------------------------------------
  const tstring  HAND_NAME = _T ("NewHand");
#define SET_DEFAULT set(NewHand::Hand::Shldr|NewHand::Hand::Elbow,{0.,70.});
  //------------------------------------------------------------------------------
};
//------------------------------------------------------------------------------
#endif // _HAND_H_

