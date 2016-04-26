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
    typedef unsigned int time_t;
    typedef unsigned int frames_t;
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
      Shldr = 1 << 1, // 2,  // плечо:     shoulder
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
    typedef std::array< JointsEnum,  JointsCount> j_array;
    typedef std::array<MusclesEnum, MusclesCount> m_array;
    //----------------------------------------------------
    std::vector< JointsEnum>  joints_;
    std::vector<MusclesEnum>  muscles_;
    //----------------------------------------------------
    MusclesEnum  musclesCumul;
    //----------------------------------------------------
    MusclesIndexEnum   muscleIndex (IN MusclesEnum  muscle) const;
     JointsIndexEnum    jointIndex (IN  JointsEnum   joint) const;
     //----------------------------------------------------
    frames_t  maxMuscleLast (IN MusclesEnum  muscle);
    //----------------------------------------------------
    frames_t  visitedSaveEach;

  private:
    //----------------------------------------------------
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
      std::array<frames_t, MusclesCount> lastsMove;
      std::array<frames_t, MusclesCount> lastsStop;
      std::array<frames_t, MusclesCount> lasts_;

      MusclesEnum  musclesMove_;
      //----------------------------------------------------
      std::array<double, MusclesCount> prevFrame_;
      //----------------------------------------------------
      double  velosity_;
    };
    HandStatus hs;

    bool drawPalm_;
    //---base position------------------------------------
    Point   palm_, hand_, arm_, shoulder_, clavicle_;
    // TODO: make array

    //---angle limits-------------------------------------
    const std::array<size_t, JointsCount> maxJointAngles;
    // maxClvclShift, maxShldrAngle, 
    // maxElbowAngle, maxWristAngle
    const double  StopDistaceRatio;

    //---internal phisical parameters---------------------
    std::array<frames_t, JointsCount>  maxMoveFrames;
    std::array<frames_t, JointsCount>  minStopFrames;

    std::array<std::vector<double>, JointsCount>  framesMove;
    std::array<std::vector<double>, JointsCount>  framesStop;
    //----------------------------------------------------
    double    nextFrame (MusclesEnum muscle, frames_t frame, bool atStop);
    bool    muscleFrame (MusclesEnum muscle, frames_t frame, bool atStop);

    void    muscleBrake (MusclesIndexEnum MuscleIndex, JointsIndexEnum JointIndex);
    void    muscleFinal (MusclesIndexEnum MuscleIndex, MusclesEnum muscle);
    void    muscleMove  (MusclesIndexEnum MuscleIndex, MusclesEnum muscle,
                         frames_t last, bool control);
    //----------------------------------------------------
    std::vector<MusclesEnum>  controls;
    void  createControls ();
    void  recursiveControlsAppend (MusclesEnum  muscles,
                                   JointsEnum   joints,
                                   size_t       cur_deep,
                                   size_t       max_deep);

    //----------------------------------------------------
    const double  minFrameMove = EPS;
    //----------------------------------------------------

  public:
    typedef  std::map<JointsEnum, MotionLaws::JointMotionLaw>  JointsMotionLaws;
    //----------------------------------------------------
    Hand (IN const Point &palm,
          IN const Point &hand,
          IN const Point &arm,
          IN const Point &shoulder,
          IN const Point &clavicle,
          IN const JointsMotionLaws &joints_frames) throw (...);
    //----------------------------------------------------
    void  draw (IN HDC hdc, IN HPEN hPen, IN HBRUSH hBrush) const;
    //----------------------------------------------------
    struct Control
    {
      MusclesEnum  muscle;
      frames_t     start;
      frames_t     last;
      //----------------------------------------------------
      Control () : muscle (EmptyMov), start (0U), last (0U) {}
      Control (MusclesEnum  muscle, frames_t start, frames_t last) :
        muscle (muscle), start (start), last (last) {}
      //----------------------------------------------------
      Control&  operator= (const Control &c)
      {
        if ( this != &c )
        {
          muscle = c.muscle;
          start = c.start;
          last = c.last;
        }
        return *this;
      }
      //----------------------------------------------------
      bool  operator<  (const Control &c) const
      { return start < c.start; }
      bool  operator== (const Control &c) const
      {
        return  (muscle == c.muscle)
             && (start == c.start)
             && (last == c.last);
      }
      bool  operator== (const MusclesEnum m) const
      { return  (muscle == m); }
      bool  operator!= (const Control &c) const
      {
        return  (muscle != c.muscle)
             || (start != c.start)
             || (last != c.last);
      }
      bool  operator!= (const MusclesEnum m) const
      { return  (muscle != m); }
      //----------------------------------------------------
      template<class Archive>
      void  serialize (Archive & ar, const unsigned int version)
      { ar & muscle & start & last; }
      //----------------------------------------------------
      operator tstring () const
      {
        tstringstream ss;
        ss << muscle << _T(" ") << start << _T(" ") << last;
        return  ss.str ();
      }
    };
    //----------------------------------------------------
    template <class Iter>
    frames_t  move (IN Iter begin, IN Iter end, OUT std::list<Point> *visited=NULL) throw (...)
    {
      frames_t  frame = 0U;
      frames_t  actual_last = 0U;

      if ( !std::is_sorted (begin, end) )
        throw new std::exception ("Controls are not sorted");

      /* Исключить незадействованные двигатели */
      MusclesEnum control_muscles = Hand::EmptyMov;
      for ( Iter it = begin; it != end; ++it )
      { control_muscles = control_muscles | it->muscle; }

      if ( std::none_of (begin, end, [](const Hand::Control &c) { return (c.last > 0); }) )
      { return  0U; }

      /* Simultaniusly moving */
      if ( musclesCumul & control_muscles )
      { /*  Что-то должно двигаться, иначе бесконечный цикл. */
        Iter  iter = begin;
        for ( frames_t time = iter->start; iter != end; ++time )
        {
          if ( time == iter->start )
          { /* Если стартуют одновременно, но разной продолжительности. */
            while ( iter != end && time == iter->start )
            {
              step (iter->muscle, iter->last);
              if ( iter != begin )
                ++actual_last;
              ++iter;
              if ( visited && !(frame % visitedSaveEach) )
                visited->push_back (position);
            }
          }
          else
          {
            step ();
            ++actual_last;
            if ( visited && !(frame % visitedSaveEach) )
              visited->push_back (position);
          }
          ++frame;
        } // end for

        while ( !hs.moveEnd_ )
        {
          step ();
          ++actual_last;
          if ( visited && !(frame % visitedSaveEach) )
            visited->push_back (position);
          ++frame;
        }
        if ( visited )
          visited->push_back (position);
      } // end if
      return  actual_last;
    }
    frames_t  move (IN std::initializer_list<Control> controls, OUT std::list<Point> *visited=NULL) throw (...);
    frames_t  move (IN MusclesEnum muscle, IN frames_t last);
    frames_t  move (IN MusclesEnum muscle, IN frames_t last, OUT std::list<Point> &visited);
    //----------------------------------------------------
    bool  timeValidStartOppositeMuscle (IN  MusclesEnum muscle);
    //----------------------------------------------------
    void  step (IN MusclesEnum muscle=EmptyMov, IN frames_t last=0U);
    //----------------------------------------------------
    const Point&  jointPosition (IN Hand::JointsEnum joint) const throw (...)
    {
      switch ( joint )
      {
        case Hand::Clvcl: return clavicle_;
        case Hand::Shldr: return shoulder_;
        case Hand::Elbow: return      arm_;
        case Hand::Wrist: return     hand_;
        default: throw new std::exception ("Inorrect joint");  // _T ??
      }
    }
    //----------------------------------------------------
    //----------------------------------------------------
    typedef std::map<Hand::JointsEnum, double> JointsSet;

    /*  jointsOpenPercent = { Clvcl, Shldr, Elbow, Wrist } < 100.0 %  */
    void  set (IN Hand::JointsSet &jointsOpenPercent);
    void  reset ();
    //----------------------------------------------------
    std::vector<const Point*>  jointsPositions () const;
    //----------------------------------------------------
    MusclesEnum  selectControl (IN size_t choose) const
    { return  (choose < controls.size ()) ? controls[choose] : Hand::EmptyMov; }
    MusclesEnum  selectControl (IN MusclesEnum  muscle=Hand::EmptyMov) const;
    //----------------------------------------------------
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
#define SET_DEFAULT  set(   NewHand::Hand::JointsSet    \
                         { {NewHand::Hand::Clvcl,  0.}, \
                           {NewHand::Hand::Shldr,  0.}, \
                           {NewHand::Hand::Elbow, 70.}, \
                           {NewHand::Hand::Wrist, 50.}  \
                         });
  //------------------------------------------------------------------------------
};
//------------------------------------------------------------------------------
#endif // _NEW_HAND_H_
