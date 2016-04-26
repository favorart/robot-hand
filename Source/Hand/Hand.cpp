#include "StdAfx.h"
#include "WindowHeader.h"
#include "Hand.h"

#define HAND_VER 2
#include "HandMuscles.h"
//--------------------------------------------------------------------------------
NewHand::Hand::frames_t  NewHand::Hand::maxMuscleLast (IN MusclesEnum  muscle)
{
  uint_t last = 0U;
  for ( auto m : muscles_ )
  {
    if ( m & muscle )
      switch ( m )
      {
        case ClvclOpn: last = last ? std::min (last, maxMoveFrames[ClvclIndex]) : maxMoveFrames[ClvclIndex]; break;
        case ClvclCls: last = last ? std::min (last, maxMoveFrames[ClvclIndex]) : maxMoveFrames[ClvclIndex]; break;
        case ShldrOpn: last = last ? std::min (last, maxMoveFrames[ShldrIndex]) : maxMoveFrames[ShldrIndex]; break;
        case ShldrCls: last = last ? std::min (last, maxMoveFrames[ShldrIndex]) : maxMoveFrames[ShldrIndex]; break;
        case ElbowOpn: last = last ? std::min (last, maxMoveFrames[ElbowIndex]) : maxMoveFrames[ElbowIndex]; break;
        case ElbowCls: last = last ? std::min (last, maxMoveFrames[ElbowIndex]) : maxMoveFrames[ElbowIndex]; break;
        case WristOpn: last = last ? std::min (last, maxMoveFrames[WristIndex]) : maxMoveFrames[WristIndex]; break;
        case WristCls: last = last ? std::min (last, maxMoveFrames[WristIndex]) : maxMoveFrames[WristIndex]; break;
      }
  }
  return last;
}
//--------------------------------------------------------------------------------
// ??? minimum velosity
// ??? current velosity

// ??? momentum velosity
/* ??? Зависимость от сил, масс, трения и моментов инерции */
/* ??? Коэффициент взаимодействия приводов */
/* ??? Торможение включением противоположного двигателя */
double  NewHand::Hand::nextFrame   (MusclesEnum muscle, frames_t frame, bool atStop)
{
  const bool USE_SPEED = true;
  const bool USE_WIND = false;
  //------------------------------------------------
  MusclesIndexEnum  MuscleIndex = muscleIndex (muscle);

  /* !!!  frame  IS  this->hs.lasts_[muscleIndex (muscle)]  */
  auto  frameUseSpeed = [this, atStop, USE_SPEED](JointsIndexEnum  JointIndex, 
                                                 MusclesIndexEnum  MuscleIndex)
  {
    double Frame;
    // try {
    do
    {
      Frame = (atStop) ? this->framesStop[JointIndex][this->hs.lasts_[MuscleIndex]] :
                         this->framesMove[JointIndex][this->hs.lasts_[MuscleIndex]];

    } while ( USE_SPEED && atStop
          && (this->hs.prevFrame_[MuscleIndex] < Frame)
          && (  ++this->hs.lasts_[MuscleIndex] < this->minStopFrames[JointIndex]) );

    // } catch (...) { Frame = 0.; }

    // double Frame  = (atStop) ? this->framesStop[JointIndex][this->hs.lasts_[MuscleIndex]] :
    //                            this->framesMove[JointIndex][this->hs.lasts_[MuscleIndex]];
    // 
    // if ( USE_SPEED && atStop && this->hs.prevFrame_[MuscleIndex] < Frame )
    //                     Frame = this->hs.prevFrame_[MuscleIndex];

    return Frame;
  };

  //------------------------------------------------
  JointsIndexEnum JointIndex = jointIndex (jointByMuscle (muscle));
  double Frame = hs.prevFrame_[MuscleIndex] = frameUseSpeed (JointIndex, MuscleIndex);
  // ??? hs.velosity_;

  // --- WIND ----------------------------------
  if ( USE_WIND && !frame )
  { 
    auto MAX = *std::max_element (framesMove[JointIndex].begin (), framesMove[JointIndex].end ());
    Frame = random (EPS, MAX);
  }
  // -------------------------------------------
  return Frame;
}
bool    NewHand::Hand::muscleFrame (MusclesEnum muscle, frames_t frame, bool atStop)
{
  double Frame = nextFrame (muscle, frame, atStop);

  double shiftClvcl = 0.;
  double angleShldr = 0.;
  double angleElbow = 0.;
  double angleWrist = 0.;

  double maxClvclShift = static_cast<double>(maxJointAngles[ClvclIndex]) / 100.;
  auto   maxShldrAngle = maxJointAngles[ShldrIndex];
  auto   maxElbowAngle = maxJointAngles[ElbowIndex];
  auto   maxWristAngle = maxJointAngles[WristIndex];

  switch ( muscle )
  { 
    case ClvclOpn: shiftClvcl = (hs.shiftClvcl_ + Frame > maxClvclShift) ? (maxClvclShift - hs.shiftClvcl_) :  Frame; break;
    case ClvclCls: shiftClvcl = (hs.shiftClvcl_ - Frame < 0.0          ) ? (          0.0 - hs.shiftClvcl_) : -Frame; break;
    case ShldrOpn: angleShldr = (hs.angleShldr_ - Frame < 1.0          ) ? (          1.0 - hs.angleShldr_) : -Frame; break;
    case ShldrCls: angleShldr = (hs.angleShldr_ + Frame > maxShldrAngle) ? (maxShldrAngle - hs.angleShldr_) :  Frame; break;
    case ElbowOpn: angleElbow = (hs.angleElbow_ - Frame < 1.0          ) ? (          1.0 - hs.angleElbow_) : -Frame; break;
    case ElbowCls: angleElbow = (hs.angleElbow_ + Frame > maxElbowAngle) ? (maxElbowAngle - hs.angleElbow_) :  Frame; break;
    case WristOpn: angleWrist = (hs.angleWrist_ - Frame < 1.0          ) ? (          1.0 - hs.angleWrist_) : -Frame; break;
    case WristCls: angleWrist = (hs.angleWrist_ + Frame > maxWristAngle) ? (maxWristAngle - hs.angleWrist_) :  Frame; break;
  }

  hs.curPosShldr_.x -= shiftClvcl;
  hs.curPosArm_.x   -= shiftClvcl;
  hs.curPosHand_.x  -= shiftClvcl;
  hs.curPosPalm_.x  -= shiftClvcl;

  hs.curPosArm_ .rotate (hs.curPosShldr_, angleShldr);

  hs.curPosHand_.rotate (hs.curPosShldr_, angleShldr);
  hs.curPosHand_.rotate (hs.curPosArm_,   angleElbow);

  hs.curPosPalm_.rotate (hs.curPosShldr_, angleShldr);
  hs.curPosPalm_.rotate (hs.curPosArm_,   angleElbow);
  hs.curPosPalm_.rotate (hs.curPosHand_,  angleWrist);

  hs.angleWrist_ += angleWrist;
  hs.angleElbow_ += angleElbow;
  hs.angleShldr_ += angleShldr;
  hs.shiftClvcl_ += shiftClvcl;

  return (shiftClvcl || angleShldr || angleElbow || angleWrist);
}
//--------------------------------------------------------------------------------
void    NewHand::Hand::muscleBrake (MusclesIndexEnum MuscleIndex, JointsIndexEnum JointIndex)
{
  hs.lastsMove[MuscleIndex] = 0U;
  hs.lastsStop[MuscleIndex] = minStopFrames[JointIndex];
  hs.lasts_   [MuscleIndex] = 0U;
}
void    NewHand::Hand::muscleFinal (MusclesIndexEnum MuscleIndex, MusclesEnum muscle)
{
  /* может производиться торможение противоположным мускулом */
  if ( !hs.lastsMove[MuscleIndex] )
  {
    /* если движения нет и торможение трением завершилось */
    //-------------------------------------------------------
    std::function<bool (frames_t)> NonZero = [](frames_t F) { return (F != 0U); };
    //-------------------------------------------------------
    hs.lastsMove[MuscleIndex] = 0U;
    hs.lastsStop[MuscleIndex] = 0U;
    hs.lasts_   [MuscleIndex] = 0U;
    //-------------------------------------------------------
    /* исключаем остановленный двигатель */
    hs.musclesMove_ = hs.musclesMove_ ^ muscle;

    /* проверяем, что остальные двигатели уже остановились */
    if ( boost::algorithm::none_of (hs.lastsMove, NonZero)
      && boost::algorithm::none_of (hs.lastsStop, NonZero) )
    { /* Полная остановка руки */
      hs.moveEnd_ = true;
      hs.musclesMove_ = Hand::EmptyMov;
    }
    //-------------------------------------------------------
  }
}
void    NewHand::Hand::muscleMove  (MusclesIndexEnum MuscleIndex, MusclesEnum muscle,
                                    frames_t last, bool control)
{
  /* если не производится никакого движения и нет сигнала о начале нового */
  if ( !control && !hs.lastsMove[MuscleIndex] && !hs.lastsStop[MuscleIndex] )
    return;
  //-------------------------------------------------------
  hs.moveEnd_ = false;

  JointsIndexEnum  JointIndex = jointIndex (jointByMuscle (muscle));
  //-------------------------------------------------------
  if ( control && !hs.lastsMove[MuscleIndex] )
  { /* начало нового движения руки */
    if ( hs.lasts_[MuscleIndex] )
    {
      /* сохранить примерную скорость движения - инерция! */
      Hand::frames_t  frame = 0U;
      while ( hs.prevFrame_[MuscleIndex] > framesMove[JointIndex][frame] )
      /* framesStop[JointIndex][hs.lasts_[muscleIndex]] */
      { ++frame; }

      hs.lasts_[MuscleIndex] = (frame > 5U) ? (frame - 5U) : 0U;
    }
    else
    { hs.lasts_[MuscleIndex] = 0U; }

    auto  lastMax = maxMoveFrames[JointIndex] - hs.lasts_[MuscleIndex];
    hs.lastsMove[MuscleIndex] = last ? std::min (last, lastMax) : lastMax;
    hs.lastsStop[MuscleIndex] = 0U;
  }
  else if ( hs.lastsMove[MuscleIndex] && control )
  { /* остановка движения - по сигналу */
    muscleBrake (MuscleIndex, JointIndex);
  }
  else if ( hs.lastsMove[MuscleIndex] && hs.lasts_[MuscleIndex] >= maxMoveFrames[JointIndex] )
  { /* остановка движения - по истечении доступных фреймов */
    muscleBrake (MuscleIndex, JointIndex);  // muscleFinal (MuscleIndex, muscle);
  }
  else if ( hs.lastsMove[MuscleIndex] && hs.lasts_[MuscleIndex] >= hs.lastsMove[MuscleIndex] )
  { /* остановка основного движения - по истечении заданной длительности */
    muscleBrake (MuscleIndex, JointIndex);
  }
  else if ( hs.lastsMove[MuscleIndex] && hs.lasts_[MuscleIndex] <  hs.lastsMove[MuscleIndex] )
  { /* продолжение движения */
    if ( !muscleFrame (muscle, hs.lasts_[MuscleIndex], false) )
    { /* Окончательная остановка */
      muscleBrake (MuscleIndex, JointIndex);  // muscleFinal (MuscleIndex, muscle);
    } // end if
  } // end else if

  //-------------------------------------------------------
  /* Движение по инерции */
  if (   (hs.lastsStop[MuscleIndex])
      && (hs.lasts_   [MuscleIndex] >= hs.lastsStop[MuscleIndex]
       || !muscleFrame (muscle, hs.lasts_[MuscleIndex], true)) )
  { /* Конец полного движения */
    muscleFinal (MuscleIndex, muscle);
  } // end if
  //-------------------------------------------------------

  /* Time is moving forward! */
  ++hs.lasts_[MuscleIndex];
}
//--------------------------------------------------------------------------------
NewHand::Hand::Hand (IN const Point &palm,
                     IN const Point &hand,     IN const Point &arm,
                     IN const Point &shoulder, IN const Point &clavicle,
                     IN const JointsMotionLaws &joints_frames ) throw (...) :
                     
                     StopDistaceRatio (0.5), // 50% от общего пробега
                     maxJointAngles ({  40U /* maxClvclShift */ , 105U /* maxShldrAngle */ ,
                                       135U /* maxElbowAngle */ , 100U /* maxWristAngle */ }),
                     maxMoveFrames ({  600U /* ClvclIndex */    , 700U /* ShldrIndex */ ,
                                       550U /* ElbowIndex */    , 350U /* WristIndex */ }),
                     visitedSaveEach (10),

                     palm_ (palm), hand_ (hand), arm_ (arm),
                     shoulder_ (shoulder), clavicle_ (clavicle),
                     drawPalm_ (false)
{ 
  for ( auto j : joints_frames )
  { joints_.push_back (j.first); }

  if ( !joints_frames.size () || joints_frames.size () > JointsCount )
    throw new std::exception ("Incorrect joints count"); // _T ??

  for ( auto joint : joints_ )
  {
    if ( joint & Wrist )
      drawPalm_ = true;

    muscles_.push_back (muscleByJoint (joint, true));
    muscles_.push_back (muscleByJoint (joint, false));

    JointsIndexEnum  jIndex = jointIndex (joint);
    minStopFrames[jIndex] = static_cast<frames_t> (maxMoveFrames[jIndex] * StopDistaceRatio);

    framesMove[jIndex].resize (maxMoveFrames[jIndex]);
    framesStop[jIndex].resize (minStopFrames[jIndex]);

    joints_frames.find (joint)->second.moveLaw->generate (framesMove[jIndex].begin (), maxMoveFrames[jIndex],
                                                          minFrameMove, maxJointAngles[jIndex]);

    double  maxVelosity = *boost::max_element (framesMove[jIndex]);

    joints_frames.find (joint)->second.stopLaw->generate (framesStop[jIndex].begin (), minStopFrames[jIndex],
                                                          minFrameMove, maxJointAngles[jIndex] * StopDistaceRatio,
                                                          maxVelosity);
    
    // ------------------------------------------------------------------------------------

    // #pragma warning (disable:4996)
    // {
    //   tstringstream ss;   
    //   ss << _T ("framesMove_") << j << _T (".txt");
    //   FILE *f = _wfopen (ss.str ().c_str (), _T ("w"));
    //   for ( auto d : framesMove[jIndex] )
    //     fwprintf (f, _T ("%lf "), d);
    //   fclose (f);
    // }
    // { tstringstream ss;
    //   ss << _T ("framesStop_") << j << _T (".txt");
    //   FILE *f = _wfopen (ss.str ().c_str (), _T ("w"));
    //   for ( auto d : framesStop[jIndex] )
    //     fwprintf (f, _T ("%lf "), d);
    //   fclose (f);
    // }

  }
  std::sort (joints_.begin (), joints_.end ());

  musclesCumul = Hand::EmptyMov;
  for ( auto real_muscle : muscles_ )
  { musclesCumul = musclesCumul | real_muscle; }

  createControls ();
  reset ();
}

bool  NewHand::Hand::timeValidStartOppositeMuscle (IN  MusclesEnum muscle)
{
  if ( !muscle ) 
  { return false; }
  //-------------------------
  for ( auto joint : joints_ )
  {
    auto mo = muscleByJoint (joint, true);
    auto mc = muscleByJoint (joint, false);
  
    if ( ((mo & hs.musclesMove_) && hs.lastsMove[jointIndex (joint)] && (mc & muscle))
      || ((mc & hs.musclesMove_) && hs.lastsMove[jointIndex (joint)] && (mo & muscle)) )
    { return false; }
  } // end for
  //-------------------------
  return true;
}

void                     NewHand::Hand::step (IN  MusclesEnum muscle, IN frames_t last)
{
  if ( muscle )
  {
    /* Исключить незадействованные двигатели */
    muscle = musclesCumul & muscle;
    /* Совместимо ли новое с тем, что уже сейчас движется? */
    if ( timeValidStartOppositeMuscle (muscle) )
    // if ( musclesValidUnion (hs.musclesMove_ | muscle) )
    { hs.musclesMove_ = hs.musclesMove_ | muscle; }
    /* Если нет выходим без изменений */
    else  return;
  }

  if ( hs.musclesMove_ )
  {
    for ( auto m : muscles_ )
    {
      if ( m & hs.musclesMove_ )
      { muscleMove (muscleIndex (m), m, last, (m & muscle) != 0U); }
    } // end for
  } // end if
}
NewHand::Hand::frames_t  NewHand::Hand::move (IN  MusclesEnum muscle, IN frames_t last)
{
  frames_t  actual_last = 0U;
  /* simultaniusly moving */
  if ( last )
  {
    step (muscle, last);
    /* Что-то должно двигаться, иначе беск.цикл */
    while ( hs.musclesMove_ && !hs.moveEnd_ )
    {
      step ();
      ++actual_last;
    }
  } // end if
  return  actual_last;
}
NewHand::Hand::frames_t  NewHand::Hand::move (IN  MusclesEnum muscle, IN frames_t last,
                                              OUT std::list<Point> &visited)
{
  frames_t  frame = 0U;
  frames_t  actual_last = 0U;
  /* simultaniusly moving */
  if ( last )
  {
    /* start movement */
    step (muscle, last);
    // visited.push_back (position);
    // tcout << std::endl << tstring (position) << std::endl;
    while ( hs.musclesMove_ && !hs.moveEnd_ )
    { /* just moving */
      step ();
      if ( !(frame % visitedSaveEach) )
        visited.push_back (position);
      // tcout << tstring (position) << std::endl;
      ++actual_last;
      ++frame;
    }
    visited.push_back (position);
  }
  return  actual_last;
}
NewHand::Hand::frames_t  NewHand::Hand::move (IN  std::initializer_list<Control> controls,
                                              OUT std::list<Point> *visited) throw (...)
{ return  move (controls.begin (), controls.end (), visited); }

void  NewHand::Hand::reset ()
{
  //-----------------------------------------------------------------
  hs.curPosClvcl_ = clavicle_;
  hs.curPosShldr_ = shoulder_;
  hs.curPosArm_   = arm_;
  hs.curPosHand_  = hand_;
  hs.curPosPalm_  = palm_;
  //-----------------------------------------------------------------
  hs.shiftClvcl_ = hs.angleShldr_ = 0.;
  hs.angleElbow_ = hs.angleWrist_ = 0.;
  //-----------------------------------------------------------------
  hs.prevFrame_.fill (EPS);

  hs.lasts_.fill (0U);
  //-----------------------------------------------------------------
  hs.lastsMove.fill (0U);
  hs.lastsStop.fill (0U);
  //-----------------------------------------------------------------
  hs.musclesMove_ = Hand::EmptyMov;
  //-----------------------------------------------------------------
  hs.moveEnd_ = false; 
  // hs.velosity_ = 0.;
}
void  NewHand::Hand::set   (IN NewHand::Hand::JointsSet &jointsOpenPercent)
                          //JointsEnum joint, const std::array<double, Hand::JointsCount> &jOp)
{
  if ( jointsOpenPercent.size () )
  {
    int index = 0;
    double  shiftClvcl, angleShldr, angleElbow, angleWrist;
    //------------------------------------------------------
    if ( jointsOpenPercent.find(Clvcl) != jointsOpenPercent.end () ) // joint & Clvcl )
    { // ?? index --> ClvclIndex
      double maxClvcl = static_cast<double>(maxJointAngles[ClvclIndex]) / 100.; // maxClvclShift;

      shiftClvcl = (jointsOpenPercent[Clvcl] > 100.0) ? 100.0 : jointsOpenPercent[Clvcl];
      shiftClvcl = shiftClvcl / 100.0 * maxClvcl;
      ++index;
    }
    else shiftClvcl = hs.shiftClvcl_;

    if ( jointsOpenPercent.find (Shldr) != jointsOpenPercent.end () ) // joint & Shldr )
    { // ?? index --> ShldrIndex
      angleShldr = (jointsOpenPercent[Shldr] > 100.0) ? 100.0 : jointsOpenPercent[Shldr];
      angleShldr = angleShldr / 100.0 * maxJointAngles[ShldrIndex]; // maxShldrAngle;
      ++index;
    }
    else angleShldr = hs.angleShldr_;

    if ( jointsOpenPercent.find (Elbow) != jointsOpenPercent.end () ) // joint & Elbow )
    { // ?? index --> ElbowIndex
      angleElbow = (jointsOpenPercent[Elbow] > 100.0) ? 100.0 : jointsOpenPercent[Elbow];
      angleElbow = angleElbow / 100.0 * maxJointAngles[ElbowIndex]; // maxElbowAngle;
      ++index;
    }
    else angleElbow = hs.angleElbow_;

    if ( jointsOpenPercent.find (Wrist) != jointsOpenPercent.end () ) // joint & Wrist )
    { // ?? index --> WristIndex
      angleWrist = (jointsOpenPercent[Wrist] > 100.0) ? 100.0 : jointsOpenPercent[Wrist];
      angleWrist = angleElbow / 100.0 * maxJointAngles[WristIndex]; // maxWristAngle;
      ++index;
    }
    else angleWrist = hs.angleWrist_;
    //------------------------------------------------------
    reset ();
    //------------------------------------------------------
    hs.curPosShldr_ .x -= shiftClvcl;
    hs.curPosArm_   .x -= shiftClvcl;
    hs.curPosHand_  .x -= shiftClvcl;
    hs.curPosPalm_  .x -= shiftClvcl;

    hs.curPosArm_ .rotate (hs.curPosShldr_, angleShldr);

    hs.curPosHand_.rotate (hs.curPosShldr_, angleShldr);
    hs.curPosHand_.rotate (hs.curPosArm_,   angleElbow);

    hs.curPosPalm_.rotate (hs.curPosShldr_, angleShldr);
    hs.curPosPalm_.rotate (hs.curPosArm_,   angleElbow);
    hs.curPosPalm_.rotate (hs.curPosHand_,  angleWrist);
    //------------------------------------------------------
    hs.shiftClvcl_ = shiftClvcl;
    hs.angleShldr_ = angleShldr;
    hs.angleElbow_ = angleElbow;
    hs.angleWrist_ = angleWrist;
  }
}
//--------------------------------------------------------------------------------
NewHand::Hand::MusclesIndexEnum  NewHand::Hand::muscleIndex (IN MusclesEnum  muscle) const
{
  if ( !muscle ) return MusclesCount; // EmptyMov
  for ( MusclesEnum m : muscles_ )
  {
    if ( m & muscle )
      switch ( muscle )
      {
        case Hand::ClvclOpn: return Hand::ClvclOpnIndex;
        case Hand::ClvclCls: return Hand::ClvclClsIndex;
        case Hand::ShldrOpn: return Hand::ShldrOpnIndex;
        case Hand::ShldrCls: return Hand::ShldrClsIndex;
        case Hand::ElbowOpn: return Hand::ElbowOpnIndex;
        case Hand::ElbowCls: return Hand::ElbowClsIndex;
        case Hand::WristOpn: return Hand::WristOpnIndex;
        case Hand::WristCls: return Hand::WristClsIndex;
        // default: return MusclesCount;
      }
  }
  return MusclesCount;
}
NewHand::Hand::JointsIndexEnum   NewHand::Hand::jointIndex  (IN  JointsEnum  joint ) const
{
  if ( !joint ) return JointsCount; // Empty
  for ( JointsEnum j : joints_ )
  {
    if ( j & joint )
      switch ( joint )
      {
        case Hand::Clvcl: return Hand::ClvclIndex;
        case Hand::Shldr: return Hand::ShldrIndex;
        case Hand::Elbow: return Hand::ElbowIndex;
        case Hand::Wrist: return Hand::WristIndex;
        // default: return JointsCount;
      }
  }
  return JointsCount;
}
//--------------------------------------------------------------------------------
void  NewHand::Hand::draw (IN HDC hdc, IN HPEN hPen, IN HBRUSH hBrush) const
{ 
  //--- draw constants ----------------------------------------------
  const double    PalmRadius = 0.05;
  const double   JointRadius = 0.03;
  const double  SectionWidth = 0.01;
  
  //-----------------------------------------------------------------
  HPEN   hPen_old   =   (HPEN) SelectObject (hdc, hPen);
  HBRUSH hBrush_old = (HBRUSH) SelectObject (hdc, hBrush);
  //-----------------------------------------------------------------
  Point c (   clavicle_   ),
        s (hs.curPosShldr_), a (hs.curPosArm_ ),
        h (hs.curPosHand_ ), w (hs.curPosPalm_);

  Point su (shoulder_.x + SectionWidth - hs.shiftClvcl_, shoulder_.y + SectionWidth),
        sd (shoulder_.x - SectionWidth - hs.shiftClvcl_, shoulder_.y - SectionWidth),
        au (     arm_.x + SectionWidth - hs.shiftClvcl_,      arm_.y + SectionWidth),
        ad (     arm_.x - SectionWidth - hs.shiftClvcl_,      arm_.y - SectionWidth),
        hu (    hand_.x + SectionWidth - hs.shiftClvcl_,     hand_.y + SectionWidth),
        hd (    hand_.x - SectionWidth - hs.shiftClvcl_,     hand_.y - SectionWidth);

  Point wu, wd;
  if ( drawPalm_ )
  {
    wu = Point ( palm_.x + SectionWidth - hs.shiftClvcl_,  palm_.y + SectionWidth),
    wd = Point ( palm_.x - SectionWidth - hs.shiftClvcl_,  palm_.y - SectionWidth);
  }
  //-----------------------------------------------------------------
  su.rotate (s, hs.angleShldr_);
  sd.rotate (s, hs.angleShldr_);
  
  au.rotate (s, hs.angleShldr_);
  ad.rotate (s, hs.angleShldr_);

  hu.rotate (s, hs.angleShldr_);
  hd.rotate (s, hs.angleShldr_);
  hu.rotate (a, hs.angleElbow_);
  hd.rotate (a, hs.angleElbow_);

  if ( drawPalm_ )
  {
    wu.rotate (s, hs.angleShldr_);
    wd.rotate (s, hs.angleShldr_);
    wu.rotate (a, hs.angleElbow_);
    wd.rotate (a, hs.angleElbow_);
    wu.rotate (h, hs.angleWrist_);
    wd.rotate (h, hs.angleWrist_);
  }
  //-----------------------------------------------------------------
  MoveToEx (hdc, Tx (1.00), Ty (su.y), NULL);
  LineTo   (hdc, Tx (su.x), Ty (su.y));
  LineTo   (hdc, Tx (au.x), Ty (au.y));

  MoveToEx (hdc, Tx (1.00), Ty (sd.y), NULL);
  LineTo   (hdc, Tx (sd.x), Ty (sd.y));
  LineTo   (hdc, Tx (ad.x), Ty (ad.y));

  au.rotate (a, hs.angleElbow_);
  ad.rotate (a, hs.angleElbow_);

  MoveToEx (hdc, Tx (au.x), Ty (au.y), NULL);
  LineTo   (hdc, Tx (hu.x), Ty (hu.y));

  MoveToEx (hdc, Tx (ad.x), Ty (ad.y), NULL);
  LineTo   (hdc, Tx (hd.x), Ty (hd.y));

  if ( drawPalm_ )
  {
    //---palm-------------------- --------------------------------------
    DrawCircle (hdc, w, PalmRadius);

    //------------------------------------------------------------------
    hu.rotate (h, hs.angleWrist_);
    hd.rotate (h, hs.angleWrist_);

    LineTo (hdc, Tx (wd.x), Ty (wd.y));
    MoveToEx (hdc, Tx (hu.x), Ty (hu.y), NULL);
    LineTo (hdc, Tx (wu.x), Ty (wu.y));
  }
  //---clavicle-------------------------------------------------------
  DrawCircle (hdc, c, JointRadius);

  //---shoulder-------------------------------------------------------
  DrawCircle (hdc, s, JointRadius);

  //---arm--------------------- --------------------------------------
  DrawCircle (hdc, a, JointRadius);
  
  //---hand-------------------- --------------------------------------
  DrawCircle (hdc, h, JointRadius);
  
  //------------------------------------------------------------------
  // отменяем ручку
  SelectObject (hdc, hPen_old);
  SelectObject (hdc, hBrush_old);
}
//--------------------------------------------------------------------------------
std::vector<const Point*>  NewHand::Hand::jointsPositions () const
{
  std::vector<const Point*> vec;
  vec.push_back (&hs.curPosClvcl_);
  for ( auto j : joints_ )
  {
    if ( j == Clvcl )
      vec.push_back (&(hs.curPosShldr_));
    else if ( j == Shldr )
      vec.push_back (&(hs.curPosArm_));
    else if ( j == Elbow )
      vec.push_back (&(hs.curPosHand_));
    else if ( j == Wrist )
      vec.push_back (&(hs.curPosPalm_));
  }
  return vec;
}
//--------------------------------------------------------------------------------
void  NewHand::Hand::recursiveControlsAppend (NewHand::Hand::MusclesEnum  muscles,
                                              NewHand::Hand::JointsEnum   joints,
                                              size_t cur_deep, size_t max_deep)
{
  for ( auto j : joints_ )
  {
    if ( !(j & joints) && (j > joints) )
    {
      if ( cur_deep == max_deep )
      {
        controls.push_back (muscles | muscleByJoint (j, true ));
        controls.push_back (muscles | muscleByJoint (j, false));
      }
      else
      {
        recursiveControlsAppend (muscles | muscleByJoint (j, true ), j | joints, cur_deep + 1U, max_deep);
        recursiveControlsAppend (muscles | muscleByJoint (j, false), j | joints, cur_deep + 1U, max_deep);
      } // end else
    } // end if
  } // end for
};
void  NewHand::Hand::createControls ()
{
  for ( auto m : muscles_ )
  { controls.push_back (m); }

  /* 1 << 0,1,2,3,4,5,6,7 */
  /* Impossible (0 & 1) (2 & 3) (4 & 5) (6 & 7) */

  size_t    maxSimultMoves = joints_.size ();
  for ( size_t SimultMoves = 1U; SimultMoves < maxSimultMoves; ++SimultMoves )
    recursiveControlsAppend (Hand::EmptyMov, Hand::Empty, 0U, SimultMoves);
}
//--------------------------------------------------------------------------------
NewHand::Hand::MusclesEnum  NewHand::Hand::selectControl (IN NewHand::Hand::MusclesEnum  muscle)
{ 
  if ( !muscle )
    return controls[random (controls.size ())];
  else
  {
    for ( auto m : controls )
      if ( !(m & muscle) && musclesValidUnion (m | muscle) )
        return m;
    // auto m = controls[random (controls.size ())];
    // while ( (m & muscle) || !musclesValidUnion (m | muscle) )
    //   m = controls[random (controls.size ())];
  }
  return  EmptyMov;
}
//--------------------------------------------------------------------------------
