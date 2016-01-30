#include "StdAfx.h"
#include "MyWindow.h"
#include "NewHand.h"

#include <boost/algorithm/cxx11/none_of.hpp>
//--------------------------------------------------------------------------------
NewHand::Hand::frames_t  NewHand::Hand::maxMuscleLast (MusclesEnum  muscle)
{
  uint_t last = 0U;
  for ( auto m : muscles_ )
  {
    if ( m & muscle )
      switch ( m )
      {
        case ClvclOpn: last = last ? min (last, maxMoveFrames[ClvclIndex]) : maxMoveFrames[ClvclIndex]; break;
        case ClvclCls: last = last ? min (last, maxMoveFrames[ClvclIndex]) : maxMoveFrames[ClvclIndex]; break;
        case ShldrOpn: last = last ? min (last, maxMoveFrames[ShldrIndex]) : maxMoveFrames[ShldrIndex]; break;
        case ShldrCls: last = last ? min (last, maxMoveFrames[ShldrIndex]) : maxMoveFrames[ShldrIndex]; break;
        case ElbowOpn: last = last ? min (last, maxMoveFrames[ElbowIndex]) : maxMoveFrames[ElbowIndex]; break;
        case ElbowCls: last = last ? min (last, maxMoveFrames[ElbowIndex]) : maxMoveFrames[ElbowIndex]; break;
        case WristOpn: last = last ? min (last, maxMoveFrames[WristIndex]) : maxMoveFrames[WristIndex]; break;
        case WristCls: last = last ? min (last, maxMoveFrames[WristIndex]) : maxMoveFrames[WristIndex]; break;
      }
  }
  return last;
}
//--------------------------------------------------------------------------------
/* ??? ����������� �������������� �������� */
double  NewHand::Hand::nextFrame   (MusclesEnum muscle, frames_t frame, bool atStop)
{
  const bool USE_SPEED = true;
  const bool USE_WIND = false;
  //------------------------------------------------

  /* !!!  frame  IS  this->hs.lasts_[index]  */
  auto  frameUseSpeed = [this, atStop, USE_SPEED](size_t index)
  { double Frame;
    do
    {
      Frame = (atStop) ? this->framesStop[index][this->hs.lasts_[index]] :
                         this->framesMove[index][this->hs.lasts_[index]];
  
    } while ( USE_SPEED && atStop
             && (  this->hs.prevFrame_[index] < Frame)
             && (++this->hs.lasts_    [index] < this->minStopFrames[index]) );
    return Frame;
  };

  //------------------------------------------------
  JointsIndexEnum index = jointIndex (jointByMuscle (muscle));
  double Frame = hs.prevFrame_[index] = frameUseSpeed (index);

  // ??? hs.velosity_;

  // --- WIND ----------------------------------
  if ( USE_WIND && !frame )
  { 
    auto MAX = *std::max_element (framesMove[index].begin (), framesMove[index].end ());
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

  switch ( muscle )
  { 
    case ClvclOpn: shiftClvcl = (hs.shiftClvcl_ + Frame > maxClvclShift ) ? (maxClvclShift - hs.shiftClvcl_) :  Frame; break;
    case ClvclCls: shiftClvcl = (hs.shiftClvcl_ - Frame < 0.0           ) ? (          0.0 - hs.shiftClvcl_) : -Frame; break;
    case ShldrOpn: angleShldr = (hs.angleShldr_ - Frame < 1.0           ) ? (          1.0 - hs.angleShldr_) : -Frame; break;
    case ShldrCls: angleShldr = (hs.angleShldr_ + Frame > maxShldrAngle ) ? (maxShldrAngle - hs.angleShldr_) :  Frame; break;
    case ElbowOpn: angleElbow = (hs.angleElbow_ - Frame < 1.0           ) ? (          1.0 - hs.angleElbow_) : -Frame; break;
    case ElbowCls: angleElbow = (hs.angleElbow_ + Frame > maxElbowAngle ) ? (maxElbowAngle - hs.angleElbow_) :  Frame; break;
    case WristOpn: angleWrist = (hs.angleWrist_ - Frame < 1.0           ) ? (          1.0 - hs.angleWrist_) : -Frame; break;
    case WristCls: angleWrist = (hs.angleWrist_ + Frame > maxWristAngle ) ? (maxWristAngle - hs.angleWrist_) :  Frame; break;
  }

  hs.curPosShldr_.x -= shiftClvcl;
  hs.curPosArm_.x   -= shiftClvcl;
  hs.curPosHand_.x  -= shiftClvcl;

  hs.curPosArm_ .rotate (hs.curPosShldr_, angleShldr);
  hs.curPosHand_.rotate (hs.curPosShldr_, angleShldr);
  hs.curPosHand_.rotate (hs.curPosArm_,   angleElbow);

  hs.angleElbow_ += angleElbow;
  hs.angleShldr_ += angleShldr;
  hs.shiftClvcl_ += shiftClvcl;

  return (shiftClvcl || angleShldr || angleElbow || angleWrist);
}
//--------------------------------------------------------------------------------
void    NewHand::Hand::muscleFinal (JointsIndexEnum jointIndex)
{
  std::function<bool (frames_t)> NonZero = [](frames_t F) { return (F != 0U); };
  //-------------------------------------------------------
  hs.lastsMove[jointIndex] = 0U;
  hs.lastsStop[jointIndex] = 0U;
  hs.lasts_   [jointIndex] = 0U;

  /* ��������� ������������� ��������� */
  hs.musclesMove_ = hs.musclesMove_ ^ muscleByJoint (static_cast<JointsEnum> (1U << jointIndex), true);
  hs.musclesMove_ = hs.musclesMove_ ^ muscleByJoint (static_cast<JointsEnum> (1U << jointIndex), false);

  if ( boost::algorithm::none_of (hs.lastsMove, NonZero)
    && boost::algorithm::none_of (hs.lastsStop, NonZero) )
  { /* ������ ��������� ���� */
    hs.moveEnd_ = true;
    hs.musclesMove_ = Hand::EmptyMov;
  }
}
void    NewHand::Hand::muscleBrake (JointsIndexEnum jointIndex)
{
  hs.lastsMove[jointIndex] = 0U;
  hs.lastsStop[jointIndex] = minStopFrames[jointIndex];
  hs.lasts_[jointIndex] = 0U;
}
void    NewHand::Hand::muscleMove  (JointsIndexEnum jointIndex, MusclesEnum muscle, frames_t last, bool control)
{
  if ( !control && !hs.lastsMove[jointIndex] && !hs.lastsStop[jointIndex] )
    return;

  hs.moveEnd_ = false;
  //-------------------------------------------------------
  if ( control && !hs.lastsMove[jointIndex] )
    //  && (!hs.lastsMove[jointIndex + 1U] && !(jointIndex % 2)
    //   || !hs.lastsMove[jointIndex - 1U] &&  (jointIndex % 2)) )
  { /* ������ �������� ���� */
    auto lastMax = maxMoveFrames[jointIndex];
    hs.lastsMove[jointIndex] = last ? min (last, lastMax) : lastMax;
    hs.lasts_   [jointIndex] = 0U;
  }
  else if ( hs.lastsMove[jointIndex] && control )
  { /* ��������� �������� - �� ������� */
    muscleBrake (jointIndex);
  }
  else if ( hs.lastsMove[jointIndex] && hs.lasts_[jointIndex] >= maxMoveFrames[jointIndex] )
  { /* ��������� �������� - �� ��������� ��������� ������� */
    muscleFinal (jointIndex);
  }
  else if ( hs.lastsMove[jointIndex] && hs.lasts_[jointIndex] >= hs.lastsMove[jointIndex] )
  { /* ��������� ��������� �������� - �� ��������� �������� ������������ */
    muscleBrake (jointIndex);
  }
  else if ( hs.lastsMove[jointIndex] && hs.lasts_[jointIndex] <  hs.lastsMove[jointIndex] )
  { /* ����������� �������� */
    if ( !muscleFrame (muscle, hs.lasts_[jointIndex], false) )
    { /* ������������� ��������� */
      muscleFinal (jointIndex);
    } // end if
  } // end else if

  //-------------------------------------------------------
  /* �������� �� ������� */
  if (   (hs.lastsStop[jointIndex])
      && (hs.lasts_   [jointIndex] >= hs.lastsStop[jointIndex]
       || !muscleFrame (muscle, hs.lasts_[jointIndex], true)) )
  { /* ����� ������� �������� */
    muscleFinal (jointIndex);
  } // end if
  //-------------------------------------------------------

  /* Time is moving forward! */
  ++hs.lasts_[jointIndex];
}
//--------------------------------------------------------------------------------
// ?? min velosity???
// current velosity ??

std::vector<double>  NewHand::generateMoveFrames (double a, double b, size_t n)
{
  std::vector<double> frames (n--);

  double  sum = 0.;
  for ( size_t i = 0U; i <= n; ++i )
  { /* diff velosity on start and end */
    frames[i] = (1. - cos (i * M_PI / n)) * 0.5;
    sum += frames[i]; /* get normalization */
  }

  /* apply normalization */
  for ( size_t i = 0U; i <= n; ++i )
  { frames[i] = frames[i] * (b - a) / sum + a; }

  return frames;
}
std::vector<double>  NewHand::generateStopFrames (double a, double b, size_t n)
{
  std::vector<double> frames (n--);

  double  sum = 0.;
  for ( size_t i = 0U; i <= n; ++i )
  {
    // frames[i] = (a + b + (a - b)*cos ( ((n / 2. + (i+1) / 2.) * M_PI) / n)) / 2.;
    double d = double (i) / n;
    frames[i] = (1. - cos ((d + 1.) * M_PI)) * 0.5;
    sum += frames[i];
  }

  for ( size_t i = 0U; i <= n; ++i )
  {
    frames[i] = frames[i] * (b - a) / sum + a;
  }

  return frames;
}

NewHand::Hand::Hand (const Point &palm,
                     const Point &hand, const Point &arm,
                     const Point &sholder, const Point &clavicle,
                     const std::vector< JointsEnum>  joints,
                     const std::vector<MusclesEnum> muscles
                    ) :
                     maxClvclShift (0.40),
                     maxShldrAngle (105U),
                     maxElbowAngle (135U),
                     maxWristAngle (100U),
                     
                     maxMoveFrames ({ 30U /* ClvclIndex */ , 60U /* ShldrIndex */ , 
                                      55U /* ElbowIndex */ , 30U /* WristIndex */ }),
                     minStopFrames ({ 15U /* ClvclIndex */ , 15U /* ShldrIndex */ , 
                                      15U /* ElbowIndex */ , 15U /* WristIndex */ }),

                     joints_ (joints), muscles_ (muscles),
                     jointsCount (joints.size ()),
                     musclesCount (muscles.size ()),
                     palm_ (palm), hand_ (hand), arm_ (arm),
                     sholder_ (sholder), clavicle_ (clavicle)
{ // 
  // std::copy ( joints.begin (),  joints.end (), std::begin( joints_));
  // std::copy (muscles.begin (), muscles.end (), std::begin(muscles_));

  framesMove[ClvclIndex] = generateMoveFrames (EPS, maxClvclShift, maxMoveFrames[ClvclIndex]);
  framesMove[ShldrIndex] = generateMoveFrames (EPS, maxShldrAngle, maxMoveFrames[ShldrIndex]);
  framesMove[ElbowIndex] = generateMoveFrames (EPS, maxElbowAngle, maxMoveFrames[ElbowIndex]);

  // 20% �� ������ �������
  framesStop[ClvclIndex] = generateStopFrames (EPS, maxClvclShift * 0.02, minStopFrames[ClvclIndex]);
  framesStop[ShldrIndex] = generateStopFrames (EPS, maxShldrAngle * 0.02, minStopFrames[ShldrIndex]);
  framesStop[ElbowIndex] = generateStopFrames (EPS, maxElbowAngle * 0.02, minStopFrames[ElbowIndex]);
  
  reset ();
}
void  NewHand::Hand::step (MusclesEnum muscle, frames_t last)
{
  if ( muscle )
  {
    MusclesEnum ms = Hand::EmptyMov;
    for ( auto m : muscles_ ) { ms = ms | m; }
    muscle = ms & muscle;

    if ( muscleValidAtOnce (hs.musclesMove_ | muscle) )
    { hs.musclesMove_ = hs.musclesMove_ | muscle; }
    else  return;
  }

  if ( hs.musclesMove_ )
    for ( auto j : joints_ )
    {
      MusclesEnum m;
      m = muscleByJoint (j, true); // open
      if ( m & hs.musclesMove_ )
      {
        muscleMove (jointIndex (j), m, last, (m & muscle) != 0U);
        continue;
      }

      m = muscleByJoint (j, false); // close
      if ( m & hs.musclesMove_ )
      {
        muscleMove (jointIndex (j), m, last, (m & muscle) != 0U);
      }
    }
}
void  NewHand::Hand::move (MusclesEnum muscle, frames_t last, std::list<Point> &visited)
{
  if ( last )
  {
    MusclesEnum ms = Hand::EmptyMov;
    for ( auto m : muscles_ ) { ms = ms | m; }
    muscle = ms & muscle;

    /* START! */
    step (muscle, last);
    visited.push_back (position);
    // std::wcout << std::endl << tstring (position) << std::endl;

    while ( muscle && !hs.moveEnd_ )
    { /* just moving */
      step ();
      visited.push_back (position);
      // std::wcout << tstring (position) << std::endl;
    }

  }
}
void  NewHand::Hand::move (MusclesEnum muscle, frames_t last) // !simultaniusly
{
  if ( last )
  {
    /* ��������� ����������������� ��������� */
    MusclesEnum ms = Hand::EmptyMov;
    for ( auto m : muscles_ ) { ms = ms | m; }
    muscle = ms & muscle;

    step (muscle, last);
    /* ���-�� ������ ���������, ����� ����.���� */
    while ( muscle && !hs.moveEnd_ )
      step ();
  }
}

void  NewHand::Hand::reset ()
{
  //-----------------------------------------------------------------
  hs.curPosClvcl_ = clavicle_;
  hs.curPosShldr_ = sholder_;
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
void  NewHand::Hand::set (JointsEnum joint, const std::array<double, Hand::JointsCount> &jOp)
{
  if ( joint )
  {
    int index = 0;
    double  shiftClvcl, angleShldr, angleElbow, angleWrist;
    //------------------------------------------------------
    if ( joint & Clvcl )
    { // ?? index --> ClvclIndex
      shiftClvcl = (jOp[index] > 100.0) ? 100.0 : jOp[index];
      shiftClvcl = shiftClvcl / 100.0 * maxClvclShift;
      ++index;
    }
    else shiftClvcl = hs.shiftClvcl_;

    if ( joint & Shldr )
    { // ?? index --> ShldrIndex
      angleShldr = (jOp[index] > 100.0) ? 100.0 : jOp[index];
      angleShldr = angleShldr / 100.0 * maxShldrAngle;
      ++index;
    }
    else angleShldr = hs.angleShldr_;

    if ( joint & Elbow )
    { // ?? index --> ElbowIndex
      angleElbow = (jOp[index] > 100.0) ? 100.0 : jOp[index];
      angleElbow = angleElbow / 100.0 * maxElbowAngle;
      ++index;
    }
    else angleElbow = hs.angleElbow_;

    if ( joint & Wrist )
    { // ?? index --> WristIndex
      angleWrist = (jOp[index] > 100.0) ? 100.0 : jOp[index];
      angleWrist = angleElbow / 100.0 * maxWristAngle;
      ++index;
    }
    else angleWrist = hs.angleWrist_;
    //------------------------------------------------------
    reset ();
    //------------------------------------------------------
    hs.curPosShldr_ .x -= shiftClvcl;
    hs.curPosArm_   .x -= shiftClvcl;
    hs.curPosHand_  .x -= shiftClvcl;
    // TODO: !!! hs.curPosPalm_

    hs.curPosArm_ .rotate (hs.curPosShldr_, angleShldr);
    hs.curPosHand_.rotate (hs.curPosShldr_, angleShldr);
    hs.curPosHand_.rotate (hs.curPosArm_,   angleElbow);
    // TODO: !!! hs.curPosPalm_
    //------------------------------------------------------
    hs.shiftClvcl_ = shiftClvcl;
    hs.angleShldr_ = angleShldr;
    hs.angleElbow_ = angleElbow;
    hs.angleWrist_ = angleWrist;
  }
}
//--------------------------------------------------------------------------------
NewHand::Hand::MusclesIndexEnum  NewHand::Hand::muscleIndex (NewHand::Hand::MusclesEnum  muscle) const
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
NewHand::Hand::JointsIndexEnum   NewHand::Hand::jointIndex  (NewHand::Hand:: JointsEnum  joint ) const
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
void  NewHand::Hand::draw (HDC hdc, HPEN hPen, HBRUSH hBrush) const
{ 
  //--- draw constants ----------------------------------------------
  const double  CircleRadius = 0.03;
  const double  WidthJoint = 0.01;
  //-----------------------------------------------------------------
  HPEN   hPen_old   =   (HPEN) SelectObject (hdc, hPen);
  HBRUSH hBrush_old = (HBRUSH) SelectObject (hdc, hBrush);
  //-----------------------------------------------------------------
  Point c (   clavicle_ ), s (hs.curPosShldr_),
        a (hs.curPosArm_), h (hs.curPosHand_ );

  Point su (sholder_.x + WidthJoint - hs.shiftClvcl_, sholder_.y + WidthJoint),
        sd (sholder_.x - WidthJoint - hs.shiftClvcl_, sholder_.y - WidthJoint),
        au (    arm_.x + WidthJoint - hs.shiftClvcl_,     arm_.y + WidthJoint),
        ad (    arm_.x - WidthJoint - hs.shiftClvcl_,     arm_.y - WidthJoint),
        hu (   hand_.x + WidthJoint - hs.shiftClvcl_,    hand_.y + WidthJoint),
        hd (   hand_.x - WidthJoint - hs.shiftClvcl_,    hand_.y - WidthJoint);

  //-----------------------------------------------------------------
  su.rotate (s, hs.angleShldr_);
  sd.rotate (s, hs.angleShldr_);

  au.rotate (s, hs.angleShldr_);
  ad.rotate (s, hs.angleShldr_);

  hu.rotate (s, hs.angleShldr_);
  hd.rotate (s, hs.angleShldr_);
  hu.rotate (a, hs.angleElbow_);
  hd.rotate (a, hs.angleElbow_);

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

  //---clavicle-------------------------------------------------------
  DrawCircle (hdc, c, CircleRadius);

  //---sholder--------------------------------------------------------
  DrawCircle (hdc, s, CircleRadius);

  //---arm--------------------- --------------------------------------
  DrawCircle (hdc, a, CircleRadius);

  //---hand-------------------- --------------------------------------
  DrawCircle (hdc, h, CircleRadius);

  //------------------------------------------------------------------
  // �������� �����
  SelectObject (hdc, hPen_old);
  SelectObject (hdc, hBrush_old);
}
//--------------------------------------------------------------------------------
std::vector<const Point*>  NewHand::Hand::points () const
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
