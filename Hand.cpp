#include "StdAfx.h"
#include "MyWindow.h"
#include "Hand.h"

using namespace OldHand;
//------------------------------------------------------------------------------
Hand::JointsEnum   OldHand::operator| (Hand::JointsEnum  j, Hand::JointsEnum  k)
{ return static_cast<Hand::JointsEnum> (static_cast<uchar_t> (j) | static_cast<uchar_t> (k)); }
Hand::JointsEnum   OldHand::operator& (Hand::JointsEnum  j, Hand::JointsEnum  k)
{ return static_cast<Hand::JointsEnum> (static_cast<uchar_t> (j) & static_cast<uchar_t> (k)); }

Hand::MusclesEnum  OldHand::operator| (Hand::MusclesEnum h, Hand::MusclesEnum k)
 { return static_cast<Hand::MusclesEnum> (static_cast<uchar_t> (h) | static_cast<uchar_t> (k)); }
Hand::MusclesEnum  OldHand::operator& (Hand::MusclesEnum h, Hand::MusclesEnum k)
{ return static_cast<Hand::MusclesEnum> (static_cast<uchar_t> (h) & static_cast<uchar_t> (k)); }

std::ostream&  operator<< (std::ostream &out, Hand::MusclesEnum muscle)
{
  if ( !muscle )  return  out << "Hand::EmptyMov ";

  for ( auto m : muscles )
  {
    if ( m & muscle )
      switch ( m )
      {
        // case Hand::EmptyMov: out << "Hand::EmptyMov "; break;
        case Hand::ClvclOpn: out << "Hand::ClvclOpn "; break;
        case Hand::ClvclCls: out << "Hand::ClvclCls "; break;
        case Hand::ShldrOpn: out << "Hand::ShldrOpn "; break;
        case Hand::ShldrCls: out << "Hand::ShldrCls "; break;
        case Hand::ElbowOpn: out << "Hand::ElbowOpn "; break;
        case Hand::ElbowCls: out << "Hand::ElbowCls "; break;
        case Hand::WristOpn: out << "Hand::WristOpn "; break;
        case Hand::WristCls: out << "Hand::WristCls "; break;
      }
  }
  return out;
}
std::ostream&  operator<< (std::ostream &out, Hand::JointsEnum   joint)
{
  if ( !joint )  return  out << "Hand::Empty ";

  for ( auto j : joints )
  {
    if ( j & joint )
      switch ( j )
      {
        // case Hand::Empty: out << "Hand::Empty "; break;
        case Hand::Clvcl: out << "Hand::Clvcl "; break;
        case Hand::Shldr: out << "Hand::Shldr "; break;
        case Hand::Elbow: out << "Hand::Elbow "; break;
        case Hand::Wrist: out << "Hand::Wrist "; break;
      }
  }
  return out;
}
//--------------------------------------------------------------------------------
uint_t  Hand::maxMuscleLast (MusclesEnum muscle)
{
  uint_t last = 0U;

  //uchar_t value = static_cast<uchar_t> (muscle);
  //for ( size_t i = 0; i < musclesCount; ++i, value >>= 1 )
  //for ( auto i : boost::irange (0U, musclesCount) )
  for (auto m : muscles_)
  {
    // auto m = muscle & (1 << i);
    if ( m & muscle )
      switch (m)
      {
        case EmptyMov: last = max (last, minJStopMoveFrames); break;
        case ClvclOpn: last = max (last, maxClvclMoveFrames); break;
        case ClvclCls: last = max (last, maxClvclMoveFrames); break;
        case ShldrOpn: last = max (last, maxShldrMoveFrames); break;
        case ShldrCls: last = max (last, maxShldrMoveFrames); break;
        case ElbowOpn: last = max (last, maxElbowMoveFrames); break;
        case ElbowCls: last = max (last, maxElbowMoveFrames); break;
        // case WristOpn: last = max (last, maxWristMoveFrames); break;
        // case WristCls: last = max (last, maxWristMoveFrames); break;
      }
  }
  return last;
}
//--------------------------------------------------------------------------------
bool  Hand::muscleMove (MusclesEnum hydNo, time_t time, bool atStop)
{ double frame = stepFrame (hydNo, time, atStop);

  double shiftClvcl = 0.0;
  double angleShldr = 0.0;
  double angleElbow = 0.0;
 double angleWrist = 0.0;
  
  switch (hydNo)
  { // default: SetError_ (ERR_NO);                                             break;
    case ClvclOpn : shiftClvcl = (shiftClvcl_ + frame > maxClvclShift) ? (maxClvclShift - shiftClvcl_ /*- EPS*/) :  frame; break; 
  case ClvclCls : shiftClvcl = (shiftClvcl_ - frame < 0.0          ) ? (          0.0 - shiftClvcl_ /*+ EPS*/) : -frame; break;
    case ShldrOpn : angleShldr = (angleShldr_ - frame < 1.0          ) ? (          1.0 - angleShldr_ /*+ EPS*/) : -frame; break;
    case ShldrCls : angleShldr = (angleShldr_ + frame > maxShldrAngle) ? (maxShldrAngle - angleShldr_ /*- EPS*/) :  frame; break;
    case ElbowOpn : angleElbow = (angleElbow_ - frame < 1.0          ) ? (          1.0 - angleElbow_ /*- EPS*/) : -frame; break; 
    case ElbowCls : angleElbow = (angleElbow_ + frame > maxElbowAngle) ? (maxElbowAngle - angleElbow_ /*- EPS*/) :  frame; break;
  // case WristOpn : angleWrist = (angleWrist  < 0.0           ) ? 0.0 :  frame; break;
  // case WristCls : angleWrist = (angleWrist  < 0.0           ) ? 0.0 : -frame; break;
  }

  // if ( shiftClvcl_ > maxClvclShift ) shiftClvcl_ = maxClvclShift;
  // if ( shiftClvcl_ < 0.0 ) shiftClvcl_ = 0.0;

  curPosShldr_.x -= shiftClvcl;
  curPosArm_  .x -= shiftClvcl;
  curPosHand_ .x -= shiftClvcl;

  curPosArm_ .rotate (curPosShldr_, angleShldr);
  curPosHand_.rotate (curPosShldr_, angleShldr);
  curPosHand_.rotate (curPosArm_,   angleElbow);
  
  angleElbow_ += angleElbow;
  angleShldr_ += angleShldr;
  shiftClvcl_ += shiftClvcl;

  return (shiftClvcl || angleShldr || angleElbow || angleWrist);
}
void  Hand::muscle     (uint_t no, bool control)
{
  time_t  time;
  //-------------------------------------------------------
  if ( !control && !timeBgn2OpenHyd_[no]
                && !timeEnd2OpenHyd_[no] )
    return;

  flagMovEnd_ = false;
  //-------------------------------------------------------
  time = time_ - timeBgn2OpenHyd_[no];
  //-------------------------------------------------------
  if ( control &&  !timeBgn2OpenHyd_[no]
               && (!timeBgn2OpenHyd_[no + 1] && !(no % 2)
               ||  !timeBgn2OpenHyd_[no - 1] &&  (no % 2)) )
  { // начало движения руки
    timeBgn2OpenHyd_[no] = time_;
  }
  else if ( control &&  timeBgn2OpenHyd_[no] )
  { // остановка движения - по сигналу
    timeBgn2OpenHyd_[no] = 0ULL;
    timeEnd2OpenHyd_[no] = time_;
  }
  else if ( time == tFrames2OpenHyd_[no] )
  { // остановка движения - по истечении доступных фреймов
    flagMovEnd_ = true;
    timeBgn2OpenHyd_[no] = 0ULL;
    timeEnd2OpenHyd_[no] = 0ULL;
  }
  else if ( timeBgn2OpenHyd_[no] && time < tFrames2OpenHyd_[no] )
  { // продолжение движения
    
    // time %= tFrames2OpenHyd_[no];
    if ( !muscleMove ((MusclesEnum) (1U << no), time, false) )
    {
      flagMovEnd_ = true;
      timeBgn2OpenHyd_[no] = 0ULL;
      timeEnd2OpenHyd_[no] = 0ULL;
    }
  } // end if

  time = time_ - timeEnd2OpenHyd_[no];
  //-------------------------------------------------------
  if ( timeEnd2OpenHyd_[no] && (time == minJStopMoveFrames
      || !muscleMove ((MusclesEnum) (1U << no), time, true)) )
  {
    flagMovEnd_ = true;
    timeBgn2OpenHyd_[no] = 0ULL;
    timeEnd2OpenHyd_[no] = 0ULL;
  } // end if
//-------------------------------------------------------
}
//--------------------------------------------------------------------------------

// ?? min velosity???
std::vector<double>  generateFrames (double a, double b, size_t n)
{
  std::vector<double> frames (n--);

  double  sum = 0.;
  for ( size_t i = 0U; i <= n; ++i )
  { 
    // frames[i] = (a + (b - a) * (atan (tan ((i / n - 0.5) / M_PI * 10.) * 7.) / M_PI + 0.5));
    // frames[i] = (a + b + (a - b) * cos (i * M_PI / n)) / 2.;

    frames[i] = (1. - cos (i * M_PI / n)) * 0.5;
    sum += frames[i];
  }

  for ( size_t i = 0U; i <= n; ++i )
  {
    frames[i] = frames[i] * (b - a) / sum + a;
  }
  // frames.back () = (b - EPS);

  return frames;
}

// current velosity ??
std::vector<double>  generateStopFrames (double a, double b, size_t n)
{
  std::vector<double> frames (n--);

  double  sum = 0.;
  for ( size_t i = 0U; i <= n; ++i )
  { 
    // frames[i] = (a + b + (a - b)*cos ( ((n / 2. + (i+1) / 2.) * M_PI) / n)) / 2.;
    double d = double (i) / n;
    frames[i] = ( 1. - cos ((d + 1.) * M_PI) ) * 0.5;
    sum += frames[i];
  }

  for ( size_t i = 0U; i <= n; ++i )
  {
    frames[i] = frames[i] * (b - a) / sum + a;
  }
  // frames.back () = (b - EPS);

  return frames;
}

Hand::Hand (const Point &hand, const Point &arm,
            const Point &sholder, const Point &clavicle,
            const std::vector<JointsEnum> &joints,
            const std::vector<MusclesEnum> &muscles):
            minJStopMoveFrames (15U),  // (5U),
            maxClvclMoveFrames (30U), // (15U),
            maxShldrMoveFrames (60U), // (30U),
            maxElbowMoveFrames (55U), // (35U),
            maxWristMoveFrames (15U),
            maxClvclShift (0.40),
            maxShldrAngle (105U),
            maxElbowAngle (135U),
            
            jointsCount (joints.size ()),
            musclesCount (muscles.size ()),
            hand_(hand),
            arm_(arm),
            sholder_ (sholder),
            clavicle_ (clavicle)
{ // joints_ (joints), muscles_ (muscles),
  std::copy ( joints.begin (),  joints.end (), std::begin( joints_));
  std::copy (muscles.begin (), muscles.end (), std::begin(muscles_));
  
  ulong_t frames[MusclesCount] = { (ulong_t)maxClvclMoveFrames, (ulong_t)maxClvclMoveFrames,
                                   (ulong_t)maxShldrMoveFrames, (ulong_t)maxShldrMoveFrames,
                                   (ulong_t)maxElbowMoveFrames, (ulong_t)maxElbowMoveFrames };

  c_frames = generateFrames (EPS, maxClvclShift, maxClvclMoveFrames);
  s_frames = generateFrames (EPS, maxShldrAngle, maxShldrMoveFrames);
  e_frames = generateFrames (EPS, maxElbowAngle, maxElbowMoveFrames);

  // 10% от общего пробега
  c_frstop = generateStopFrames (EPS, maxClvclShift * 0.01, minJStopMoveFrames);
  s_frstop = generateStopFrames (EPS, maxShldrAngle * 0.01, minJStopMoveFrames);
  e_frstop = generateStopFrames (EPS, maxElbowAngle * 0.01, minJStopMoveFrames);

  current_velosity = 0.;

  std::memmove (tFrames2OpenHyd_, frames, sizeof (time_t) * musclesCount);
  
  reset ();
}
void  Hand::step (const bool  control[MusclesCount])
{ time_++;

 for(uint_t i=0; i<musclesCount; ++i)
    muscle (i, control ? control[i] : false);
}
void  Hand::step (MusclesEnum hydNo)
{ time_++;

  for (uint_t i = 0U; i<musclesCount; ++i)
    muscle (i, ((hydNo & (1U<<i)) != 0U) );

}
void  Hand::move (MusclesEnum muscle, time_t last, std::list<Point> &visited)
{
  // std::wcout << std::endl;
  /* START! */
  step (muscle);
  visited.push_back (position);
  // std::wcout << std::wstring (position) << std::endl;

  while ( last-- )
  { /* moving */
    step ();
    visited.push_back (position);
    // std::wcout << std::wstring (position) << std::endl;
  }

  if ( !flagMovEnd_ )
  { /* STOP! */
    step (muscle);
    visited.push_back (position);
    // std::wcout << L"end " << std::wstring(position) << std::endl;
  }

  while ( !flagMovEnd_ )
  { /* coming to a stop */
    step ();
    visited.push_back (position);
    // std::wcout << std::wstring (position) << std::endl;
  }

}
void  Hand::move (MusclesEnum muscle, time_t last) // !simultaniusly
{ step (muscle);
  while ( last-- )
   step ();
  
  if( !flagMovEnd_ )
   step (muscle);
  while ( !flagMovEnd_ )
   step ();
}
void  Hand::draw (HDC hdc, HPEN hPen, HBRUSH hBrush) const
{ 
  //--- draw constants ----------------------------------------------
  const static double  REllipse = 0.03;
  const static double  WSholder = 0.01;
  //-----------------------------------------------------------------
  HPEN   hPen_old   = (HPEN)   SelectObject (hdc, hPen);
  HBRUSH hBrush_old = (HBRUSH) SelectObject (hdc, hBrush);
  //-----------------------------------------------------------------
  Point c (clavicle_),  s (curPosShldr_),
        a (curPosArm_), h (curPosHand_);

  Point su (sholder_.x + WSholder - shiftClvcl_, sholder_.y + WSholder),
        sd (sholder_.x - WSholder - shiftClvcl_, sholder_.y - WSholder),
        au (    arm_.x + WSholder - shiftClvcl_,     arm_.y + WSholder),
        ad (    arm_.x - WSholder - shiftClvcl_,     arm_.y - WSholder),
        hu (   hand_.x + WSholder - shiftClvcl_,    hand_.y + WSholder),
        hd (   hand_.x - WSholder - shiftClvcl_,    hand_.y - WSholder);

  //-----------------------------------------------------------------
  su.rotate (s, angleShldr_);
  sd.rotate (s, angleShldr_);

  au.rotate (s, angleShldr_);
  ad.rotate (s, angleShldr_);

  hu.rotate (s, angleShldr_);
  hd.rotate (s, angleShldr_);
  hu.rotate (a, angleElbow_);
  hd.rotate (a, angleElbow_);

  //-----------------------------------------------------------------
  MoveToEx (hdc, Tx (1.00), Ty (su.y), NULL);
  LineTo   (hdc, Tx (su.x), Ty (su.y));
  LineTo   (hdc, Tx (au.x), Ty (au.y));
  
  MoveToEx (hdc, Tx (1.00), Ty (sd.y), NULL);
  LineTo   (hdc, Tx (sd.x), Ty (sd.y));
  LineTo   (hdc, Tx (ad.x), Ty (ad.y));

  au.rotate (a, angleElbow_);
  ad.rotate (a, angleElbow_);

  MoveToEx (hdc, Tx (au.x), Ty (au.y), NULL);
  LineTo   (hdc, Tx (hu.x), Ty (hu.y));
  
  MoveToEx (hdc, Tx (ad.x), Ty (ad.y), NULL);
  LineTo   (hdc, Tx (hd.x), Ty (hd.y));
  
  //---clavicle-------------------------------------------------------
  Ellipse (hdc, Tx (-REllipse + c.x), Ty ( REllipse + c.y),
                Tx ( REllipse + c.x), Ty (-REllipse + c.y));
  
  //---sholder--------------------------------------------------------
  Ellipse (hdc, Tx (-REllipse + s.x), Ty ( REllipse + s.y),
                Tx ( REllipse + s.x), Ty (-REllipse + s.y));
  
  //---arm--------------------- --------------------------------------
  Ellipse (hdc, Tx (-REllipse + a.x), Ty ( REllipse + a.y),
                Tx ( REllipse + a.x), Ty (-REllipse + a.y));
  
  //---hand-------------------- --------------------------------------
  Ellipse (hdc, Tx (-REllipse + h.x), Ty ( REllipse + h.y),
                Tx  (REllipse + h.x), Ty (-REllipse + h.y));
  //------------------------------------------------------------------
  // отменяем ручку
  SelectObject (hdc, hPen_old);
  SelectObject (hdc, hBrush_old);
}
             
void  Hand::reset ()
{ time_ = 0ULL;

 flagMovEnd_ = false;
 //-----------------------------------------------------------------
 //     hand_.x = -0.3;     hand_.y = 0.9;
 //      arm_.x =  0.3;      arm_.y = 0.6;
 //  sholder_.x =  0.8;  sholder_.y = 0.1;
  // clavicle_.x =  0.8; clavicle_.y = 0.1;
 //-----------------------------------------------------------------
  std::memset (timeBgn2OpenHyd_, 0, sizeof (*timeBgn2OpenHyd_) * musclesCount);
  std::memset (timeEnd2OpenHyd_, 0, sizeof (*timeEnd2OpenHyd_) * musclesCount);
 //-----------------------------------------------------------------
  curPosShldr_ = sholder_;
 curPosArm_   =     arm_;
 curPosHand_  =    hand_;
 //-----------------------------------------------------------------
 shiftClvcl_ = angleElbow_ = angleShldr_ = 0.0;
}
//void  Hand::set   (const uchar_t jOp[jointsCount])
//{ reset ();
// //----------------------------------------------
//  shiftClvcl_ = (jOp[0] > 100) ? 100.0 : jOp[0];
//  angleShldr_ = (jOp[1] > 100) ? 100.0 : jOp[1];
//  angleElbow_ = (jOp[2] > 100) ? 100.0 : jOp[2];
//  //----------------------------------------------
//  angleElbow_ = angleElbow_ / 100.0 * maxElbowAngle;
// angleShldr_ = angleShldr_ / 100.0 * maxShldrAngle;
//  shiftClvcl_ = shiftClvcl_ / 100.0 * maxClvclShift;
//
//  curPosShldr_.x -= shiftClvcl_;
//  curPosArm_  .x -= shiftClvcl_;
//  curPosHand_ .x -= shiftClvcl_; 
//
// curPosArm_ .rotate (curPosShldr_, angleShldr_);
// curPosHand_.rotate (curPosShldr_, angleShldr_);
// curPosHand_.rotate (curPosArm_,   angleElbow_);
//  //----------------------------------------------
//}
/* jOp = { Clvcl, Shldr, Elbow } < 100.0 % */
void  Hand::set (JointsEnum joint, const std::array<double,Hand::JointsCount> &jOp)
{
  if ( joint )
  {
    int index = 0;
    double shiftClvcl, angleShldr, angleElbow;
    //----------------------------------------------
    if ( joint & Clvcl )
    {
      shiftClvcl = (jOp[index] > 100.0) ? 100.0 : jOp[index];
      shiftClvcl = shiftClvcl / 100.0 * maxClvclShift;
      ++index;
    }
    else shiftClvcl = shiftClvcl_;

    if ( joint & Shldr )
    {
      angleShldr = (jOp[index] > 100.0) ? 100.0 : jOp[index];
      angleShldr = angleShldr / 100.0 * maxShldrAngle;
      ++index;
    }
    else angleShldr = angleShldr_;

    if ( joint & Elbow )
    {
      angleElbow = (jOp[index] > 100.0) ? 100.0 : jOp[index];
      angleElbow = angleElbow / 100.0 * maxElbowAngle;
      ++index;
    }
    else angleElbow = angleElbow_;

    reset ();
    //----------------------------------------------
    curPosShldr_.x -= shiftClvcl;
      curPosArm_.x -= shiftClvcl;
     curPosHand_.x -= shiftClvcl;

    curPosArm_ .rotate (curPosShldr_, angleShldr);
    curPosHand_.rotate (curPosShldr_, angleShldr);
    curPosHand_.rotate (curPosArm_,   angleElbow);
    //----------------------------------------------
    shiftClvcl_ = shiftClvcl;
    angleShldr_ = angleShldr;
    angleElbow_ = angleElbow;
  }
}
void  Hand::set (MusclesEnum muscle, uint_t frame)
{
  double res = 0.;
  for ( auto i : boost::irange (0U, frame) )
    res += stepFrame (muscle, (ulong_t)i, false);
}
//--------------------------------------------------------------------------------
double  Hand::stepFrame (MusclesEnum muscle, time_t time, bool atStop) const
{ double  frame;

  int i = (int) time;
  switch ( muscle )
  {
    default:                      frame = 0.;                                   break;
    case ClvclOpn: case ClvclCls: frame = (atStop) ? c_frstop[i] : c_frames[i]; break;
    case ShldrOpn: case ShldrCls: frame = (atStop) ? s_frstop[i] : s_frames[i]; break;
    case ElbowOpn: case ElbowCls: frame = (atStop) ? e_frstop[i] : e_frames[i]; break;
  }
  
  // double cum_sum = 0.;
  // for ( auto i = 0U; i <= time; ++i )
  //   cum_sum += (*frames)[time];
  // std::wcout << time << ' ' << (*frames)[time] << ' ' << cum_sum << std::endl;

  // if ( atStop )
  // {
  //   double velosity = (time) ? ((*frames)[time] - (*frames)[time - 1U]) : ((*frames)[time]);
  //   while ( current_velosity < velosity )
  //   { 
  //     if ( time >= frames->size () )
  //     { current_velosity = frame = 0.; }
  //     ++time;
  //     velosity = (time) ? ((*frames)[time] - (*frames)[time - 1U]) : ((*frames)[time]);
  //   }
  //   frame = (*frames)[time]; // velosity;
  // }
  // current_velosity = (time) ? ((*frames)[time] - (*frames)[time - 1U]) : ((*frames)[time]);
 return frame;
}
//--------------------------------------------------------------------------------
size_t  Hand::muscleIndex (Hand::MusclesEnum  muscle)
{
  if ( !muscle ) return musclesCount;
  for ( auto m : muscles_ )
  {
    if ( m & muscle )
      switch ( muscle )
      {
        case Hand::ClvclOpn: return 0U;
        case Hand::ClvclCls: return 1U;
        case Hand::ShldrOpn: return 2U;
        case Hand::ShldrCls: return 3U;
        case Hand::ElbowOpn: return 4U;
        case Hand::ElbowCls: return 5U;
        case Hand::WristOpn: return 6U;
        case Hand::WristCls: return 7U;
        default: return musclesCount;
      }
  }
  return musclesCount;
}

namespace OldHand
{
  //--------------------------------------------------------------------------------
  bool  muscleValidAtOnce (Hand::MusclesEnum muscle)
  {
    if ( !muscle
        || ((Hand::ClvclOpn & muscle) && (Hand::ClvclCls & muscle))
        || ((Hand::ShldrOpn & muscle) && (Hand::ShldrCls & muscle))
        || ((Hand::ElbowOpn & muscle) && (Hand::ElbowCls & muscle))
        || ((Hand::WristOpn & muscle) && (Hand::WristCls & muscle))
        )
    {
      return false;
    }
    return true;
  }
  //--------------------------------------------------------------------------------
  //size_t  nextMuscleIndex (Hand::MusclesEnum  muscle, size_t li=0U)
  //{
  //  size_t index = li;
  //  if ( !muscle ) return Hand::musclesCount;
  //  for ( auto m : Hand::muscles | boost::adaptors::sliced (li, Hand::musclesCount) )
  //  {
  //    if ( m & muscle )
  //      return index;
  //    ++index;
  //  }
  //  return  Hand::musclesCount;
  //}
  //--------------------------------------------------------------------------------
  Hand::MusclesEnum  muscleByJoint (Hand::JointsEnum joint, bool open)
  {
    if ( !joint ) return Hand::EmptyMov;

    for ( auto j : joints )
    {
      if ( j & joint )
        switch ( j )
        {
          case Hand::Clvcl: return (open) ? Hand::ClvclOpn : Hand::ClvclCls;
          case Hand::Shldr: return (open) ? Hand::ShldrOpn : Hand::ShldrCls;
          case Hand::Elbow: return (open) ? Hand::ElbowOpn : Hand::ElbowCls;
          case Hand::Wrist: return (open) ? Hand::WristOpn : Hand::WristCls;
          default:          return Hand::EmptyMov;
        }
    }
    return Hand::EmptyMov;
  }
  Hand::JointsEnum   jointByMuscle (Hand::MusclesEnum muscle)
  {
    if ( !muscle ) return Hand::Empty;

    for ( auto m : muscles )
    {
      if ( m & muscle )
        switch ( m )
        {
          default:             return Hand::Empty;
          case Hand::ClvclOpn:
          case Hand::ClvclCls: return Hand::Clvcl;
          case Hand::ShldrOpn:
          case Hand::ShldrCls: return Hand::Shldr;
          case Hand::ElbowOpn:
          case Hand::ElbowCls: return Hand::Elbow;
          case Hand::WristOpn:
          case Hand::WristCls: return Hand::Wrist;
        }
    }
    return Hand::Empty;
  }
  //--------------------------------------------------------------------------------
  Hand::MusclesEnum  selectHandMove (uint_t choose)
  {
    Hand::MusclesEnum  muscles;
    switch ( choose )
    {
      default: muscles = Hand::EmptyMov;                                   break;
      case  0: muscles = Hand::ClvclOpn;                                   break;
      case  1: muscles = Hand::ShldrOpn;                                   break;
      case  2: muscles = Hand::ElbowOpn;                                   break;
      case  3: muscles = Hand::ClvclCls;                                   break;
      case  4: muscles = Hand::ShldrCls;                                   break;
      case  5: muscles = Hand::ElbowCls;                                   break;
      case  6: muscles = Hand::ClvclOpn | Hand::ShldrOpn;                  break;
      case  7: muscles = Hand::ClvclOpn | Hand::ElbowOpn;                  break;
      case  8: muscles = Hand::ShldrOpn | Hand::ElbowOpn;                  break;
      case  9: muscles = Hand::ClvclOpn | Hand::ShldrOpn | Hand::ElbowOpn; break;
      case 10: muscles = Hand::ClvclOpn | Hand::ShldrOpn | Hand::ElbowCls; break;
      case 11: muscles = Hand::ClvclCls | Hand::ShldrOpn;                  break;
      case 12: muscles = Hand::ClvclCls | Hand::ElbowOpn;                  break;
      case 13: muscles = Hand::ShldrCls | Hand::ElbowOpn;                  break;
      case 14: muscles = Hand::ClvclCls | Hand::ShldrOpn | Hand::ElbowOpn; break;
      case 15: muscles = Hand::ClvclCls | Hand::ShldrOpn | Hand::ElbowCls; break;
      case 16: muscles = Hand::ClvclOpn | Hand::ShldrCls;                  break;
      case 17: muscles = Hand::ClvclOpn | Hand::ElbowCls;                  break;
      case 18: muscles = Hand::ShldrOpn | Hand::ElbowCls;                  break;
      case 19: muscles = Hand::ClvclOpn | Hand::ShldrCls | Hand::ElbowOpn; break;
      case 20: muscles = Hand::ClvclOpn | Hand::ShldrCls | Hand::ElbowCls; break;
      case 21: muscles = Hand::ClvclCls | Hand::ShldrCls;                  break;
      case 22: muscles = Hand::ClvclCls | Hand::ElbowCls;                  break;
      case 23: muscles = Hand::ShldrCls | Hand::ElbowCls;                  break;
      case 24: muscles = Hand::ClvclCls | Hand::ShldrCls | Hand::ElbowOpn; break;
      case 25: muscles = Hand::ClvclCls | Hand::ShldrCls | Hand::ElbowCls; break;
    }
    return muscles;
  }
  //--------------------------------------------------------------------------------
};