#include "StdAfx.h"
#include "Robo.h"
#include "RoboControl.h"


const double Robo::RoboI::minFrameMove = (EPS * M_PI / 180.);
//---------------------------------------------------------
Robo::Control::Control(const Robo::Actuator* a, size_t sz) : Control()
{
    if (sz >= MAX_ACTUATORS)
        throw std::runtime_error("too much muscles for control");

    actuals = sz;
    for (size_t i = 0; i < sz; ++i)
        actuators[i] = a[i];
}

//---------------------------------------------------------
void Robo::Control::append(const Robo::Actuator& a)
{
    if (actuals >= MAX_ACTUATORS - 1)
        throw std::runtime_error("too much muscles for control");

    if (actuals == 0 || actuators[actuals - 1].start <= a.start)
    {
        actuators[actuals++] = a;
        return;
    }

    /* sorted insert */
    for (size_t i = 0; i < actuals; ++i)
        if (actuators[i].start > a.start)
        {
            memmove(&actuators[i + 1], &actuators[i], actuals - i);
            actuators[i] = a;
            ++actuals;
            return;
        }
}

void Robo::Control::pop_back()
{

}

//---------------------------------------------------------
void Robo::Control::removeStartPause()
{
    Robo::frames_t start_time = actuators[0].start;
    if (start_time > 0)
        for (size_t i = 0; i < actuals; ++i)
            actuators[i].start -= start_time;
}

//----------------------------------------------------
bool Robo::Control::validateMusclesTimes() const
{
    if (actuals <= 1)
        return true;
    
    //for (auto iti = this->begin(); iti != this->end(); ++iti)
    //    for (auto itj = std::next(iti); itj != this->end(); ++itj)
    //        if (itj->start >= (iti->start + iti->last))
    //            break; // т.к. отсортированы по возрастанию start
    //        else if (!musclesValidUnion(iti->muscle, itj->muscle))
    //        {
    //            // Если есть перекрытие по времени и есть
    //            // работающие одновременно противоположные мышцы
    //            return false;
    //        }
    return true;
}

//---------------------------------------------------------
void Robo::Control::fillRandom(IN Robo::muscle_t muscles_count, IN const std::function<Robo::frames_t(Robo::muscle_t)> &muscleMaxLasts,
                               IN Robo::frames_t lasts_min, IN unsigned moves_count_min, IN unsigned moves_count_max)
{
    clear();

    unsigned moves_count = random(moves_count_min, moves_count_max);

    Robo::Actuator a{ MInvalid /* muscle - empty move */, 0 /* start */ , 0 /* last */ };
    for (unsigned mv = 0; mv < moves_count; ++mv)
    {
        a.muscle = random(muscles_count);
        if (!a.last)
        { a.last = random(lasts_min, muscleMaxLasts(a.muscle)); }
        else
        { a.last = random(lasts_min, a.last); }
    
        append(a);
        a.start += (a.last + 1);
    }

    /* Tank */
    //for (unsigned mv = 0; mv < moves_count; ++mv)
    //{
    //    a.muscle = random(muscles_count);
    //    /*if*/ while (!a.last)
    //    { a.last = random(lasts_min, muscleMaxLasts(a.muscle)); }
    //    //else
    //    //{ a.last = random(lasts_min, a.last); }
    //
    //    append(a);
    //    //a.start += (a.last + 1);
    //}
}



//void  Hand::recursiveControlsAppend(muscle_t muscles, joint_t joints,
//                                    size_t cur_deep, size_t max_deep)
//{
//    for (auto j : joints_)
//    {
//        if (!(j & joints) && (j > joints))
//        {
//            if (cur_deep == max_deep)
//            {
//                controls.push_back(muscles | muscleByJoint(j, true));
//                controls.push_back(muscles | muscleByJoint(j, false));
//            }
//            else
//            {
//                recursiveControlsAppend(muscles | muscleByJoint(j, true), j | joints, cur_deep + 1U, max_deep);
//                recursiveControlsAppend(muscles | muscleByJoint(j, false), j | joints, cur_deep + 1U, max_deep);
//            } // end else
//        } // end if
//    } // end for
//};
//void  Hand::createControls()
//{
//    for (auto m : muscles_)
//    { controls.push_back(m); }
//
//    /* 1 << 0,1,2,3,4,5,6,7 */
//    /* Impossible (0 & 1) (2 & 3) (4 & 5) (6 & 7) */
//
//    size_t    maxSimultMoves = joints_.size();
//    for (size_t SimultMoves = 1U; SimultMoves < maxSimultMoves; ++SimultMoves)
//        recursiveControlsAppend(Hand::EmptyMov, Hand::Empty, 0U, SimultMoves);
//}

//---------------------------------------------------------
tostream& Robo::operator<<(tostream &out, const Robo::Actuator &a)
{
    out << _T("{ ") /* << a.muscle */ 
        << _T(" ") << a.start
        << _T(" ") << a.last
        << _T(" }");
    return out;
}
tostream& Robo::operator<<(tostream &out, const Robo::Control &controls)
{
    out << _T("ctrl[ ");
    for (const Robo::Actuator &a : controls)
        out << a << _T(", ");
    out << _T(" ]");
    return out;
}

//---------------------------------------------------------
