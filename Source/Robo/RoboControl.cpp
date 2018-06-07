#include "Robo.h"
#include "RoboControl.h"


const double Robo::RoboI::minFrameMove = (Utils::EPSILONT * M_PI / 180.);
//---------------------------------------------------------
Robo::Control::Control(const Robo::Actuator *a, size_t sz)
{
    if (sz > MAX_ACTUATORS)
        throw std::logic_error("too much muscles for control");

    for (actuals = 0; actuals < sz; ++actuals)
        actuators[actuals] = a[actuals];
}

//---------------------------------------------------------
void Robo::Control::append(const Robo::Actuator &a)
{
    if (actuals >= MAX_ACTUATORS)
        throw std::logic_error("too much muscles for control");
    // actuals is INDEX TO INSERT and LENGTH
    if (actuals == 0 || actuators[actuals - 1].start <= a.start)
    {
        actuators[actuals++] = a;
        return;
    }
    // sorted insert
    for (size_t i = 0; i < actuals; ++i)
        if (actuators[i].start > a.start)
        {
            memmove(&actuators[i + 1], &actuators[i], (actuals + 1 - i) * sizeof(Actuator));
            actuators[i] = a;
            ++actuals;
            return;
        }
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
void Robo::Control::fillRandom(Robo::muscle_t muscles_count, const std::function<Robo::frames_t(Robo::muscle_t)> &muscleMaxLasts,
                               Robo::frames_t lasts_min, unsigned moves_count_min, unsigned moves_count_max, bool simul)
{
    unsigned moves_count = Utils::random(moves_count_min, moves_count_max);
    clear();

    Robo::Actuator a{ MInvalid /* muscle - empty move */, 0 /* start */ , 0 /* last */ };
    for (unsigned mv = 0; mv < moves_count; ++mv)
    {
        a.muscle = Utils::random(muscles_count);
        if (!a.lasts)
        { a.lasts = Utils::random(lasts_min, muscleMaxLasts(a.muscle)); }
        else
        { a.lasts = Utils::random(lasts_min, a.lasts); }
        ///*if*/ while (!a.last)
        //{ a.last = random(lasts_min, muscleMaxLasts(a.muscle)); } /* Tank */
    
        append(a);
        if (!simul)
            a.start += (a.lasts + 1u);
        else
            a.start = Utils::random(0u, a.lasts + 1u);
    }
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
tostream& Robo::operator<<(tostream &s, const Robo::Actuator &a)
{
    return s << _T("{ ") << uint32_t(a.muscle)
             << _T(" ") << a.start
             << _T(" ") << a.lasts
             << _T(" }");
}

//---------------------------------------------------------
tistream& Robo::operator>>(tistream &s, Robo::Actuator &a)
{ 
    uint32_t m;
    s >> ConfirmInput(_T("{")) >> m
      >> /*ConfirmInput(_T(" ")) >>*/ a.start
      >> /*ConfirmInput(_T(" ")) >>*/ a.lasts
      >> ConfirmInput(_T("}"));
    a.muscle = m;
    return s;
}

//---------------------------------------------------------
tostream& Robo::operator<<(tostream &s, const Robo::Control &controls)
{
    s << _T("ctrl[ ");
    for (const Robo::Actuator &a : controls)
        s << a << (((&a - &controls.actuators[0]) == (controls.actuals - 1)) ? _T(" ]") : _T(", "));
    return s;
}

//---------------------------------------------------------
tistream& Robo::operator>>(tistream &s, Robo::Control &controls)
{
    s >> ConfirmInput(_T("ctrl[ "));
    {
        Robo::Actuator a;
        s >> a >> ConfirmInput(_T(", "));
        controls.append(a);
    } while (s.fail());
    //---------------------------
    s.setstate(std::ios::goodbit);
    s >> ConfirmInput(_T(" ]"));
    return s;
}
//---------------------------------------------------------
