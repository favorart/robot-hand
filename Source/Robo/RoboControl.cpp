#include "Robo.h"
#include "RoboControl.h"


const double Robo::RoboI::minFrameMove = Utils::EPSILONT;
//---------------------------------------------------------
Robo::Control::Control(const Robo::Actuator *a, size_t sz)
{
    if (sz > MAX_ACTUATORS)
        throw std::logic_error("Control: Too much actuators for Control!");

    for (actuals = 0; actuals < sz; ++actuals)
        actuators[actuals] = a[actuals];
    _validated = false;
}

//---------------------------------------------------------
void Robo::Control::append(const Robo::Actuator &a)
{
    auto f = br::find(actuators, a);
    if (f != std::end(actuators))
        throw std::logic_error("append: duplicate");

    if (actuals >= MAX_ACTUATORS)
        throw std::logic_error("append: Too much actuators for Control!");
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
    _validated = false;
}

//---------------------------------------------------------
void Robo::Control::remove(size_t i)
{
    if (!actuals)
        throw std::logic_error("remove: Controls are empty!");

    actuals--;
    for (size_t j = i; j < actuals; ++j)
        actuators[j] = actuators[j + 1];
    actuators[actuals] = { MInvalid, 0, 0 };
    _validated = false;
}

//---------------------------------------------------------
void Robo::Control::pop_back()
{
    if (!actuals)
        throw std::logic_error("pop_back: Controls are empty!");
    actuals--;
    actuators[actuals] = { MInvalid, 0, 0 };
    _validated = false;
}

//---------------------------------------------------------
bool Robo::Control::operator==(const Robo::Control &c) const
{
    if (this == &c)
        return true;
    if (actuals != c.actuals)
        return false;
    for (size_t i = 0; i < actuals; ++i)
        if (actuators[i] != c.actuators[i])
            return false;
    return true;
}

//---------------------------------------------------------
void Robo::Control::removeStartPause()
{
    Robo::frames_t start_time = actuators[0].start;
    if (start_time > 0)
        for (size_t i = 0; i < actuals; ++i)
            actuators[i].start -= start_time;
    _validated = false;
}

//----------------------------------------------------
bool Robo::Control::validateMusclesTimes() const
{
    if (actuals <= 1)
        return true;

    throw std::logic_error("validateMusclesTimes: Not implemented!");

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
    _validated = false;
}

//---------------------------------------------------------
Robo::muscle_t Robo::Control::select(Robo::muscle_t muscle) const
{
    if (!muscle)
    { return actuators[Utils::random(size())].muscle; }
    else
    {
        for (auto &a : actuators)
            if (a.muscle != muscle && (a.muscle / 2) != (muscle / 2))
                return a.muscle;
        //auto m = actuators[random(size())].muscle;
        //while (m == muscle || (m / 2) == (muscle / 2))
        //  m = actuators[random(size())];
    }
    _validated = false;
    return MInvalid;
}

//---------------------------------------------------------
auto Robo::Control::begin()       -> decltype(boost::begin(actuators)) { return boost::begin(actuators); }
auto Robo::Control::begin() const -> decltype(boost::begin(actuators)) { return boost::begin(actuators); }
auto Robo::Control::end()         -> decltype(boost::end(actuators))   { return boost::begin(actuators) + actuals; } // ?? std::advance()
auto Robo::Control::end()   const -> decltype(boost::end(actuators))   { return boost::begin(actuators) + actuals; } // ?? std::advance()

//---------------------------------------------------------
void Robo::Control::validated(Robo::muscle_t n_muscles) const
{
    if (_validated) return;
    /* Что-то должно двигаться, иначе беск.цикл */
    if (!actuals)
        throw std::logic_error("validated: Controls are empty!\r\n" + this->str());
    /* Управление должно быть отсортировано по времени запуска двигателя */
    if (actuators[0].start != 0 || !br::is_sorted(*this))
        throw std::logic_error("validated: Controls are not sorted!\r\n" + this->str());
    /* Исключить незадействованные двигатели */
    if (ba::any_of(*this, [n_muscles](const Actuator &a) { return (!a.lasts) || (a.muscle >= n_muscles); }))
        throw std::logic_error("validated: Controls have UNUSED or INVALID muscles!\r\n" + this->str());
    _validated = true;
}

//---------------------------------------------------------
bool Robo::Control::validate(Robo::muscle_t n_muscles) const
{
    if (_validated) return true;
    auto is_invalid = [n_muscles](const Actuator &a) {
        return (!a.lasts) || (a.muscle >= n_muscles);
    };
   _validated = (!actuals ||
        actuators[0].start != 0 || 
        !br::is_sorted(*this) ||
        ba::any_of(*this, is_invalid));
    return _validated;
}


//---------------------------------------------------------
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
    return s << _T("{") << uint32_t(a.muscle)
             << _T(",") << a.start
             << _T(",") << a.lasts
             << _T("}");
}

//---------------------------------------------------------
tistream& Robo::operator>>(tistream &s, Robo::Actuator &a)
{ 
    uint32_t m;
    s >> ConfirmInput(_T("{")) >> m
      >> ConfirmInput(_T(",")) >> a.start
      >> ConfirmInput(_T(",")) >> a.lasts
      >> ConfirmInput(_T("}"));
    a.muscle = m;
    return s;
}

//---------------------------------------------------------
tostream& Robo::operator<<(tostream &s, const Robo::Control &controls)
{
    s << _T("c[");
    for (const Robo::Actuator &a : controls)
        s << a << (((&a - &controls.actuators[0]) == (controls.actuals - 1)) ? _T("") : _T(","));
    s << _T("]");
    return s;
}
//---------------------------------------------------------
tistream& Robo::operator>>(tistream &s, Robo::Control &controls)
{
    s >> ConfirmInput(_T("c["));
    {
        Robo::Actuator a;
        s >> a >> ConfirmInput(_T(","));
        controls.append(a);
    } while (s.fail());

    s.setstate(std::ios::goodbit);
    s >> ConfirmInput(_T("]"));
    return s;
}
//---------------------------------------------------------
std::ostream& Robo::Control::stream(std::ostream &s) const
{
    s << "c[";
    for (const auto &a : *this)
    {
        a.stream(s);
        s << (((&a - &actuators[0]) == (actuals - 1)) ? "" : ",");
    }
    s << "]";
    return s;
}
//---------------------------------------------------------
