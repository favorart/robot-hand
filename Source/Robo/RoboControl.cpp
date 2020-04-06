#include "Robo.h"
#include "RoboControl.h"

using namespace std;
using namespace Robo;

constexpr double Robo::RoboI::minFrameMove = 1e-10;// Utils::EPSILONT;
//---------------------------------------------------------
bool Robo::Actuator::intersect(const Robo::Actuator &a) const
{
    return (a.start <= start && start <= (a.start + a.lasts)) ||
           (a.start <= lasts && lasts <= (a.start + a.lasts)) ||
           (start <= a.start && a.start <= (start + lasts)) ||
           (start <= a.lasts && a.lasts <= (start + lasts));
}

//---------------------------------------------------------
bool Robo::Actuator::operator<(const Actuator &a) const
{
    return (muscle < a.muscle) && (start < a.start); //(muscle != MInvalid)
}

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
        if (actuators[i] >= a)
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
    if (i >= actuals)
        CERROR("remove: Invalid index: " << i << " actuals=" << actuals);
    actuals--;
    for (size_t j = i; j < actuals; ++j)
        actuators[j] = actuators[j + 1];
    actuators[actuals] = { MInvalid, 0, 0 };
    _validated = false;
}

//---------------------------------------------------------
void Robo::Control::remove(const Robo::Actuator *a)
{
    size_t pos = (a - actuators.data());
    if (a >= actuators.data() && pos < actuals)
        std::copy(begin() + pos + 1, end(), begin() + pos);
        //std::memmove((void*)(a), (void*)(a + 1), (actuals - pos) * sizeof(Actuator));
    throw std::logic_error("remove: Actuator does not belong to Control!");
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
    {
        for (size_t i = 0; i < actuals; ++i)
            actuators[i].start -= start_time;
        _validated = false;
    }
}

//----------------------------------------------------
bool Robo::Control::intersectMusclesOpposites() const
{
    if (actuals <= 1)
        return true;
    for (auto iti = this->begin(); iti != this->end(); ++iti)
        for (auto itj = std::next(iti); itj != this->end(); ++itj)
            if (itj->start > (iti->start + iti->lasts))
                break; // т.к. отсортированы по возрастанию start
            // есть противоположные мышцы и есть перекрытие по времени
            else if (RoboI::muscleOpposite(iti->muscle) == itj->muscle && iti->intersect(*itj))
                return false;
    return true;
}

//---------------------------------------------------------
void Robo::Control::fillRandom(Robo::muscle_t n_muscles, const MaxMuscleCount &muscleMaxLasts,
                               Robo::frames_t lasts_min, unsigned min_n_moves, unsigned max_n_moves, bool simul)
{
    if (min_n_moves <= 0 || max_n_moves <= 0 || max_n_moves < min_n_moves)
        throw std::logic_error("fillRandom: Incorrect Min Max n_moves");

    unsigned moves_count = Utils::random(min_n_moves, max_n_moves);
    if (moves_count >= MAX_ACTUATORS)
        throw std::logic_error("fillRandom: Too much actuators for Control!");

    clear();
    Robo::Actuator a{};
    for (unsigned mv = 0; mv < moves_count; ++mv)
    {
        a.muscle = Utils::random(n_muscles);
        while (!a.lasts)
            a.lasts = Utils::random(lasts_min, muscleMaxLasts(a.muscle));
        actuators[actuals++] = a; // append to end NOT SORTED

        if (!simul)
            a.start += (a.lasts + 1);
        else
            a.start = Utils::random<frames_t>(0, a.lasts + 1);
    }
    br::sort(*this); // then SORT
    // remove duplicates and combine intersects
    for (auto i = 0u; i < (actuals - 1); ++i)
        for (auto j = (i + 1); j < actuals;)
        {
            auto &ai = actuators[i], &aj = actuators[j];
            frames_t ai_fin = ai.start + ai.lasts;
            if (ai.muscle == aj.muscle && ai.intersect(aj))
            {
                ai.lasts = std::max(ai_fin, aj.start + aj.lasts);
                ai.start = std::min(ai.start, aj.start);
                ai.lasts -= ai.start;

                std::memmove(&actuators[j], &actuators[j + 1u], actuals - j);
                --actuals;
                actuators[actuals] = { MInvalid, 0, 0 };
            }
            else if (ai_fin < aj.start)
                break;
            else
                ++j;
        }
    _validated = true;
}

//---------------------------------------------------------
std::vector<Robo::Actuator> Robo::Control::align() const
{
    return {};
}

//---------------------------------------------------------
auto Robo::Control::begin()       -> decltype(boost::begin(actuators)) { return boost::begin(actuators); }
auto Robo::Control::begin() const -> decltype(boost::begin(actuators)) { return boost::begin(actuators); }
auto Robo::Control::end()         -> decltype(boost::end(actuators))   { return boost::begin(actuators) + actuals; }
auto Robo::Control::end()   const -> decltype(boost::end(actuators))   { return boost::begin(actuators) + actuals; }

auto Robo::Control::rbegin()       -> decltype(boost::rbegin(actuators)) { return boost::rbegin(actuators); }
auto Robo::Control::rbegin() const -> decltype(boost::rbegin(actuators)) { return boost::rbegin(actuators); }
auto Robo::Control::rend()         -> decltype(boost::rend(actuators))   { auto it = boost::rbegin(actuators); boost::advance(it, actuals); return it; }
auto Robo::Control::rend()   const -> decltype(boost::rend(actuators))   { auto it = boost::rbegin(actuators); boost::advance(it, actuals); return it; }

//---------------------------------------------------------
void Robo::Control::validated(Robo::muscle_t n_muscles) const
{
    if (_validated) return;
    /* Что-то должно двигаться, иначе беск.цикл */
    if (!actuals)
        throw std::logic_error("validated: Controls are empty!\r\n" + this->str());
    /* Управление должно быть отсортировано по времени запуска двигателя */
    frames_t start = 0;
    auto not_sorted = [&start](const Actuator &a) { 
        bool res = (start > a.start);
        start = a.start;
        return res;
    };
    if (/*actuators[0].start != 0 ||*/ ba::any_of(*this, not_sorted))
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
    frames_t start = 0; // actuators[0].start == 0
    auto is_invalid = [n_muscles, &start](const Actuator &a) {
        bool res = (start > a.start);
        start = a.start;
        return res || (!a.lasts) || (a.muscle >= n_muscles);
    };
    return _validated = (actuals > 0 && ba::none_of(*this, is_invalid));
}

//---------------------------------------------------------
void Robo::Control::order(Robo::muscle_t n_muscles, bool keep_actors)
{
    if (!validate(n_muscles))
    {
        br::for_each(actuators, [keep_actors](auto &a) { if (!a.lasts) a.lasts = 1;
        //if (!keep_actors) remove(a - actuators.data());
        });
        br::sort(actuators);
    }
    removeStartPause();
}

//---------------------------------------------------------
void Robo::Control::shorter(size_t index, frames_t velosity, bool infl_oppo_start, bool infl_oppo_lasts)
{
    muscle_t m = actuators[index].muscle;
    bool removed = false;

    if (actuators[index].lasts > velosity)
        actuators[index].lasts -= velosity;
    else
    {
        velosity -= actuators[index].lasts;
        remove(index);
        removed = true;
    }

    if (velosity && ((!removed && infl_oppo_start) || infl_oppo_lasts))
    {
        auto m_op = RoboI::muscleOpposite(m);
        auto it_op = begin();
        while ((it_op = std::find(++it_op, end(), m_op)) != end())
        {
            if (infl_oppo_start && !removed && it_op->start > actuators[index].start)
            {
                it_op->start -= std::min(it_op->start, velosity);
                for (size_t pos = (it_op - begin()); pos > 0 && actuators[pos-1].start > actuators[pos].start; pos--)
                    std::swap(actuators[pos], actuators[pos-1]);
            }
            if (infl_oppo_lasts)
                longer(it_op - begin(), velosity);

            if (!validate(RoboI::musclesMaxCount))
                tcout << std::endl;
        }
    }
    //if (removed) --index;
    //_validated = false;
}

//---------------------------------------------------------
void Robo::Control::longer(size_t index, frames_t velosity, bool oppo_start)
{
    actuators[index].lasts += velosity;
    if (oppo_start)
    {
        auto m_op = RoboI::muscleOpposite(actuators[index].muscle);
        for (auto it_op = std::find(begin() + index, end(), m_op); it_op != end(); ++it_op)
            if (it_op->start > actuators[index].start)
            {
                it_op->start += velosity;
                for (auto it = it_op + 1; it != end() && it_op->start > it->start; ++it_op, ++it)
                    std::swap(*it, *it_op);

                if (!validate(RoboI::musclesMaxCount))
                    tcout << std::endl;
            }
    }
    //_validated = false;
}

//-------------------------------------------------------------------------------
Robo::Control Robo::operator+(const Robo::Control &cl, const Robo::Control &cr)
{
    Robo::Control c{ cl };
    for (auto &a : cr)
        if (a.muscle != Robo::MInvalid && a.lasts != 0)
            c.append(a);
        else CWARN("muscle==MInvalid or last==0");
    return c;
}
Robo::Control Robo::operator+(const Robo::Control &cl, const Robo::Actuator &a)
{
    Robo::Control c{ cl };
    if (a.muscle != Robo::MInvalid && a.lasts != 0)
        c.append(a);
    else CWARN("muscle==MInvalid or last==0");
    return c;
}
Robo::Control Robo::operator+(const Robo::Control &cl, const std::vector<Robo::Actuator> &v)
{
    Robo::Control c{ cl };
    for (auto &a : v)
        if (a.muscle != Robo::MInvalid && a.lasts != 0)
            c.append(a);
    //else CWARN("muscle==MInvalid or last==0");
    return c;
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
    a.muscle = muscle_t(m);
    return s;
}

//---------------------------------------------------------
tostream& Robo::operator<<(tostream &s, const Robo::Control &controls)
{
    s << _T("c[");
    for (const Robo::Actuator &a : controls)
        s << a << ((size_t(&a - &controls.actuators[0]) == (controls.actuals - 1)) ? _T("") : _T(","));
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
        s << ((size_t(&a - &actuators[0]) == (actuals - 1)) ? "" : ",");
    }
    s << "]";
    return s;
}
