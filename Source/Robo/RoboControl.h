#pragma once
#include <iostream>

namespace Robo
{
//-------------------------------------------------------------------------------
using frames_t = size_t;   ///< descrete time
using muscle_t = uint8_t;  ///< muscle index (no holes)
//-------------------------------------------------------------------------------
class RoboI;
const muscle_t MInvalid = 0xFF;
const frames_t LastInfinity = -1;
//-------------------------------------------------------------------------------
struct Actuator
{
    muscle_t  muscle = MInvalid;
    frames_t  start = 0;
    frames_t  last = 0;
    //----------------------------------------------------
    Actuator() = default;
    Actuator(muscle_t muscle, frames_t start, frames_t last) :
        muscle(muscle), start(start), last(last) {}
    Actuator(Actuator&&) = default;
    Actuator(const Actuator&) = default;
    Actuator& operator=(const Actuator&) = default;
    //----------------------------------------------------
    bool operator<  (const Actuator &a) const
    { return (start < a.start); }
    bool operator== (const Actuator &a) const
    { return (muscle == a.muscle && start == a.start && last == a.last); }
    bool operator!= (const Actuator &a) const
    { return !(*this == a); }
    bool operator== (const muscle_t  m) const
    { return (this->muscle == m); }
    bool operator!= (const muscle_t  m) const
    { return (this->muscle != m); }
    //----------------------------------------------------
    friend tostream& operator<<(tostream&, const Actuator&);
    friend tistream& operator>>(tistream&, Actuator&);
    //----------------------------------------------------
    template<class Archive>
    void serialize(Archive &ar, unsigned version)
    { ar & muscle & start & last; }
};
//-------------------------------------------------------------------------------
class  Control
{
    static const unsigned MAX_ACTUATORS = 8;         ///< number of brakes
    std::array<Actuator, MAX_ACTUATORS> actuators{}; ///< sorted by start
    size_t actuals = 0;

    // ----------------------------------------
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, unsigned version)
    {
        ar & actuals;
        for (auto &a : actuators)
            ar & a;
    }

public:
    Control() = default;
    Control(const Actuator &a) : Control()
    { actuators[actuals++] = a; }
    Control(const Actuator *a, size_t sz);

    Control(Control&&) = default;
    Control(const Control&) = default;
    Control& operator=(const Control &c) = default;

    //----------------------------------------------------
    using iterator = std::array<Actuator, MAX_ACTUATORS>::iterator;
    using const_iterator = std::array<Actuator, MAX_ACTUATORS>::const_iterator;

    auto begin()       -> decltype(boost::begin(actuators)) { return boost::begin(actuators); }
    auto begin() const -> decltype(boost::begin(actuators)) { return boost::begin(actuators); }
    auto   end()       -> decltype(boost::end(actuators)) { return boost::begin(actuators) + actuals; } // ?? std::advance()
    auto   end() const -> decltype(boost::end(actuators)) { return boost::begin(actuators) + actuals; } // ?? std::advance()

    //----------------------------------------------------
    size_t size() const { return (actuals); }
    void append(const Actuator& a); // sorted
    void push_back(const Actuator& a) { append(a); }

    void clear()
    { actuals = 0; actuators.fill({}); }

    //----------------------------------------------------
    Actuator& operator[](size_t i)
    { return actuators[(i < actuals) ? i : (actuals - 1)]; };
    const Actuator& operator[](size_t i) const
    { return actuators[(i < actuals) ? i : (actuals - 1)]; };

    //----------------------------------------------------
    bool  operator== (const Control &c) const
    {
        if (this != &c)
            for (size_t i = 0; i < actuals; ++i)
                if (actuators[i] != c.actuators[i])
                    return false;
        return true;
    }
    bool  operator!= (const Control &c) const
    { return !(*this == c); }
    bool  operator== (const muscle_t m) const
    { return  (actuals == 1 && actuators[0].muscle == m); }
    bool  operator!= (const muscle_t m) const
    { return  !(*this == m); }

    //----------------------------------------------------
    void removeStartPause();
    bool validateMusclesTimes() const;

    //----------------------------------------------------
    void  Control::fillRandom(IN muscle_t muscles_count,
                              IN const std::function<frames_t(muscle_t)> &muscleMaxLasts,
                              IN frames_t lasts_min = 50,
                              IN unsigned moves_count_min = 1,
                              IN unsigned moves_count_max = 3);
    //----------------------------------------------------
    friend tostream& operator<<(tostream&, const Control&);
    friend tistream& operator>>(tistream&, Control&);
};
//-------------------------------------------------------------------------------
inline Control EmptyMov() { return {}; }
//-------------------------------------------------------------------------------
inline Control operator+(const Control &cl, const Control &cr)
{ 
    Control c = cl;
    for (auto &a : cr)
        if (a.muscle != MInvalid && a.last != 0)
            c.append(a);
        else CWARN("muscle==MInvalid or last==0");
    return c;
}
inline Control operator+(const Control &cl, const Actuator &a)
{
    Control c = cl;
    if (a.muscle != MInvalid && a.last != 0)
        c.append(a);
    else CWARN("muscle==MInvalid or last==0");
    return c;
}
inline Control operator+(const Control &cl, const std::vector<Actuator> &v)
{
    Control c = cl;
    for (auto &a : v)
        if (a.muscle != MInvalid && a.last != 0)
            c.append(a);
        //else CWARN("muscle==MInvalid or last==0");
    return c;
}
//-------------------------------------------------------------------------------
}
//-------------------------------------------------------------------------------
BOOST_CLASS_VERSION(Robo::Actuator, 2)
BOOST_CLASS_VERSION(Robo::Control, 2)
//-------------------------------------------------------------------------------
