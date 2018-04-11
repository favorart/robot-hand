#pragma once
#include <iostream>

namespace Robo
{
//-------------------------------------------------------------------------------
using frames_t = size_t;   // descrete time
using muscle_t = uint8_t;  // muscle index (no holes)
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
    friend tostream& operator<<(tostream &out, const Actuator &a);

    template<class Archive>
    void  serialize(Archive &ar, const unsigned int version)
    { ar & muscle & start & last; }
};
//-------------------------------------------------------------------------------
class  Control
{
    static const unsigned MAX_ACTUATORS = 8;         // number of brakes
    std::array<Actuator, MAX_ACTUATORS> actuators{}; // sorted by start
    size_t actuals = 0;

public:
    Control() = default;
    Control(const Actuator& a) : Control()
    { actuators[actuals++] = a; }
    Control(const Actuator* a, size_t sz);

    Control(const Control&) = default;
    Control(Control&&) = default;
    Control& operator=(const Control &c) = default;

    //----------------------------------------------------
    using iterator = std::array<Actuator, MAX_ACTUATORS>::iterator;
    using const_iterator = std::array<Actuator, MAX_ACTUATORS>::const_iterator;

    auto begin()       -> decltype(boost::begin(actuators)) { return boost::begin(actuators); }
    auto begin() const -> decltype(boost::begin(actuators)) { return boost::begin(actuators); }
    auto   end()       -> decltype(boost::end(actuators)) { return boost::begin(actuators) + actuals; } // std::advance()
    auto   end() const -> decltype(boost::end(actuators)) { return boost::begin(actuators) + actuals; } // std::advance()

    //----------------------------------------------------
    size_t size() const { return actuals; }
    void append(const Actuator& a); // sorted
    void pop_back();
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
    template<class Archive>
    void  serialize(Archive &ar, const unsigned int version)
    { for (auto& a : actuators) a.serialize(ar, version); }
    //----------------------------------------------------
    friend tostream& operator<<(tostream &out, const Control &controls);
};

//-------------------------------------------------------------------------------
inline Control operator+(const Control &cl, const Control &cr)
{ 
    Control c = cl;
    for (auto &a : cr)
        if (a.muscle != Robo::MInvalid)
            c.append(a);
    return c;
}
inline Control operator+(const Control &cl, const Actuator &a)
{
    Control c = cl;
    if (a.muscle != Robo::MInvalid)
        c.append(a);
    return c;
}
inline Control operator+(const Control &cl, const std::vector<Actuator> &v)
{
    Control c = cl;
    for (auto &a : v)
        if (a.muscle != Robo::MInvalid)
            c.append(a);
    return c;
}

//-------------------------------------------------------------------------------
inline Control EmptyMov() { return {}; }
//-------------------------------------------------------------------------------
}
