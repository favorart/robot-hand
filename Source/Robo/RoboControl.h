#pragma once

namespace Robo
{
//-------------------------------------------------------------------------------
using frames_t = size_t;   ///< descrete time
using muscle_t = uint8_t;  ///< muscle index (no holes)
//-------------------------------------------------------------------------------
class RoboI;
const muscle_t MInvalid = 0xFF;
const frames_t LastsInfinity = -1;
//-------------------------------------------------------------------------------
struct Actuator
{
    muscle_t  muscle = MInvalid;
    frames_t  start = 0;
    frames_t  lasts = 0;
    //----------------------------------------------------
    Actuator() = default;
    Actuator(muscle_t muscle, frames_t start, frames_t lasts) :
        muscle(muscle), start(start), lasts(lasts) {}
    Actuator(Actuator&&) = default;
    Actuator(const Actuator&) = default;
    Actuator& operator=(const Actuator&) = default;
    //----------------------------------------------------
    bool operator<  (const Actuator &a) const
    { return (start < a.start); }
    bool operator== (const Actuator &a) const
    { return (muscle == a.muscle && start == a.start && lasts == a.lasts); }
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
    { ar & muscle & start & lasts; }
    //----------------------------------------------------
    tstring tstr() const
    { tstringstream ss; ss << *this; return ss.str(); }

protected:
    std::ostream& stream(std::ostream &s) const
    { return s << "{ " << uint32_t{ muscle } << " " << start << " " << lasts << " }"; }
    std::string str() const
    { std::stringstream ss; stream(ss); return ss.str(); }
    //----------------------------------------------------
    friend class Control;
};
//-------------------------------------------------------------------------------
class Control
{
protected:
    static const unsigned MAX_ACTUATORS = 128;       ///< number of brakes
    std::array<Actuator, MAX_ACTUATORS> actuators{}; ///< sorted by start
    size_t actuals = 0;
    // ----------------------------------------
    mutable bool _validated = false;

    // ----------------------------------------
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, unsigned version)
    {
        ar & actuals;
        for (auto &a : actuators)
            ar & a;
    }

    // ----------------------------------------
    std::string str() const
    {
        std::stringstream ss;
        stream(ss);
        return ss.str();
    }
    //----------------------------------------------------
    std::ostream& stream(std::ostream&) const;

public:
    Control() = default;
    Control(const Actuator &a) : Control()
    { actuators[actuals++] = a; }
    Control(const Actuator *a, size_t sz);

    template<size_t N>
    Control(const std::bitset<N> &muscles, frames_t start, frames_t lasts)
    {
        if (!muscles.any()) return;
        if (!lasts || !muscles.any())
            throw std::runtime_error("Invalid control");

        for (muscle_t m = 0; m < N; ++m)
            if (muscles[m])
                append({ m, start, lasts });
        _validated = false;
    }

    Control(Control&&) = default;
    Control(const Control&) = default;
    Control& operator=(const Control &c) = default;

    //----------------------------------------------------
    using iterator = std::array<Actuator, MAX_ACTUATORS>::iterator;
    using const_iterator = std::array<Actuator, MAX_ACTUATORS>::const_iterator;

    auto begin()       -> decltype(boost::begin(actuators));
    auto begin() const -> decltype(boost::begin(actuators));
    auto   end()       -> decltype(boost::end(actuators));
    auto   end() const -> decltype(boost::end(actuators));

    //----------------------------------------------------
    size_t size() const { return (actuals); }
    std::vector<Actuator> v() const { std::vector<Actuator> v(this->begin(), this->end()); return v; }
    void append(const Actuator& a); // sorted
    void push_back(const Actuator& a) { append(a); }

    void pop_back();
    void pop(size_t i) { remove(i); }
    void remove(size_t i);

    void clear()
    { actuals = 0; actuators.fill({MInvalid,0,0}); _validated = false; }

    tstring tstr() const
    {
        tstringstream ss;
        ss << *this;
        return ss.str();
    }

    //----------------------------------------------------
    Actuator& operator[](size_t i)
    {
        _validated = false;
        return actuators[(i < actuals) ? i : (actuals - 1)];
    };
    const Actuator& operator[](size_t i) const
    { return actuators[(i < actuals) ? i : (actuals - 1)]; };

    //----------------------------------------------------
    bool  operator== (const Control &c) const;
    bool  operator!= (const Control &c) const
    { return !(*this == c); }
    bool  operator== (const muscle_t m) const
    { return  (actuals == 1 && actuators[0].muscle == m); }
    bool  operator!= (const muscle_t m) const
    { return  !(*this == m); }

    const Control& operator+=(const Control &c)
    {
        for (const auto &a : c)
            if (a.muscle != MInvalid && a.lasts != 0)
                append(a);
            else
                CWARN("muscle==MInvalid or last==0");
        return *this;
    }

    const Actuator& front() const { return actuators[0]; };
    const Actuator& back() const { return actuators[actuals-1]; };
    Actuator& front() { return actuators[0]; };
    Actuator& back() { return actuators[actuals-1]; };
    //----------------------------------------------------
    void removeStartPause();
    bool validateMusclesTimes() const;
    bool validate(Robo::muscle_t n_muscles) /*nothrow*/ const;
    void validated(Robo::muscle_t n_muscles) const;
    
    muscle_t select(muscle_t muscle) const;

    //----------------------------------------------------
    void fillRandom(muscle_t muscles_count,
                    const std::function<frames_t(muscle_t)> &muscleMaxLasts,
                    frames_t lasts_min = 50,
                    unsigned moves_count_min = 1,
                    unsigned moves_count_max = 3,
                    bool simul = true);
    //----------------------------------------------------
    friend tostream& operator<<(tostream&, const Control&);
    friend tistream& operator>>(tistream&, Control&);
    //----------------------------------------------------
    friend std::ostream& operator<<(std::ostream &s, const Control &controls)
    {
        controls.stream(s);
        return s;
    }
};
//-------------------------------------------------------------------------------
inline Control EmptyMov() { return {}; }
//-------------------------------------------------------------------------------
inline Control operator+(const Control &cl, const Control &cr)
{ 
    Control c = cl;
    for (auto &a : cr)
        if (a.muscle != MInvalid && a.lasts != 0)
            c.append(a);
        else CWARN("muscle==MInvalid or last==0");
    return c;
}
inline Control operator+(const Control &cl, const Actuator &a)
{
    Control c = cl;
    if (a.muscle != MInvalid && a.lasts != 0)
        c.append(a);
    else CWARN("muscle==MInvalid or last==0");
    return c;
}
inline Control operator+(const Control &cl, const std::vector<Actuator> &v)
{
    Control c = cl;
    for (auto &a : v)
        if (a.muscle != MInvalid && a.lasts != 0)
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
