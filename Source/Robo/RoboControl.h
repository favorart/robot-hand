#pragma once

namespace Robo
{
//-------------------------------------------------------------------------------
using frames_t = size_t;   ///< descrete time
using muscle_t = uint8_t;  ///< muscle index (no holes)
//-------------------------------------------------------------------------------
class RoboI;
const muscle_t MInvalid = 0xFF;
const frames_t LastsInfinity = std::numeric_limits<frames_t>::max(); //-1
//-------------------------------------------------------------------------------
struct Actuator final
{
    muscle_t  muscle = MInvalid;
    frames_t  start = 0;
    frames_t  lasts = 0;
    //----------------------------------------------------
    Actuator() = default;
    Actuator(muscle_t muscle, frames_t start, frames_t lasts) :
        muscle(muscle), start(start), lasts(lasts) {}
    Actuator(muscle_t muscle, frames_t start, frames_t lasts, int nmuscles) :
        muscle(muscle), start(start), lasts(lasts)
    { if (nmuscles <= muscle) throw std::runtime_error("Actuator: muscle > nmuscles"); }
    Actuator(Actuator&&) = default;
    Actuator(const Actuator&) = default;
    Actuator& operator=(const Actuator&) = default;
    //----------------------------------------------------
    bool operator<  (const Actuator &a) const;
    bool operator>  (const Actuator &a) const
    { return !(*this < a) && !(*this == a); }
    bool operator<= (const Actuator &a) const
    { return (*this < a) || (*this == a); }
    bool operator>=  (const Actuator &a) const
    { return !(*this < a); }

    bool operator== (const Actuator &a) const
    { return (this == &a) || (muscle == a.muscle && start == a.start && (lasts <= a.lasts || a.lasts <= lasts)); }
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
    void serialize(Archive &ar, unsigned /*version*/)
    { ar & muscle & start & lasts; }

    bool intersect(const Robo::Actuator&) const;
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
class Control final
{
protected:
    static const unsigned MAX_ACTUATORS = 128;       ///< number of brakes
    std::array<Actuator, MAX_ACTUATORS> actuators{}; ///< sorted by start
    size_t actuals = 0;
    mutable bool _validated = false;

    // ----------------------------------------
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, unsigned /*version*/)
    { ar & actuals; for (auto &a : actuators) ar & a; }
    // ----------------------------------------
    std::string str() const { std::stringstream ss; stream(ss); return ss.str(); }
    std::ostream& stream(std::ostream&) const;
    //----------------------------------------------------

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

    auto  begin()       -> decltype(boost::begin(actuators));
    auto  begin() const -> decltype(boost::begin(actuators));
    auto    end()       -> decltype(boost::end(actuators));
    auto    end() const -> decltype(boost::end(actuators));
    auto rbegin()       -> decltype(boost::rbegin(actuators));
    auto rbegin() const -> decltype(boost::rbegin(actuators));
    auto   rend()       -> decltype(boost::rend(actuators));
    auto   rend() const -> decltype(boost::rend(actuators));
    //----------------------------------------------------
    size_t size() const { return (actuals); }
    void append(const Actuator& a); // sorted
    void push_back(const Actuator& a) { append(a); } // sorted

    void shorter (size_t index, frames_t velosity, bool infl_oppo_start = false, bool infl_oppo_lasts = false);
    void longer  (size_t index, frames_t velosity, bool infl_oppo_start = false);

    void pop_back();
    void pop(size_t i) { remove(i); }
    void remove(size_t i);
    void remove(const Actuator*);

    bool find(const Actuator&) const;
    void clear()
    { actuals = 0; actuators.fill({MInvalid,0,0}); _validated = false; }
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

    const Control& operator+=(const Control &cl) { return (*this + cl); }
    const Control& operator+=(const Actuator &a) { return (*this + a); }

    const Actuator& front() const { return actuators[0]; };
    const Actuator& back() const { return actuators[actuals-1]; };
    Actuator& front() { return actuators[0]; };
    Actuator& back() { return actuators[actuals-1]; };
    //----------------------------------------------------
    void removeStartPause();
    bool intersectMusclesOpposites() const;
    void order(Robo::muscle_t n_muscles, bool keep_actors=true) /*nothrow*/;
    bool validate(Robo::muscle_t n_muscles) /*nothrow*/ const;
    void validated(Robo::muscle_t n_muscles) /*!throw*/ const;
    //----------------------------------------------------
    using MaxMuscleCount = std::function<frames_t(muscle_t)>;
    void fillRandom(muscle_t n_muscles,
                    const MaxMuscleCount &muscleMaxLasts,
                    frames_t lasts_min = 50,
                    unsigned min_n_moves = 1,
                    unsigned max_n_moves = 3,
                    bool simul = true);
    //----------------------------------------------------
    std::vector<Actuator> v() const { return std::vector<Actuator>(begin(),end()); }
    std::vector<Actuator> align() const; // <<<< TODO:
    //----------------------------------------------------
    friend tostream& operator<<(tostream&, const Control&);
    friend tistream& operator>>(tistream&, Control&);
    //----------------------------------------------------
    tstring tstr() const { tstringstream ss; ss << *this; return ss.str(); }
    //----------------------------------------------------
    friend std::ostream& operator<<(std::ostream &s, const Control &controls)
    {
        controls.stream(s);
        return s;
    }
    
    friend Control operator+(const Control&, const Control&);
    friend Control operator+(const Control&, const Actuator&);
    friend Control operator+(const Control&, const std::vector<Actuator>&);
    //-------------------------------------------------------------------------------
    static Control EmptyMove() { return Control{}; }
};
//-------------------------------------------------------------------------------

/**  матрица: 
*         строки - битовые входы приводов,
*         столбцы - такты по времени работы робота.
*    параметр N - число доступных приводов.
*/
template <size_t N>
class BitsControl final
{
public:
    using Bitwise = std::bitset<N>;
    using Episode = std::vector<Bitwise>;
protected:
    Episode episode{};
public:
    BitsControl() {}
    BitsControl(Robo::frames_t episode_length) : episode(episode_length, Bitwise{}) {}
    BitsControl(const Episode &episode) : episode(episode) {}
    //-------------------------------------------------------------------------------
    using iterator = typename Episode::iterator;
    using const_iterator = typename Episode::const_iterator;

    iterator       begin()       { return std::begin(episode); }
    const_iterator begin() const { return std::begin(episode); }
    iterator       end()         { return std::begin(episode) + episode.size(); } // ?? std::advance()
    const_iterator end()   const { return std::begin(episode) + episode.size(); } // ?? std::advance()
    //-------------------------------------------------------------------------------
    Bitwise& operator[](Robo::frames_t frame)
    {
        if (frame < episode.size())
            return episode[frame];
        CERROR("BitsControl: invalid index");
    }
    const Bitwise& operator[](Robo::frames_t frame) const { return (*this)[frame]; }
    //-------------------------------------------------------------------------------
    bool validate() /*nothrow*/ const
    { return (episode.size() > 0 && episode.front() != 0 && episode.back() != 0); }
    void validated() /*!throw*/ const
    { if (!validate()) throw std::logic_error("BitsControl not validated"); }
    //-------------------------------------------------------------------------------
    friend BitsControl operator&(const BitsControl &l, const BitsControl &r)
    {
        BitsControl res;
        res.episode.reserve(std::max(l.size(), r.size()));

        for (const auto &pair : boost::combine(l, r))
        {
            const Robo::bitwise &il, &il;
            std::tie(il, il) = pair;
            res.episode.push_back(il & ir);
        }
        return res;
    }
    friend BitsControl operator|(const BitsControl &l, const BitsControl &r)
    {
        BitsControl res;
        res.episode.reserve(std::max(l.size(), r.size()));

        for (const auto &pair : boost::combine(l, r))
        {
            const Robo::bitwise &il, &il;
            std::tie(il, il) = pair;
            res.episode.push_back(il | ir);
        }
        return res;
    }
    //friend BitsControl operator||(const BitsControl &l, const BitsControl &r, frames_t particle=10) // prefix??
    //{
    //    BitsControl res;
    //    return res;
    //}
    //-------------------------------------------------------------------------------
    static BitsControl EmptyMove() { return BitsControl{}; }
};
} // namespace Robo
//-------------------------------------------------------------------------------
BOOST_CLASS_VERSION(Robo::Actuator, 2)
BOOST_CLASS_VERSION(Robo::Control, 2)
//BOOST_CLASS_VERSION(Robo::BitsControl, 1)
//-------------------------------------------------------------------------------
