#pragma once

#include "verbose.h"
//#include <numeric>
namespace Utils {
//constexpr double EPSILONT      = std::numeric_limits<double>::min * 10;
constexpr double EPSILONT        = 1e-7;
constexpr double EPSILONT_VISUAL = 1e-2;

//-------------------------------------------------------------------------------
template <typename Integer>
Integer  random(Integer max)
{ return  (max) ? (static_cast<Integer> (rand()) % (max)) : (max); }
template <typename Integer>
Integer  random(Integer min, Integer max)
{ return  (max) ? (static_cast<Integer> (rand()) % (max)+min) : (max); }

//-------------------------------------------------------------------------------
inline double  random(double max)
{ return  (static_cast<double> (rand()) / RAND_MAX) * max; }
inline double  random(double min, double max)
{ return  (static_cast<double> (rand()) / RAND_MAX) * (max - min) + min; }

//-------------------------------------------------------------------------------
template<typename INTEGER>
INTEGER interval_map(INTEGER input, std::pair<INTEGER, INTEGER> inv_in, std::pair<INTEGER, INTEGER> inv_out)
{ return (inv_out.second - inv_out.first) * (input - inv_in.first) / (inv_in.second - inv_in.first) + inv_out.first; }

tfstream utf8_stream(const tstring &fn, int mode);
//-------------------------------------------------------------------------------
tstring uni(const std::string&);
tstring uni(const std::wstring&);

std::string ununi(const tstring& s);
//-------------------------------------------------------------------------------
tstring format(const TCHAR *fmt, ...);
//-------------------------------------------------------------------------------
std::string now();
//-------------------------------------------------------------------------------
struct CArgs
{
    bool testings{ false };
    tstring config{ _T("") };
    tstring database{ _T("") };
    tstring testsfile{ _T("") };
    tstring testname{ _T("") };
    tstring lm_config{ _T("") };
};
} // Utils

//------------------------------------------------------
using tseparator = boost::char_separator<TCHAR>;
using ttokenizer = boost::tokenizer<boost::char_separator<TCHAR>, tstring::const_iterator, tstring>;
//------------------------------------------------------
template <typename Enum, size_t N = size_t(Enum::_LAST_)>
Enum scanEnumOneHot(const tstring &buf, const std::array<const TCHAR*, N> &outputs)
{
    if (buf == outputs[0])
        return Enum(0);
    tseparator sep(_T("|"));
    ttokenizer tokens(buf, sep);
    uint64_t e = 0;
    auto beg = std::cbegin(outputs);
    beg++;
    for (auto &tok : tokens)
    {
        auto it = std::find(beg, outputs.cend(), tok);
        if (it == outputs.cend())
            CERROR("Invalid " << typeid(Enum).name() << ' ' << tok << ' ' << std::distance(beg, it));
        e |= (1ULL << std::distance(beg, it));
        //tcout << *it << ' ' << int(e) << ' ';
    }
    return Enum(e);
}
//------------------------------------------------------
template <typename Enum, size_t N = size_t(Enum::_LAST_)>
void printEnumOneHot(Enum enum_val, const std::array<const TCHAR*, N> &outputs)
{
    if (enum_val == Enum(0))
    {
        tcout << outputs[0];
        return;
    }
    for (uint64_t e = 1, i = 0; i < N; ++i, e <<= 1)
        if (e & uint64_t(enum_val))
            tcout << ((i == 1) ? _T("") : _T("|")) << outputs[i];
}

//-------------------------------------------------------------------------------
tstring getLastErrorString();
tstring getCurrentTimeString(tstring format, std::time_t *the_time);
//-------------------------------------------------------------------------------
class ConfirmInput
{
    const TCHAR *_confirm;
public:
    ConfirmInput(const TCHAR *confirm) : _confirm(confirm) {}
    friend tistream& operator>>(tistream &s, const ConfirmInput &ci)
    {
        TCHAR c;
        for (const TCHAR* p = ci._confirm; *p; ++p)
        {
            if (std::isspace(*p))
                tistream::sentry k(s); // discard whitespace
            else if ((c = s.get()) != *p)
                s.setstate(std::ios::failbit); // stop extracting
        }
        return s;
    }
};
//-------------------------------------------------------------------------------
