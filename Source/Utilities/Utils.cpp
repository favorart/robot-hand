#include "StdAfx.h"


namespace Utils {
//------------------------------------------------------------------------------
/*  Размедение с повторениями:
 *    currents   - текущее размещение
 *    n_alphabet - размер алфавита { 0, 1, ... k }
 */
bool next_placement_repeats(std::vector<int> &currents, int n_alphabet)
{
    bool result = true;
    size_t position = currents.size();
    while (position > 0U && result)
    {
        ++currents[position - 1U];
        result = (currents[position - 1U] == n_alphabet);
        if (result)
        { currents[position - 1U] = 0U; }
        --position;
    }
    return (!result);
}

//------------------------------------------------------------------------------
void inputAngles(const tstring &filename, std::vector<double> &angles)
{
    std::ifstream fin(filename);
    // --------------------------------
    if (fin.is_open())
    {
        size_t mii;
        if (fin >> mii && mii > 0)
            angles.reserve(mii);
        else throw std::exception("ifstream: input error.");
        // --------------------------------
        for (size_t i = 0; i < mii; ++i)
        {
            double angle;
            if (fin >> angle)
                angles.push_back(angle);
            else throw std::exception("ifstream: input error.");
        }
    }
    else throw std::exception("ifstream: input error.");
}

void outputAngles(const tstring &filename, const std::vector<double> &angles)
{
    std::ofstream fout(filename + _T("-out.txt"));
    // --------------------------------
    if (fout.is_open())
    {
        fout << angles.size() << ' ';
        // --------------------------------
        for (auto & a : angles)
            fout << a << ' ';
        fout << std::endl;
    }
    else throw std::exception("ifstream: output error.");
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// However, the double allocation and the need to delete the buffer 
// concern me(performance and exception safety) so I modified them to be like this:
std::wstring s2ws(const std::string& s)
{
    int slength = (int)s.length() + 1;
    int rlength = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0);
    wchar_t* buf = new wchar_t[rlength];
    MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, rlength);
    std::wstring r(buf);
    delete[] buf;
    return r;
}
std::string ws2s(const std::wstring& s)
{
    int slength = (int)s.length() + 1;
    int rlength = WideCharToMultiByte(CP_ACP, 0, s.c_str(), slength, 0, 0, 0, 0);
    char* buf = new char[rlength];
    WideCharToMultiByte(CP_ACP, 0, s.c_str(), slength, buf, rlength, 0, 0);
    std::string r(buf);
    delete[] buf;
    return r;
}
//------------------------------------------------------------------------------
tstring uni(const std::string& s)
{
#if defined(UNICODE) || defined(_UNICODE)
    return s2ws(s);
#else
    return s;
#endif
}
tstring uni(const std::wstring& s)
{
#if defined(UNICODE) || defined(_UNICODE)
    return s;
#else
    return ws2s(s);
#endif
}
//------------------------------------------------------------------------------
}
