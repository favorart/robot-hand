

int LV_CLEVEL = LV_CINFO;
//------------------------------------------------------
tstring putVerboseLevel() { return VerboseLevelOutputs[LV_CLEVEL]; }
//------------------------------------------------------
int scanVerboseLevel(const tstring &str_level)
{
    int level = 0;
    tstring buf{ unquote(str_level) };
    boost::trim(buf);
    for (auto &s : VerboseLevelOutputs)
    {
        if (buf == s)
            return level;
        ++level;
    }
    CERROR("Invalid Verbose Level!");
    return LV_CINFO;
}

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
        else throw std::runtime_error("ifstream: input error.");
        // --------------------------------
        for (size_t i = 0; i < mii; ++i)
        {
            double angle;
            if (fin >> angle)
                angles.push_back(angle);
            else throw std::runtime_error("ifstream: input error.");
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
    else throw std::runtime_error("ofstream: output error.");
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
tstring Utils::uni(const std::string& s)
{
#if defined(UNICODE) || defined(_UNICODE)
    return s2ws(s);
#else
    return s;
#endif
}
tstring Utils::uni(const std::wstring& s)
{
#if defined(UNICODE) || defined(_UNICODE)
    return s;
#else
    return ws2s(s);
#endif
}
std::string Utils::ununi(const tstring& s)
{
#if !defined(UNICODE) || !defined(_UNICODE)
    return s;
#else
    return ws2s(s);
#endif
}
//------------------------------------------------------------------------------
tstring Utils::format(const TCHAR *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    std::vector<TCHAR> v(1024);
    while (true)
    {
        va_list args2;
        va_copy(args2, args);
        int res = _vsntprintf(v.data(), v.size(), fmt, args2);
        if ((res >= 0) && (res < static_cast<int>(v.size())))
        {
            va_end(args);
            va_end(args2);
            return tstring{ v.data() };
        }
        size_t size;
        if (res < 0)
            size = v.size() * 2;
        else
            size = static_cast<size_t>(res) + 1;
        v.clear();
        v.resize(size);
        va_end(args2);
    }
}
//-------------------------------------------------------------------------------
/// Create a string with last error message
tstring getLastErrorString()
{
    DWORD error = GetLastError();
    if (error)
    {
        LPVOID lpMsgBuf;
        DWORD bufLen = FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER |
                                      // FORMAT_MESSAGE_IGNORE_INSERTS |
                                      FORMAT_MESSAGE_FROM_SYSTEM,
                                      NULL,
                                      error,
                                      MAKELANGID(LANG_SYSTEM_DEFAULT/*LANG_NEUTRAL*/, SUBLANG_DEFAULT),
                                      (LPTSTR)&lpMsgBuf,
                                      0, NULL);
        if (bufLen > 1)
        {
            LPCSTR lpMsgStr = (LPCSTR)lpMsgBuf;
            tstring result(lpMsgStr, lpMsgStr + bufLen);
            LocalFree(lpMsgBuf);
            return result;
        }
    }
    return tstring{};
}
//-------------------------------------------------------------------------------
tstring getCurrentTimeString(tstring format, std::time_t *the_time)
{
    std::time_t rawtime;
    if (!the_time)
    { rawtime = std::time(nullptr); }
    else
    { rawtime = *the_time; }
    struct tm  *TimeInfo = std::localtime(&rawtime);

    tstringstream ss;
    ss << std::put_time(TimeInfo, format.c_str());
    return ss.str();
}
//-------------------------------------------------------------------------------
std::string Utils::now()
{
    time_t rawtime;
    struct tm *timeinfo;
    char buffer[80];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer, sizeof(buffer), "%Y.%m.%d-%H.%M.%S", timeinfo);
    return { buffer };
}
//-------------------------------------------------------------------------------
tfstream Utils::utf8_stream(const tstring &fn, int mode)
{
    //https://msdn.microsoft.com/en-us/library/7dd6d271-472d-4750-8fb5-ea8f55fbef62.aspx
    const std::locale empty_locale = std::locale::empty(); //locale::global() might works as well
    using converter_type = std::codecvt_utf8<TCHAR>;
    const converter_type* converter = new converter_type;
    const std::locale utf8_locale = std::locale(empty_locale, converter);
    tfstream stream(fn, mode);
    stream.imbue(utf8_locale); // replaces the current locale
    //tstring line;
    //std::getline(stream, line);
    return stream;
}
//-------------------------------------------------------------------------------
