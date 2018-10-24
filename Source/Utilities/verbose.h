#pragma once

extern int LV_CLEVEL;

constexpr int LV_OUTALL = 0;
constexpr int LV_CDEBUG = 1;
constexpr int LV_CINFO  = 2;
constexpr int LV_CWARN  = 3;
constexpr int LV_CERROR = 4;
constexpr int LV_CALERT = 5;

#define _CVERBOSE_(LV,message)       { if (LV_CLEVEL <= LV)                         \
                                       {                                            \
                                           fs::path p(__FILE__);                    \
                                           tcout << p.filename() << _T("@")         \
                                                 << __LINE__ << _T(": ") << message \
                                                 << std::endl;                      \
                                       }                                            \
                                     }
#define _CWARNING_(LV,message,title) { if (LV_CLEVEL <= LV)                         \
                                       {                                            \
                                           fs::path p(__FILE__);                    \
                                           tstring last_err = getLastErrorString(); \
                                           tcout << __FUNCTION__ << _T("() ")       \
                                                 << p.filename() << _T("@")         \
                                                 << __LINE__ << _T(": ")            \
                                                 << std::endl << message            \
                                                 << std::endl << last_err           \
                                                 << std::endl;                      \
                                           tcerr << __FUNCTION__ << _T("() ")       \
                                                 << p.filename() << _T("@")         \
                                                 << __LINE__ << _T(": ")            \
                                                 << std::endl << message            \
                                                 << std::endl << last_err           \
                                                 << std::endl;                      \
                                                                                    \
                                           if (title != _T("WARNING"))              \
                                           {                                        \
                                              tstringstream ss;                     \
                                              ss << message  << std::endl           \
                                                 << last_err << std::endl;          \
                                              MessageBoxW(NULL,                     \
                                                          ss.str().c_str(),         \
                                                          title,                    \
                                                          MB_OK | MB_ICONERROR);    \
                                           }                                        \
                                       }                                            \
                                        /* throw std::exception(); */               \
                                     }

#define CDEBUG(msg)    _CVERBOSE_(LV_CDEBUG,msg)
#define CINFO(msg)     _CVERBOSE_(LV_CINFO,msg)
#define CWARN(msg)     _CWARNING_(LV_CWARN,msg,_T("WARNING"))
#define CERROR(msg)    _CWARNING_(LV_CERROR,msg,_T("ERROR"))
#define CALERT(msg)    _CWARNING_(LV_CALERT,msg,_T("ALERT"))

//inline void CDEBUG(const char*tstring s) {}
//inline void CINFO() {}
//inline void CWARN() {}
//inline void CERROR() {}
//inline void CALERT() {}


#ifdef REPORT
// class to capture the caller and print it.  
class Reporter
{
public:
    Reporter(std::string Caller, std::string File, int Line)
        : caller_(Caller)
        , file_(File)
        , line_(Line)
    {}

    int operator()(int one, int two)
    {
        std::cout
            << "Reporter: FunctionName() is being called by "
            << caller_ << "() in " << file_ << ":" << line_ << std::endl;
        // can use the original name here, as it is still defined
        return FunctionName(one, two);
    }
private:
    std::string   caller_;
    std::string   file_;
    int           line_;

};

// remove the symbol for the function, then define a new version that instead
// creates a stack temporary instance of Reporter initialized with the caller
#  undef FunctionName
#  define FunctionName Reporter(__FUNCTION__,__FILE__,__LINE__)
#endif

//-------------------------------------------------------------------------------
template <typename Base>
class Factory
{
public:
    static std::shared_ptr<Base> create(tptree &root)
    {
        auto &node = root.get_child_optional(Base::name()).get_value_or(root);
        tstring type = node.get<tstring>(_T("type"));
        for (auto &make : makes)
        {
            auto t = make(type, node);
            if (t) return t;
        }
        CWARN(_T("No make function for type=") << type);
        return std::shared_ptr<Base>(nullptr);
    }
private:
    static std::vector<std::function<std::shared_ptr<Base>(const tstring&, tptree&)>> makes;
};

