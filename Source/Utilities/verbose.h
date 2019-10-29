#pragma once

extern int LV_CLEVEL;

constexpr int LV_OUTALL = 0;
constexpr int LV_CDEBUG = 1;
constexpr int LV_CINFO  = 2;
constexpr int LV_CWARN  = 3;
constexpr int LV_CERROR = 4;
constexpr int LV_CALERT = 5;

//-------------------------------------------------------------------------------
#define _CVERBOSE_(LV,message)       { if (LV_CLEVEL <= LV)                         \
                                       {                                            \
                                           bfs::path p(__FILE__);                   \
                                           tcout << p.filename() << _T("@")         \
                                                 << __LINE__ << _T(": ") << message \
                                                 << std::endl;                      \
                                       }                                            \
                                     }
#define _CWARNING_(LV,message,title) { if (LV_CLEVEL <= LV)                         \
                                       {                                            \
                                           bfs::path p(__FILE__);                   \
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
                                           if (LV >= LV_CERROR)                     \
                                           {                                        \
                                              tstringstream ss;                     \
                                              ss << message  << std::endl           \
                                                 << last_err << std::endl;          \
                                              MessageBox(NULL,                      \
                                                         ss.str().c_str(),          \
                                                         title,                     \
                                                         MB_OK | MB_ICONERROR);     \
                                              throw std::runtime_error("");         \
                                           }                                        \
                                       }                                            \
                                     }
#define SHOW_CERROR(message)         { tstring msg = Utils::uni({ message });       \
                                       if (msg.length() > 0)                        \
                                       {                                            \
                                           bfs::path p(__FILE__);                   \
                                           tcout << __FUNCTION__ << _T("() ")       \
                                                 << p.filename() << _T("@")         \
                                                 << __LINE__ << _T(": ")            \
                                                 << msg << std::endl;               \
                                           tcerr << __FUNCTION__ << _T("() ")       \
                                                 << p.filename() << _T("@")         \
                                                 << __LINE__ << _T(": ")            \
                                                 << msg << std::endl;               \
                                            MessageBox(NULL,msg.c_str(),_T("error"),\
                                                       MB_OK | MB_ICONERROR);       \
                                        }                                           \
                                     }

//-------------------------------------------------------------------------------
#define CDEBUG(msg)    _CVERBOSE_(LV_CDEBUG,msg)
#define CINFO(msg)     _CVERBOSE_(LV_CINFO,msg)
#define CWARN(msg)     _CWARNING_(LV_CWARN,msg,_T("WARNING"))
#define CERROR(msg)    _CWARNING_(LV_CERROR,msg,_T("ERROR"))
#define CALERT(msg)    _CWARNING_(LV_CALERT,msg,_T("ALERT"))

//-------------------------------------------------------------------------------
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
    using MakeMethods = std::vector<std::function<std::shared_ptr<Base>(const tstring&, tptree&)>>;
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
    static MakeMethods makes;
};

//-------------------------------------------------------------------------------
#define STRINGIFY(s) #s
#define NORM_NAME(P) ((STRINGIFY(P)[0]=='_')?tstring(_T(STRINGIFY(P))).substr(1,strlen(STRINGIFY(P))-1):tstring(_T(STRINGIFY(P))))

#define CONF_GET_OPT_SCOPE_VAL(config, name, scope, val)  name=config.get_optional<decltype(name)>(_T(STRINGIFY(scope) ".")+NORM_NAME(name)).get_value_or(val)
#define CONF_GET_OPT_VAL(config, name, val)               name=config.get_optional<decltype(name)>(NORM_NAME(name)).get_value_or(val)

#define CONF_GET_OPT_SCOPE(config, name, scope)           name=config.get_optional<decltype(name)>(_T(STRINGIFY(scope) ".")+NORM_NAME(name)).get_value_or(name)
#define CONF_GET_OPT(config, name)                        name=config.get_optional<decltype(name)>(NORM_NAME(name)).get_value_or(name)

#define CONF_GET_SCOPE(config, name, scope)               name=config.get<decltype(name)>(_T(STRINGIFY(scope) ".")+NORM_NAME(name))
#define CONF_GET(config, name)                            name=config.get<decltype(name)>(NORM_NAME(name))

#define CONF_PUT_SCOPE(config, name, scope)                    config.put(_T(STRINGIFY(scope) ".")+NORM_NAME(name), name)
#define CONF_PUT(config, name)                                 config.put(NORM_NAME(name), name)
//-------------------------------------------------------------------------------

