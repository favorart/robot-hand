#include "StdAfx.h"

#include <shellapi.h>
#include <fcntl.h>
#include <io.h>


//-------------------------------------------------------------------------------
#if defined(UNICODE) || defined(_UNICODE)
#define tfdopen _wfdopen
#define _O_TTEXT _O_WTEXT
#define CommandLineToArgvT CommandLineToArgvW
#else
#define tfdopen _fdopen
#define _O_TTEXT _O_TEXT
#define CommandLineToArgvT CommandLineToArgvA
#endif
//-------------------------------------------------------------------------------
void redirectConsoleIO()
{
    // allocate a console for this app
    AllocConsole();

    CONSOLE_SCREEN_BUFFER_INFO coninfo;
    // set the screen buffer to be big enough to let us scroll text
    GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &coninfo);

    // maximum mumber of lines the output console should have
    const WORD MAX_CONSOLE_LINES = 150U;
    coninfo.dwSize.Y = MAX_CONSOLE_LINES;
    SetConsoleScreenBufferSize(GetStdHandle(STD_OUTPUT_HANDLE), coninfo.dwSize);

    // redirect unbuffered STDOUT to the console
    intptr_t lStdHandle = (intptr_t)GetStdHandle(STD_OUTPUT_HANDLE);
    int hConHandle = _open_osfhandle(lStdHandle, _O_TTEXT);

    //FILE *fp;
    //fp = _fdopen (hConHandle, "w");
    //*stdout = *fp;
    //setvbuf (stdout, NULL, _IONBF, 90);

    // redirect unbuffered STDIN to the console
    lStdHandle = (intptr_t)GetStdHandle(STD_INPUT_HANDLE);
    hConHandle = _open_osfhandle(lStdHandle, _O_TTEXT);

    //fp = tfdopen (hConHandle, _T("r"));
    //*stdin = *fp;
    //setvbuf (stdin, NULL, _IONBF, 90);

    // redirect unbuffered STDERR to the console
    lStdHandle = (intptr_t)GetStdHandle(STD_ERROR_HANDLE);
    hConHandle = _open_osfhandle(lStdHandle, _O_TTEXT);

    //fp = tfdopen (hConHandle, _T("w"));
    //*stderr = *fp;
    //setvbuf (stderr, NULL, _IONBF, 90);

    freopen("CONIN$",  "r", stdin);
    freopen("CONOUT$", "w", stdout);
    freopen("CONOUT$", "w", stderr);

    // make cout, wcout, cin, wcin, wcerr, cerr, wclog and clog
    // point to console as well
    std::ios::sync_with_stdio();

    tcout << "streams" << std::endl;
    _tprintf(_T("redirected..\n"));
    //fflush(stdout);
}
//-------------------------------------------------------------------------------
#include "Utils.h"
#include <boost/program_options.hpp>
namespace po = boost::program_options;

#if defined(UNICODE) || defined(_UNICODE)
using po_tparser = po::basic_command_line_parser<TCHAR>;
using po_tparsed_options = po::wparsed_options;

#define po_tvalue po::wvalue
#else
using po_tparser = po::basic_command_line_parser<TCHAR>;
using po_tparsed_options = po::parsed_options;

#define po_tvalue po::value
#endif

//-------------------------------------------------------------------------------
void getConsoleArguments(Utils::CArgs &args)
{
    int argc;
    LPTSTR *lpArgv = CommandLineToArgvT(GetCommandLine(), &argc);
    // wcstombs ( wstring --> string )
    
    {
        po::options_description desc("Global params:");
        desc.add_options()
            ("help,h", "Help menu")
            ("config_path,c", po_tvalue(&args.config), "Path to json config file")
            ("database_path,d", po_tvalue(&args.database)->default_value(_T(""), ""), "Path to trajectories database")
            ("lm_conf_path,l", po_tvalue(&args.lm_config)->default_value(_T(""), ""), "Path to LM config file")
            ("tests_path,p", po_tvalue(&args.testsfile)->default_value(_T(""), ""), "Path to Tests file")
            ("test_name,n", po_tvalue(&args.testname)->default_value(_T(""), ""), "Prefix of Test name")
            ("test,t", po::bool_switch(&args.testings)->default_value(false)->implicit_value(true), "Testing regime");
        
        po::variables_map vm; // здесь будут значения артументов, если не указать контейнер в po::value
        try
        {
            po_tparser parser(argc, lpArgv);
            parser.options(desc);
            parser.style(po::command_line_style::default_style);
            po_tparsed_options parsedOptions = parser.run();
            po::store(parsedOptions, vm);
            po::notify(vm);
        }
        catch (std::exception& e)
        {
            std::stringstream ss;
            ss << e.what() << std::endl << desc << std::endl;
            tstring lines = Utils::uni(ss.str());

            tcout << _T("2D-Robotic Simulator.\nScientific development machine learning algorithm for control robots.\n") << std::endl;
            tcout << lines << std::endl;
        }
    }
    LocalFree(lpArgv);
}
//-------------------------------------------------------------------------------
