#include "StdAfx.h"

#ifndef  _WINDOW_DATA_H_
#define  _WINDOW_DATA_H_

#include "WindowHeader.h"
#include "Robo.h"
//------------------------------------------------------------------------------
namespace RoboMoves {
class Record;
class Store;
}
namespace RoboPos {
class LearnMoves;
}
class RecTarget;
class CanvasScaleLetters;

namespace Utils {
tstring uni(const std::string& s);
tstring uni(const std::wstring& s);
};
//------------------------------------------------------------------------------
struct MyWindowData
{
    struct MouseHandler
    {
        bool       click{ false };
        win_point  coords{}; /* координаты мыши в пикселях */
        Point      aim{};
    } mouse;
    // ---------------------------------
    struct Canvas
    {
        HWND    hLabMAim, hLabTest, hLabStat;
        HPEN    hPen_red, hPen_grn, hPen_blue, hPen_cian, hPen_orng;
        HBRUSH  hBrush_white, hBrush_null, hBrush_back;
        // ---------------------------------
        HDC     hStaticDC{ NULL };
        HBITMAP hStaticBitmap{ NULL };
        bool    hStaticBitmapChanged{ true };
        // ---------------------------------
        std::shared_ptr<CanvasScaleLetters> pLetters{};
        // ---------------------------------
        bool              workingSpaceShow{ false };
        Robo::Trajectory  workingSpaceTraj{};
        // --- adjacency -------------------
        bool allPointsDBShow{ false };
        size_t store_size{ 0 };

        std::list<std::shared_ptr<RoboMoves::Record>> pointsDB{};
        std::list<std::shared_ptr<Robo::Trajectory>>   trajsDB{};
        // ---------------------------------
        Robo::Trajectories testingTrajsList{};
        bool               testingTrajsShow{ false };
        // ---------------------------------
        std::list<Point> uncoveredPointsList{};
        bool             uncoveredPointsShow{ true };
        // --------------------------------
    } canvas;

    // --- show_frames_trajectory ------
    class TrajectoryFrames
    {
        const size_t skip_show_steps = 15U;
        // ---------------------------------
        bool show_ = false;
        bool animation_ = true;
        Robo::Trajectory trajectory_ = {};
        // ---------------------------------
        size_t     control_cur_ = 0;
        Robo::Control controls_ = {};
        Robo::frames_t  frames_ = 0;
        Point         base_pos_ = {};
        // ---------------------------------

    public:
        // ---------------------------------
        // ---------------------------------
        /* Microsoft specific: C++ properties */
        __declspec(property(get = _get_traj, put = _put_traj)) const Robo::Trajectory &trajectory;
        const Robo::Trajectory&   _get_traj() const { return trajectory_; }
        void _put_traj(const Robo::Trajectory& traj) { show_ = true; trajectory_ = traj; }

        __declspec(property(get = _get_anim, put = _put_anim))  bool animation;
        bool _get_anim() const { return animation_; }
        void _put_traj(bool anim) { animation_ = anim; }

        __declspec(property(get = _get_show))  bool show;
        bool _get_show() const { return show_; }
        // ---------------------------------

        void  clear();
        void  draw(HDC hdc, HPEN hPen) const;
        void  step(RoboMoves::Store &store, Robo::RoboI &robo,
                   const boost::optional<Robo::Control> controls = boost::none);
    };
    TrajectoryFrames trajFrames;
    // ---------------------------------
    std::shared_ptr<boost::thread> pWorkerThread;
    // ---------------------------------
    std::shared_ptr<RecTarget> pTarget;
    std::shared_ptr<Robo::RoboI> pRobo;
    std::shared_ptr<RoboMoves::Store> pStore;
    std::shared_ptr<RoboPos::LearnMoves> pLM;
    // ---------------------------------
    static bool zoom;
    bool testing = false;
    Robo::frames_t frames = 0;
    // ---------------------------------
    size_t   complexity = 0;
    tstring  currFileName{};
    // ---------------------------------
    struct StoreSearch
    {
        const double radius = 0.05;
        const double side = 0.1;
    } search;
    // ---------------------------------
    MyWindowData(const tstring &config, const tstring &database);
    ~MyWindowData();
    // ---------------------------------
    void read_config(IN const tstring &filename);
    void write_config(IN const tstring &filename) const;
};
//-------------------------------------------------------------------------------
template<typename Function, typename... Args> inline
void  WorkerThreadRunTask (MyWindowData &wd, tstring message, Function task, Args... args)
{
    try
    {
        if (!wd.testing && !wd.pWorkerThread)
        {
            wd.testing = true;
            /* Set text of label 'Stat' */
            SendMessage(wd.canvas.hLabTest, WM_SETTEXT, NULL, reinterpret_cast<LPARAM>(message.c_str()));
            wd.pWorkerThread.reset(new boost::thread(task, args...));
        }
    }
    catch (std::exception &e)
    {
        tstring line = Utils::uni(std::string{ e.what() });
        MessageBox(NULL, line.c_str(), _T("Error"), MB_OK | MB_ICONERROR);
        //InvalidateRect(hWnd, &myRect, FALSE);
    }
}
void  WorkerThreadTryJoin (MyWindowData &wd);
//-------------------------------------------------------------------------------
void  onPaintStaticBckGrnd (HDC hdc, MyWindowData &wd);
void  onPaintStaticFigures (HDC hdc, MyWindowData &wd);
void  onPainDynamicFigures (HDC hdc, MyWindowData &wd);
//-------------------------------------------------------------------------------
void  onWindowTimer (MyWindowData &wd);
void  onWindowMouse (MyWindowData &wd);
//-------------------------------------------------------------------------------
void  onShowDBPoints (MyWindowData &wd);
void  onShowDBTrajes (MyWindowData &wd);
//------------------------------------------------------------------------------
tstring getJointsHelp (const Robo::RoboI& robo);
//------------------------------------------------------------------------------
#endif // _WINDOW_DATA_H_