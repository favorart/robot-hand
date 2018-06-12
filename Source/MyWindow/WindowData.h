#include "StdAfx.h"

#ifndef  _WINDOW_DATA_H_
#define  _WINDOW_DATA_H_

#include "WindowHeader.h"
#include "Robo.h"
//-------------------------------------------------------------------------------
namespace RoboMoves {
class Record;
class Store;
}
namespace RoboPos {
class LearnMoves;
}
class RecTarget;
class CanvasScaleLetters;

//-------------------------------------------------------------------------------
struct MyWindowData
{
    struct MouseHandler
    {
        bool   click{ false };
        POINT coords{}; ///< координаты мыши в пикселях
        Point    aim{};
    } mouse;
    // ---------------------------------
    static bool zoom;
    struct Canvas
    {
        HWND    hLabCanvas, hLabHelp, hLabMAim, hLabTest, hLabStat;
        HPEN    hPen_red, hPen_grn, hPen_blue, hPen_cian, hPen_orng;
        HBRUSH  hBrush_white, hBrush_null, hBrush_back;
        // ---------------------------------
        RECT    myRect;
        // ---------------------------------
        HDC     hStaticDC{ NULL };
        HBITMAP hStaticBitmap{ NULL };
        // ---------------------------------
        bool    hStaticBitmapChanged{ true };
        bool    hDynamicBitmapChanged{ true };
        // ---------------------------------
        LabelsPositions lp;
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

    /// Show frames trajectory
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
    Robo::frames_t frames = 0; ///< Global App Time (to show animation)
    // ---------------------------------
    bool testing = false;
    std::shared_ptr<boost::thread> pWorkerThread;
    // ---------------------------------
    std::shared_ptr<RecTarget> pTarget;
    std::shared_ptr<Robo::RoboI> pRobo;
    std::shared_ptr<RoboMoves::Store> pStore;
    std::shared_ptr<RoboPos::LearnMoves> pLM;
    // ---------------------------------
    struct StoreSearch
    {
        const double radius = 0.05;
        const double side = 0.1;
    };
    const StoreSearch search;
    // ---------------------------------
    tstring currFileName{};
    tstring getCurrFileName() const
    {
        tstringstream ss;
        ss << pRobo->name() << _T("-robo-moves-")
           << getCurrentTimeString(_T("%Y.%m.%d-%H.%M"))
           << _T(".bin");
        // ?? currFileName = ss.str();
        return ss.str();
    }
    // ---------------------------------
    void MyWindowData::save(const tstring &fn_db) const;
    void MyWindowData::load(const tstring &fn_db);

    static std::list<Point> predicts;
    static std::list<Point> reals;
    // ---------------------------------
    MyWindowData(const tstring &config, const tstring &database);
    ~MyWindowData();
    // ---------------------------------
    void read_config(IN const tstring &filename);
    void write_config(IN const tstring &filename) const;
};
//-------------------------------------------------------------------------------
template<typename Function, typename... Args>
void  WorkerThreadRunTask (MyWindowData &wd, const tstring &message, Function task, Args... args)
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
    catch (const std::exception &e)
    {
        CERROR(e.what());
        //InvalidateRect(hWnd, &myRect, FALSE);
    }
}
bool  WorkerThreadTryJoin (MyWindowData &wd);
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
//-------------------------------------------------------------------------------
bool  repeatRoboMove(MyWindowData &wd);
bool  makeRoboMove(MyWindowData &wd);
//-------------------------------------------------------------------------------
tstring getJointsHelp (const Robo::RoboI& robo);
//-------------------------------------------------------------------------------
#endif // _WINDOW_DATA_H_