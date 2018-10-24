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
class TargetI;
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
    enum class Zoom : int8_t { NONE = 0, STATIC, WHEEL };
    static Zoom zoom;
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
        std::vector<Point/*Robo::Learn::State*/> uncoveredPointsList{};
        bool uncoveredPointsShow{ true };
        // --------------------------------
    } canvas;

    /// Show frames trajectory
    class TrajectoryFrames
    {
        bool show_ = false;
        bool animation_ = true;
        size_t skip_show_frames_ = 15U;
        size_t controls_curr_ = 0;
        Robo::Control controls_{};
        Point base_pos_{};

    public:
        size_t skipShowFrames() const { return skip_show_frames_; }
        bool show() const { return show_; }
        bool animation() const { return animation_; }
        void setAnim(bool animation) { animation_ = animation; }
        void setSkipShowFrames(size_t ssf) { skip_show_frames_ = ssf; }
        void clear();
        void step(RoboMoves::Store &store, Robo::RoboI &robo, const Robo::Control &controls);
        void step(RoboMoves::Store &store, Robo::RoboI &robo);
    };
    TrajectoryFrames trajFrames;
    //Robo::frames_t frames = 0; ///< Global App Time (to show animation)
    // ---------------------------------
    bool testing = false;
    std::shared_ptr<boost::thread> pWorkerThread;
    // ---------------------------------
    std::shared_ptr<TargetI> pTarget;
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
    void save(const tstring &fn_db) const;
    void load(const tstring &fn_db);

    static std::list<Point> goals;
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