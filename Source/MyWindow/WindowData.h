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
namespace Utils {
struct CArgs;
}

//-------------------------------------------------------------------------------
struct MyWindowData
{
    struct MouseHandler
    {
        bool   click{ false }; ///< был ли клик в этот фрэйм в рабочей области окна
        POINT coords{}; ///< координаты мыши в пикселях
        Point    aim{}; ///< координаты мыши преобразованы во внутреннее представление
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
        //std::list<std::shared_ptr<Robo::Trajectory>> trajsDB{};
        // ---------------------------------
        Robo::StateTrajectories testingTrajsList{};
        bool testingTrajsShow{ false };
        // ---------------------------------
        std::vector<Point/*Robo::Learn::State*/> uncoveredPointsList{};
        bool uncoveredPointsShow{ true };
        // --------------------------------
        bool   centerAxes{ false };
        double uncoveredRadiusZoomed{ 0.005 };
        double uncoveredRadiusNormal{ 0.000 };
        double radiusDBzoom{ 0.003 };
        double radiusDBnorm{ 0.000 };
        double radiusDB{ 0.01 };
        // --------------------------------
        CGradient cGradient{ CGradient::Longz };
        // --------------------------------
    } canvas{};
    // ---------------------------------
    class TrajectoryFrames
    {
        bool show_ = false;
        bool animation_ = true;
        Robo::frames_t skip_show_frames_ = 170;
        Robo::frames_t controls_curr_ = 0;
        Robo::Control controls_{};
        Point base_pos_{};
    public:
        Robo::frames_t skipShowFrames() const { return skip_show_frames_; }
        bool show() const { return show_; }
        bool animation() const { return animation_; }
        void setAnim(bool animation) { animation_ = animation; }
        void setSkipShowFrames(Robo::frames_t ssf) { skip_show_frames_ = ssf; }
        void clear();
        void step(RoboMoves::Store&, Robo::RoboI&, const Robo::Control&);
        void step(RoboMoves::Store&, Robo::RoboI&);
    };
    TrajectoryFrames trajFrames{}; ///< show frames trajectory
    // ---------------------------------
    bool testing = false;
    std::shared_ptr<boost::thread> pWorkerThread{};
    // ---------------------------------
    std::shared_ptr<TargetI> pTarget{};
    std::shared_ptr<Robo::RoboI> pRobo{};
    //std::shared_ptr<Robo::RoboI> pRoboClone{}; ///< For the !weather! testing <<< TODO !!
    std::shared_ptr<RoboMoves::Store> pStore{};
    //Robo::frames_t frames = 0; ///< Global App Time (to show animation)
    // ---------------------------------
    std::shared_ptr<RoboPos::LearnMoves> pLM{};
    std::shared_ptr<Utils::CArgs> pCArgs{};
    // ---------------------------------
    tstring currFileName{};
    tstring storeLoad{ false };
    int storeSaveFormat{ 1 /*TXT*/ };
    bool redirect{ false };
    // ---------------------------------
    struct StoreSearch
    {
        const double radius = 0.05;
        const double side = 0.1;
    };
    const StoreSearch search{};
    // ---------------------------------
#ifdef DEBUG_SHOW_CONTRADICTION
    static std::list<Point> goals;
    static std::list<Point> predicts;
    static std::list<Point> reals;
#endif
    // ---------------------------------
    MyWindowData(const Utils::CArgs&);
    ~MyWindowData();
    // ---------------------------------
    void read_config(IN const tstring &filename);
    void write_config(IN const tstring &filename) const;
    void write_canvas(tptree&) const;
    void read_canvas(tptree&);
    // ---------------------------------
    tstring getCurrFileName() const;
    // ---------------------------------
    void save(const tstring &fn_db) const;
    void load(const tstring &fn_db);
    // ---------------------------------
    void reinitRobo(/*Robo::pRoboI*/);
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
        SHOW_CERROR(e.what());
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
#endif // _WINDOW_DATA_H_