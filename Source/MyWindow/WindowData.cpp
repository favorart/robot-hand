#include "WindowData.h"
#include "WindowDraw.h"
#include "WindowDrawLetters.h"

#include "Robo.h"
#include "RoboMovesTarget.h"
#include "RoboMovesStore.h"
#include "RoboLearnMoves.h"
#include "RoboMuscles.h"
#include "Tank.h"
#include "Hand.h"

using namespace Robo;
using namespace RoboMoves;
//-------------------------------------------------------------------------------
bool repeatRoboMove(MyWindowData &wd);
bool   makeRoboMove(MyWindowData &wd);
//-------------------------------------------------------------------------------
void MyWindowData::TrajectoryFrames::step(Store &store, RoboI &robo, const Control &controls)
{
    show_ = true;
    // ------------
    robo.reset();
    base_pos_ = robo.position();
    // ------------
    if (animation_)
    {
        controls_curr_ = 0;
        controls_ = controls;
        // ============
        robo.step(controls_, controls_curr_);
        // ============
    }
    else
    {
        // ============
        robo.move(controls);
        // ============
        store.insert(Record{ robo.position(), base_pos_, robo.position(), controls, robo.trajectory() });
        // ------------
        controls_.clear();
        controls_curr_ = 0;
        base_pos_ = {};
    }
    // ------------
    step(store, robo);
}
void MyWindowData::TrajectoryFrames::step(Store &store, RoboI &robo)
{
    /* Auto-drawing trajectory animation */
    if (show_ && animation_ && controls_curr_ && controls_.size())
    {
        for (size_t i = 0; i < skip_show_frames_; ++i)
        {
            // ============
            robo.step(controls_, controls_curr_);
            // ============
            if (robo.moveEnd())
            {
                // ------------
                store.insert(Record{ robo.position(), base_pos_, robo.position(), controls_, robo.trajectory() });
                // ------------
                controls_.clear();
                controls_curr_ = 0;
                base_pos_ = {};
                // ------------
                break;
            }
        }
    }
}
void MyWindowData::TrajectoryFrames::clear()
{
    show_ = false;
    // ------------
    controls_.clear();
    controls_curr_ = 0;
    base_pos_ = {};
}
//-------------------------------------------------------------------------------
MyWindowData::Zoom MyWindowData::zoom = MyWindowData::Zoom::NONE;
//-------------------------------------------------------------------------------
MyWindowData::MyWindowData(const tstring &config, const tstring &database) :
    pWorkerThread{ nullptr },
    pTarget{ nullptr },
    pStore{ std::make_shared<RoboMoves::Store>() }
{
    read_config(config);
    currFileName =  currFileName;

    if (database != _T(""))
        currFileName = database;
    else
        currFileName = getCurrFileName();

    /* создаем ручки */
    canvas.hPen_grn  = CreatePen(PS_SOLID, 1, RGB(100, 180, 050));
    canvas.hPen_red  = CreatePen(PS_SOLID, 2, RGB(255, 000, 000));
    canvas.hPen_blue = CreatePen(PS_SOLID, 2, RGB(000, 000, 255));
    canvas.hPen_cian = CreatePen(PS_SOLID, 1, RGB(057, 168, 157));
    canvas.hPen_orng = CreatePen(PS_SOLID, 2, RGB(255, 128, 000));

    /* создаем кисти */
    canvas.hBrush_white = (HBRUSH)GetStockObject(WHITE_BRUSH);
    canvas.hBrush_null = (HBRUSH)GetStockObject(NULL_BRUSH);
    canvas.hBrush_back = CreateSolidBrush(RGB(235, 235, 255));
    /* background color = RGB (255,204,238) */
    canvas.pLetters = std::make_shared<CanvasScaleLetters>(pTarget->min(), pTarget->max());
}
MyWindowData::~MyWindowData()
{
    //=======================
    /* очищаем ручку */
    DeleteObject(canvas.hPen_grn);
    DeleteObject(canvas.hPen_red);
    DeleteObject(canvas.hPen_blue);
    DeleteObject(canvas.hPen_cian);
    DeleteObject(canvas.hPen_orng);

    DeleteObject(canvas.hBrush_white);
    DeleteObject(canvas.hBrush_null);
    DeleteObject(canvas.hBrush_back);

    DeleteDC(canvas.hStaticDC);
    DeleteObject(canvas.hStaticBitmap);
    //=======================    
    if (pWorkerThread)
    {
        pWorkerThread->interrupt();
        pWorkerThread.reset();
    }
    //=======================
    // save(currFileName);
    //=======================
}
//-------------------------------------------------------------------------------
void MyWindowData::save(const tstring &filename) const
{
    write_config(filename);
    pStore->dump_off(filename);
}
void MyWindowData::load(const tstring &filename)
{
    read_config(filename);
    pStore->pick_up(filename);
}
//-------------------------------------------------------------------------------
void  onPaintStaticBckGrnd(HDC hdc, MyWindowData &wd)
{
    if (wd.canvas.centerAxes)
        drawDecardsCoordinates(hdc);
    else
        drawCoordinates(hdc, wd.canvas.pLetters->show);
    wd.pTarget->draw(hdc, wd.canvas.hPen_grn,
                     wd.canvas.targetLines,
                     wd.canvas.targetPoints,
                     wd.canvas.targetRadius);
}
void  onPaintStaticFigures(HDC hdc, MyWindowData &wd)
{
    {
        /// TODO : REMOVE
        const double CircleRadius = 0.004;

        for (auto &pred : MyWindowData::goals)
            drawCross(hdc, pred, CircleRadius, wd.canvas.hPen_grn);        
        
        for (auto &pred : MyWindowData::predicts)
            drawCircle(hdc, pred, CircleRadius, wd.canvas.hPen_orng);
        
        for (auto &pred : MyWindowData::reals)
            drawCircle(hdc, pred, CircleRadius, wd.canvas.hPen_red);
    }
    // --------------------------------------------------------------
    if (wd.canvas.workingSpaceShow)
        drawTrajectory(hdc, wd.canvas.workingSpaceTraj, wd.canvas.hPen_orng);
    // ----- Отрисовка точек БД -------------------------------------
    if (!wd.testing && wd.canvas.allPointsDBShow && !wd.pStore->empty())
    {
        frames_t robot_max_lasts = musclesMaxLasts(*wd.pRobo);
        robot_max_lasts = (Robo::LastsInfinity == robot_max_lasts) ? Robo::LastsInfinity : (robot_max_lasts * wd.pRobo->musclesCount());

        auto sR = (MyWindowData::zoom == MyWindowData::Zoom::STATIC) ? wd.canvas.storeRzoomed : wd.canvas.storeRnormal;
        auto uR = (MyWindowData::zoom == MyWindowData::Zoom::STATIC) ? wd.canvas.uncoveredRzoomed : wd.canvas.uncoveredRnormal;

        WorkerThreadRunTask(wd, _T(" *** drawing ***  "),
                            [hdc](Store &store, frames_t maxLast, Trajectory uncoveredPoints,
                                  HPEN uncoveredPen, double uR, double sR, CGradient cGrad) {

            GradPens gradPens(maxLast); // ????
            auto getPen = [&gradPens](size_t longs) { return gradPens(longs); };
            store.draw(hdc, sR, getPen);
            
            for (auto &pt : uncoveredPoints)
                drawCircle(hdc, pt, uR, uncoveredPen);
        }, std::ref(*wd.pStore), robot_max_lasts,
           wd.canvas.uncoveredPointsShow ? wd.canvas.uncoveredPointsList : Trajectory{},
           wd.canvas.hPen_red, uR, sR, wd.canvas.cGradient);
    } // end if
}
void  onPainDynamicFigures(HDC hdc, MyWindowData &wd)
{
    if (wd.canvas.hDynamicBitmapChanged && !wd.testing)
    {
        // ----- Отрисовка фигуры ---------------------------------------
        wd.pRobo->draw(hdc, wd.canvas.hPen_red, wd.canvas.hBrush_white);

        if (wd.trajFrames.show())
            drawTrajectory(hdc, wd.pRobo->trajectory(), wd.canvas.hPen_orng);
        // --------------------------------------------------------------
        if (wd.canvas.testingTrajsShow && !wd.canvas.testingTrajsList.empty())
        {
            for (auto &t : wd.canvas.testingTrajsList)
                drawTrajectory(hdc, t, wd.canvas.hPen_blue);
        }
        // ----- Отрисовка точек БД -------------------------------------
        if (!wd.canvas.pointsDB.empty())
        {
            HPEN hPen_old = (HPEN)SelectObject(hdc, wd.canvas.hPen_cian);
            for (auto &p : wd.canvas.pointsDB)
                drawCircle(hdc, p->hit, wd.canvas.dbRadius);
            SelectObject(hdc, hPen_old);
        }
        // --------------------------------------------------------------
        if (wd.canvas.pLetters->show)
        {
            std::vector<Point> jPos(wd.pRobo->jointsCount());
            for (joint_t joint = 0; joint < wd.pRobo->jointsCount(); ++joint)
                jPos.push_back(wd.pRobo->jointPos(joint));
            wd.canvas.pLetters->draw(hdc, jPos /*, &wd.pRobo->position()*/);
        }
    }
    // --------------------------------------------------------------
    if (wd.mouse.click)
        drawCircle(hdc, wd.mouse.aim, wd.search.radius, wd.canvas.hPen_cian);
}
//-------------------------------------------------------------------------------
bool  WorkerThreadTryJoin(MyWindowData &wd)
{
    if (wd.pWorkerThread && wd.pWorkerThread->try_join_for(boost::chrono::milliseconds(10)))
    {
        /* joined */
        wd.pWorkerThread.reset();
        wd.pWorkerThread = nullptr;

        wd.testing = false;
        wd.canvas.hDynamicBitmapChanged = true;

        /* Set text of label 'Stat'  */
        SendMessage(wd.canvas.hLabTest, WM_SETTEXT, NULL, reinterpret_cast<LPARAM>(_T(" Done  ")));
        return true;
    }
    return false;
}
//-------------------------------------------------------------------------------
void  onShowDBPoints(MyWindowData &wd)
{
  if ( !wd.testing )
  {
    wd.canvas.pointsDB.clear ();
    wd.canvas.testingTrajsList.clear ();
    wd.pStore->adjacencyPoints (wd.canvas.pointsDB, wd.mouse.aim, wd.search.radius);
  }
}
void  onShowDBTrajes(MyWindowData &wd)
{
    wd.trajFrames.clear();
    for (auto &rec : wd.canvas.pointsDB)
    { wd.canvas.trajsDB.push_back(std::make_shared<Trajectory>(rec->trajectory)); }
}
//-------------------------------------------------------------------------------
void  onWindowTimer(MyWindowData &wd)
{
    if (wd.testing)
        return;
    // =============================
    wd.trajFrames.step(*wd.pStore, *wd.pRobo);
    // =============================
    if (wd.trajFrames.show())
    {
        // анимация тестовой траектории
        wd.canvas.hDynamicBitmapChanged = true;
        return;
    }
    // =============================
    if (wd.pRobo->frame() > 0)
    {
        // анимация ручного управления
        if (!wd.pRobo->moveEnd())
            wd.canvas.hDynamicBitmapChanged = true;        
        for(auto i = 0u; i < wd.pRobo->getVisitedRarity(); ++i)
            wd.pRobo->step();
    }
}
//-------------------------------------------------------------------------------
bool  repeatRoboMove(MyWindowData &wd)
{
    auto p = wd.pStore->getClosestPoint(wd.mouse.aim, wd.search.side);
    if (!p.first)
        throw std::runtime_error("repeatRoboMove: Empty adjacency");
    // -------------------------------------------------
    if (boost_distance(p.second.hit, wd.mouse.aim) <= wd.pTarget->precision())
        return false;
    // -------------------------------------------------
    /* Repeat Hand Movement */
    wd.pRobo->reset();
    wd.trajFrames.step(*wd.pStore, *wd.pRobo, p.second.controls);
    // -------------------------------------------------
    if (!wd.trajFrames.animation())
    {
        if (!boost::equal(wd.pRobo->trajectory(), p.second.trajectory))
            throw std::runtime_error("repeatRoboMove: Restore another trajectory");
    }
    // -------------------------------------------------
    tstringstream ss;
    tstring text = getWindowTitleString(wd.canvas.hLabMAim);
    ss << text << _T("\r");
    ss << p.second.controls << _T("\r");
    SendMessage(wd.canvas.hLabMAim, WM_SETTEXT, NULL, (LPARAM)(ss.str().c_str()));
    // -------------------------------------------------
    return true;
}
bool  makeRoboMove(MyWindowData &wd)
{
    wd.trajFrames.clear();
    wd.canvas.testingTrajsList.clear();
    // -------------------------------------------------
    if (!repeatRoboMove(wd))
    {
        const Point &aim = wd.mouse.aim;
        wd.pLM->testStage3(aim);
        // -------------------------------------------------
        if (0 /* Around Trajectories !!! */)
        {
            // adjacency_refs_t  range;
            // std::pair<Record, Record>  x_pair, y_pair;
            // auto count = wd.pStore->adjacencyByPBorders (aim, 0.06, x_pair, y_pair); // range, aim, min, max);
            // 
            // tcout << count << std::endl;
            // 
            // wd.canvas.testingTrajsList.push_back (x_pair.first.trajectory);
            // wd.canvas.testingTrajsList.push_back (y_pair.first.trajectory);
            // wd.canvas.testingTrajsList.push_back (x_pair.second.trajectory);
            // wd.canvas.testingTrajsList.push_back (y_pair.second.trajectory);
            // return false;
        }
        // -------------------------------------------------
        if (0 /* LinearOperator && SimplexMethod */)
        {
            // HandMoves::controling_t controls;
            // Positions::LinearOperator  lp (wd.pStore, wd.mouse_aim,
            //                                0.07, controls, true);
            // // lp.predict (wd.mouse_aim, controls);
            // 
            // wd.pRobo.SET_DEFAULT;
            // wd.canvas.hand.move (controls.begin (), controls.end (), &wd.trajFrames.trajectory);
        }
        // -------------------------------------------------
        return !repeatRoboMove(wd);
    }
    // -------------------------------------------------
    return true;
}
//-------------------------------------------------------------------------------
using namespace NewHand;
using namespace Mobile;

Factory<TargetI> ftarget;
Factory<RoboI> frobo;
// http://zenol.fr/blog/boost-property-tree/en.html
std::vector<std::function<std::shared_ptr<RoboI>(const tstring&, tptree&)>> Factory<RoboI>::makes{ Robo::NewHand::Hand::make, Robo::Mobile::Tank::make };
std::vector<std::function<std::shared_ptr<TargetI>(const tstring&, tptree&)>> Factory<TargetI>::makes{ RecTarget::make, PolyTarget::make };
//-------------------------------------------------------------------------------
void MyWindowData::read_config(IN const tstring &filename)
{
    try
    {
        tptree root;
        tfstream fin(filename, std::ios::in);
        if (!fin.is_open())
            std::runtime_error("config is not exist");
        pt::read_json(fin, root);

        // -------------------------------------------------------------------------
        bool redirect = root.get_optional<bool>(_T("redirect")).get_value_or(false);
        if (redirect)
        {
            auto stdout_filename = ("out-" + Utils::now() + ".txt");
            if (std::freopen(stdout_filename.c_str(), "w", stdout) == NULL)
                CERROR("freopen: invalid IO stream redirect");
        }
        tstring pickup = root.get_optional<tstring>(_T("pickup")).get_value_or(_T(""));
        if (pickup.length())
        {
            pStore->pick_up(pickup);
        }
        // -------------------------------------------------------------------------

        pRobo = Factory<RoboI>::create(root);
        pTarget = Factory<TargetI>::create(root);
        //throw std::runtime_error("read_config: incorrect class Robo version");

        double precision_mm = root.get_optional<double>(_T("precision")).get_value_or(1.5);
        if (precision_mm <= 0)
            throw std::runtime_error("read_config: incorrect precision");

        tstring fn_config = root.get<tstring>(_T("lm_config"));

        pLM = std::make_shared<RoboPos::LearnMoves>(*pStore, *pRobo, *pTarget, precision_mm, fn_config);

        unsigned skip_show_frames = root.get_optional<unsigned>(_T("env.skipShowFrames")).get_value_or(15);
        trajFrames.setSkipShowFrames(skip_show_frames);

        bool animation = root.get_optional<bool>(_T("env.animation")).get_value_or(true);
        trajFrames.setAnim(animation);

        canvas.centerAxes = root.get_optional<bool>(_T("env.centerAxes")).get_value_or(true);
        canvas.targetLines = root.get_optional<bool>(_T("env.targetLines")).get_value_or(true);
        canvas.targetPoints = root.get_optional<bool>(_T("env.targetPoints")).get_value_or(true);
        canvas.targetRadius = root.get_optional<double>(_T("env.targetRadius")).get_value_or(0.007);

        canvas.uncoveredRzoomed = root.get_optional<double>(_T("env.uncoveredRzoomed")).get_value_or(0.005);
        canvas.uncoveredRnormal = root.get_optional<double>(_T("env.uncoveredRnormal")).get_value_or(0.000);
        canvas.storeRzoomed = root.get_optional<double>(_T("env.storeRzoomed")).get_value_or(0.003);
        canvas.storeRnormal = root.get_optional<double>(_T("env.storeRnormal")).get_value_or(0.000);
        canvas.dbRadius = root.get_optional<double>(_T("env.dbRadius")).get_value_or(0.01);
    }
    catch (const std::exception &e)
    {
        CERROR(e.what());
        std::exit(1);
    }
}
#include "RoboPosTour.h"
void MyWindowData::write_config(IN const tstring &filename) const
{
    try
    {
        tptree root;

        pRobo->save(root);
        pTarget->save(root);

        double precision_mm = pTarget->precision() / RoboPos::TourI::divToMiliMeters;
        root.get<double>(_T("precision"), precision_mm);
        
        tptree env;
        root.put_child(_T("env"), env);

        env.get<unsigned>(_T("skipShowFrames"), unsigned(trajFrames.skipShowFrames()));
        env.get<bool>(_T("animation"), trajFrames.animation());

        tfstream fout(filename, std::ios::out);
        pt::write_json(fout, root);
    }
    catch (const std::exception &e)
    {
        CERROR(e.what());
    }
}
//-------------------------------------------------------------------------------
