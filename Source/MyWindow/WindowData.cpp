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
#include "Utils.h"

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
        if (!store.exactRecordByControl(controls_))
            store.insert(Record{ robo.position(), base_pos_, robo.position(), controls_, robo.trajectory() });
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
                auto *pRec = store.exactRecordByControl(controls_);
                if (!pRec)
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
    _config(config),
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
    //write_config(filename);
    pStore->dump_off(filename, *pRobo, Store::Format(storeSaveFormat));
}
void MyWindowData::load(const tstring &filename)
{
    read_config(filename);
    pStore->pick_up(filename, pRobo, Store::Format(storeSaveFormat));
}
//-------------------------------------------------------------------------------
RoboMoves::Store::GetHPen makeGrad(CGradient cg, GradPens &gradPens)
{
    std::srand(unsigned(std::time(NULL)));
    switch (cg)
    {
    case CGradient::Longz:
        gradPens.setColors({ RGB(150, 10, 245), RGB(245, 10, 150) }, 25);
        return [&gradPens](const Record &rec) { return gradPens(/*longs*/rec.longestMusclesControl()); };
    case CGradient::Dense:
        gradPens.setColors({ RGB(0,0,255), RGB(255,0,0) }, 35);
        gradPens.shuffleGradient();
        return [&gradPens](const Record &rec) { return gradPens(rec.controlsDense()); };
    case CGradient::Strats:
    {
        size_t n_controls = 20;
        gradPens.setColors({ RGB(0,255,0), RGB(255,0,0) }, RoboI::musclesMaxCount * n_controls);
        gradPens.shuffleGradient();
        return [&gradPens](const Record &rec) {
            auto n_strat = RoboMoves::getStrategy(rec.controls);
            return gradPens(frames_t(n_strat));
        };
    }
    //case CGradient::None:
    default: throw std::logic_error("Invalid CGradient");
    }
}
//-------------------------------------------------------------------------------
void  onPaintStaticBckGrnd(HDC hdc, MyWindowData &wd)
{
    if (wd.canvas.centerAxes)
        drawDecardsCoordinates(hdc);
    else
        drawCoordinates(hdc, wd.canvas.pLetters->show);
    wd.pTarget->draw(hdc, wd.canvas.hPen_grn);
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
        //robot_max_lasts = (Robo::LastsInfinity == robot_max_lasts) ? Robo::LastsInfinity : (robot_max_lasts * wd.pRobo->musclesCount());

        auto sR = (MyWindowData::zoom == MyWindowData::Zoom::STATIC) ? wd.canvas.radiusDBzoom : wd.canvas.radiusDBnorm;
        auto uR = (MyWindowData::zoom == MyWindowData::Zoom::STATIC) ? wd.canvas.uncoveredRadiusZoomed : wd.canvas.uncoveredRadiusNormal;

        WorkerThreadRunTask(wd, _T(" *** drawing ***  "),
                            [hdc](const Store &store, CGradient cGrad, frames_t maxLasts, double sR,
                                  const Trajectory &uncoveredPoints, HPEN uncoveredPen, double uR) {
            try
            {
                GradPens gradPens(maxLasts);
                Store::GetHPen g = makeGrad(cGrad, gradPens);
                store.draw(hdc, sR, g);

                for (auto &pt : uncoveredPoints)
                    drawCircle(hdc, pt, uR, uncoveredPen);
            }
            catch (const std::exception &e)
            {
                SHOW_CERROR(e.what());
            }
        }, std::ref(*wd.pStore), wd.canvas.cGradient, robot_max_lasts, sR,
           (wd.canvas.uncoveredPointsShow ? wd.canvas.uncoveredPointsList : Trajectory{}),
           wd.canvas.hPen_red, uR);
    } // end if
}
void  onPainDynamicFigures(HDC hdc, MyWindowData &wd)
{
    if (wd.canvas.hDynamicBitmapChanged && !wd.testing)
    {
        // ----- Отрисовка фигуры ---------------------------------------
        wd.pRobo->draw(hdc, wd.canvas.hPen_red, wd.canvas.hBrush_white);

        if (wd.trajFrames.show())
            drawStateTrajectory(hdc, wd.pRobo->trajectory(), wd.canvas.hPen_orng);
        // --------------------------------------------------------------
        if (wd.canvas.testingTrajsShow && !wd.canvas.testingTrajsList.empty())
        {
            for (auto &t : wd.canvas.testingTrajsList)
                drawStateTrajectory(hdc, t, wd.canvas.hPen_blue);
        }
        // ----- Отрисовка точек БД -------------------------------------
        if (!wd.canvas.pointsDB.empty())
        {
            HPEN hPen_old = (HPEN)SelectObject(hdc, wd.canvas.hPen_cian);
            for (auto &p : wd.canvas.pointsDB)
                drawCircle(hdc, p->hit, wd.canvas.radiusDB);
            SelectObject(hdc, hPen_old);
        }
        // --------------------------------------------------------------
        if (wd.canvas.pLetters->show)
            wd.canvas.pLetters->draw(hdc, &wd.pRobo->jointPos(0)/*curPos.begin()*/, wd.pRobo->jointsCount()+1, wd.canvas.centerAxes);
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
    //wd.canvas.testingTrajsList.clear ();
    wd.pStore->adjacencyPoints (wd.canvas.pointsDB, wd.mouse.aim, wd.search.radius);
  }
}
void  onShowDBTrajes(MyWindowData &wd)
{
    wd.trajFrames.clear();
    //for (auto &rec : wd.canvas.pointsDB)
    //{ wd.canvas.trajsDB.push_back(std::make_shared<StateTrajectory>(rec->trajectory)); }
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
        for(auto i = 0u; i < 1/*wd.pRobo->getVisitedRarity()*/; ++i)
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
    //MyWindowData::reals.clear();
    //br::copy(p.second.trajectory, std::back_inserter(MyWindowData::reals));
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
    //wd.canvas.testingTrajsList.clear();
    // -------------------------------------------------
    if (!repeatRoboMove(wd))
    {
        const Point &aim = wd.mouse.aim;
        // if (!wd.pLM) wd.pLM = std::make_shared<RoboPos::LearnMoves>(*wd.pStore, *wd.pRobo, *wd.pTarget, wd._lm_config); //??
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

Factory<RoboI> frobo;
Factory<RoboI>::MakeMethods Factory<RoboI>::makes{ Robo::NewHand::Hand::make, Robo::Mobile::Tank::make };

Factory<TargetI> ftarget;
Factory<TargetI>::MakeMethods Factory<TargetI>::makes{ RecTarget::make, PolyTarget::make };
//-------------------------------------------------------------------------------
// http://zenol.fr/blog/boost-property-tree/en.html
void MyWindowData::read_config(IN const tstring &filename)
{
    try
    {
        tptree root;
        tfstream fin = Utils::utf8_stream(filename, std::ios::in);
        if (!fin.is_open())
            throw std::runtime_error("read_config: file is not exist");
        pt::read_json(fin, root);
        
        pRobo = Factory<RoboI>::create(root);
        pTarget = Factory<TargetI>::create(root);

        if (!pRobo) throw std::runtime_error("read_config: incorrect class RoboI version");
        if (!pTarget) throw std::runtime_error("read_config: incorrect class TargetI version");

        GET_OPT(root, storeSaveFormat);
        GET_OPT(root, redirect);
        if (redirect)
        {
            auto stdout_filename = ("out-" + Utils::now() + ".txt");
            if (std::freopen(stdout_filename.c_str(), "w", stdout) == NULL)
                throw std::runtime_error("read_config: invalid IO stream redirect freopen");
        }

        storeLoad = root.get_optional<tstring>(_T("startUpLoad")).get_value_or(_T(""));
        if (storeLoad.length())
        {
            pStore->pick_up(storeLoad, pRobo, Store::Format(storeSaveFormat));
        }
        // -------------------------------------------------------------------------

        double precision_mm = root.get_optional<double>(_T("precision")).get_value_or(1.5);
        if (precision_mm <= 0)
            throw std::runtime_error("read_config: incorrect precision");

        _lm_config = root.get<tstring>(_T("lm_config"));
        pLM = std::make_shared<RoboPos::LearnMoves>(*pStore, *pRobo, *pTarget, _lm_config); // ????
        read_canvas(root);

        LV_CLEVEL = root.get_optional<int>(_T("verbose")).get_value_or(1);
    }
    catch (const std::exception &e)
    {
        SHOW_CERROR(e.what());
        std::exit(1);
    }
}
void MyWindowData::read_canvas(tptree &root)
{
    auto node = root.get_child(_T("canvas"));
    canvas.radiusDB = node.get_optional<double>(_T("radiusDB")).get_value_or(0.01);
    canvas.radiusDBzoom = node.get_optional<double>(_T("radiusDBzoom")).get_value_or(0.000);
    canvas.radiusDBnorm = node.get_optional<double>(_T("radiusDBnorm")).get_value_or(0.003);
    canvas.uncoveredRadiusZoomed = node.get_optional<double>(_T("uncoveredRzoomed")).get_value_or(0.005);
    canvas.uncoveredRadiusNormal = node.get_optional<double>(_T("uncoveredRnormal")).get_value_or(0.000);

    canvas.centerAxes = node.get_optional<bool>(_T("centerAxes")).get_value_or(true);

    trajFrames.setAnim(node.get_optional<bool>(_T("animation")).get_value_or(true));
    trajFrames.setSkipShowFrames(node.get_optional<frames_t>(_T("skipShowFrames")).get_value_or(15));
}
void MyWindowData::write_canvas(tptree &root) const
{
    tptree node;
    node.put(_T("radiusDB"), canvas.radiusDB);
    node.put(_T("radiusDBzoom"), canvas.radiusDBzoom);
    node.put(_T("radiusDBnorm"), canvas.radiusDBnorm);
    node.put(_T("uncoveredRzoomed"), canvas.uncoveredRadiusZoomed);
    node.put(_T("uncoveredRnormal"), canvas.uncoveredRadiusNormal);
    node.put(_T("centerAxes"), canvas.centerAxes);
    node.put(_T("animation"), trajFrames.animation());
    node.put(_T("skipShowFrames"), trajFrames.skipShowFrames());
    root.put_child(_T("canvas"), node);
}
#include "RoboPosTour.h"
void MyWindowData::write_config(IN const tstring &filename) const
{
    try
    {
        tptree root;
        pRobo->save(root);
        pTarget->save(root);

        write_canvas(root);

        double precision_mm = pTarget->precision() / RoboPos::TourI::divToMiliMeters;
        root.put(_T("precision"), precision_mm);
        root.put(_T("lm_config"), _lm_config);
        
        root.put(_T("verbose"), LV_CLEVEL);
        root.put(_T("redirect"), redirect);
        root.put(_T("saveFormat"), storeSaveFormat);
        root.put(_T("#saveFormat"), _T("none(0)|txt(1)|bin(2)"));
        root.put(_T("startUpLoad"), storeLoad/*_T("Hand-v4-robo-moves-2018.10.23-16.55.bin")*/);

        tfstream fout = Utils::utf8_stream(filename, std::ios::out);
        if (!fout.is_open())
            throw std::runtime_error("write_config: file is not opened");
        fout.precision(3);
        fout.setf(std::ios::fixed);// | std::ios::showpoint | std::ios::floatfield);  // useless
        pt::write_json(fout, root);
    }
    catch (const std::exception &e)
    {
        SHOW_CERROR(e.what());
    }
}
//-------------------------------------------------------------------------------
