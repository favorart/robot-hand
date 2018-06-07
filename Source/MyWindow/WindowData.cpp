#include "WindowData.h"
#include "WindowDraw.h"
#include "WindowDrawLetters.h"

#include "RoboPos.h"
#include "RoboMuscles.h"
#include "RoboLearnMoves.h"
#include "Tank.h"
#include "Hand.h"


using namespace Robo;
using namespace RoboMoves;
using namespace NewHand;
using namespace Mobile;
//-------------------------------------------------------------------------------
bool repeatRoboMove(MyWindowData &wd);
bool   makeRoboMove(MyWindowData &wd);
//-------------------------------------------------------------------------------
void  MyWindowData::TrajectoryFrames::step(Store &store, RoboI &robo, const boost::optional<Control> controls)
{
    if (controls.is_initialized())
    {
        show_ = true;
        if (animation)
        {
            robo.reset();
            base_pos_ = robo.position();
            controls_ = *controls;

            for (size_t i = 0; i < controls_.size() && controls_[i].start == frames_; ++i)
            {
                robo.step(frames_, controls_[i].muscle, controls_[i].lasts);
                control_cur_ = i;
            }
            ++frames_;
        }
        else
        {
            robo.reset();
            Point robo_base = robo.position();
            // -------------------------------------------
            robo.move(*controls, trajectory_);
            // -------------------------------------------
            store.insert(Record{ robo.position(), robo_base, robo.position(), *controls, trajectory });
            // -------------------------------------------
            controls_.clear();
            control_cur_ = 0;
            base_pos_ = {};
            frames_ = 0;
        }
    }
    // ============
    /* Auto-drawing trajectory animation */
    if (show_ && animation_ && frames_)
    {
        for (size_t i = 0; i < skip_show_steps; ++i)
        {
            // ============
            for (size_t i = control_cur_; (i < controls_.size()) && (controls_[i].start == frames_); ++i)
            {
                robo.step(frames_, controls_[i].muscle, controls_[i].lasts);
                control_cur_ = i;
            }
            // ============
            robo.step(frames_);
            // ============
            if (robo.moveEnd())
            {
                const Point &robo_pos = robo.position();
                trajectory_.push_back(robo_pos);
                // -------------------------------------------
                store.insert(Record{ robo_pos, base_pos_, robo_pos, controls_, trajectory_ });
                // -------------------------------------------
                controls_.clear();
                control_cur_ = 0;
                base_pos_ = {};
                frames_ = 0;
                // -------------------------------------------
                break;
            }
            else if (!(frames_ % robo.getVisitedRarity()))
            { trajectory_.push_back(robo.position()); }
            // ============
            ++frames_;
            // ============
        }
    }
}
void  MyWindowData::TrajectoryFrames::clear()
{
    show_ = false;
    trajectory_.clear();
    // -------------------------------------------
    controls_.clear();
    control_cur_ = 0;
    base_pos_ = {};
    frames_ = 0;
}
void  MyWindowData::TrajectoryFrames::draw(HDC hdc, HPEN hPen) const
{ if (show_) { drawTrajectory(hdc, trajectory, hPen); } }
//-------------------------------------------------------------------------------
bool MyWindowData::zoom = false;
//-------------------------------------------------------------------------------
MyWindowData::MyWindowData(const tstring &config, const tstring &database) :
    pWorkerThread{ nullptr },
    pTarget{ nullptr }, //{ std::make_shared<RecTarget>( /* 200U, 200U, */ 18U, 18U, -0.41, 0.43, -0.03, -0.85) },
    pStore{ std::make_shared<RoboMoves::Store>() }
{
    read_config(config);
    currFileName =  currFileName;

    if (database != _T(""))
        currFileName = database;
    else
        currFileName = getCurrFileName();
    
    //Hand::JointInput wrist{ Hand::Joint::Wrist,
    //                        Point{ -0.75, 1.05 },
    //                        100U,
    //                        350U,
    //                        0.,
    //                        MotionLaws::getHandMLaw(MotionLaws::HandMLaw::CONAC),
    //                        true
    //                      };
    //Hand::JointInput elbow{ Hand::Joint::Elbow,
    //                        Point{ -0.70, 1.00 },
    //                        135U,
    //                        550U,
    //                        70.,
    //                        MotionLaws::getHandMLaw(MotionLaws::HandMLaw::CONAC),
    //                        true
    //                      };
    //
    //Hand::JointInput shoulder{ Hand::Joint::Shldr,
    //                           Point{ 0.10, 0.85 },
    //                           105U,
    //                           700U,
    //                           0.,
    //                           MotionLaws::getHandMLaw(MotionLaws::HandMLaw::CONAC),
    //                           true
    //                         };
    //Hand::JointInput clavicle{ Hand::Joint::Clvcl,
    //                           Point{ 0.75, 0.25 },
    //                           40U,
    //                           600U,
    //                           0.,
    //                           MotionLaws::getHandMLaw(MotionLaws::HandMLaw::CONAC),
    //                           true
    //                         };
    //  
    //elbow.frames.stopDistanceRatio = 0.2;
    //shoulder.frames.stopDistanceRatio = 0.2;
    //std::list<Hand::JointInput> inputs = { /*wrist ,*/ elbow, shoulder /*, clavicle*/ };    
    //robo = std::make_shared<Hand>(Point{ 0.75, 0.25 } /* baseClavicle */, inputs);

    //Tank::JointInput l{
    //    Tank::Joint::LTrack,
    //    Point{ 0.45, 0.95 },
    //    25U,
    //    15.,
    //    MotionLaws::getHandMLaw(MotionLaws::HandMLaw::CONAC),
    //    true
    //};
    //Tank::JointInput r{
    //    Tank::Joint::RTrack,
    //    Point{ 0.55, 0.95 },
    //    25U,
    //    15.,
    //    MotionLaws::getHandMLaw(MotionLaws::HandMLaw::CONAC),
    //    true
    //};
    //l.frames.stopDistanceRatio = 0.1;
    //r.frames.stopDistanceRatio = 0.1;
    //robo = std::make_shared<Tank>(Point{ 0.5, 0.95 } /* baseCenter */, std::list<Tank::JointInput>{ l, r });

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
    // if ( lt ) delete lt;
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
    drawCoordinates(hdc, wd.canvas.pLetters->show);
    /// TODO : drawDecardsCoordinates(hdc);
    wd.pTarget->draw(hdc, wd.canvas.hPen_grn,
                     true /* false */,    /* internal lines */
                     true /* false */,   /* internal points */
                     false);       /* ellipse instead pixel */
}
void  onPaintStaticFigures(HDC hdc, MyWindowData &wd)
{
    {
        /// TODO : REMOVE
        //const double CircleRadius = 0.01;
        //
        //for (auto &pred : MyWindowData::predicts)
        //    drawCircle(hdc, pred, CircleRadius, wd.canvas.hPen_orng);
        //
        //for (auto &pred : MyWindowData::reals)
        //    drawCircle(hdc, pred, CircleRadius, wd.canvas.hPen_red);
    }
    // --------------------------------------------------------------
    if (wd.canvas.workingSpaceShow)
        drawTrajectory(hdc, wd.canvas.workingSpaceTraj, wd.canvas.hPen_orng);
    // ----- Отрисовка точек БД -------------------------------------
    if (!wd.testing && wd.canvas.allPointsDBShow && !wd.pStore->empty())
    {
        WorkerThreadRunTask(wd, _T(" *** drawing ***  "),
                            [hdc](Store &store, frames_t robo_max_last,
                                  Trajectory uncoveredPoints, HPEN uncoveredPen) {
            GradPens gradPens(robo_max_last);
            auto getPen = [&gradPens](size_t longs) { return gradPens(longs); };
            store.draw(hdc, (MyWindowData::zoom) ? 0.0005 : 0., getPen);

            for (auto &pt : uncoveredPoints)
                drawCircle(hdc, pt, (MyWindowData::zoom) ? 0.005 : 0., uncoveredPen);
        }, std::ref(*wd.pStore), musclesMaxLasts(*wd.pRobo),
           wd.canvas.uncoveredPointsShow ? wd.canvas.uncoveredPointsList : Trajectory{},
           wd.canvas.hPen_red);
        
    } // end if
}
void  onPainDynamicFigures(HDC hdc, MyWindowData &wd)
{
    const double CircleRadius = 0.01;
    // --------------------------------------------------------------
    /* Target to achive */
    // if ( wd.lt )  wd.lt->draw (hdc, wd);
    if (wd.canvas.hDynamicBitmapChanged && !wd.testing)
    {
        // ----- Отрисовка фигуры ---------------------------------------
        wd.pRobo->draw(hdc, wd.canvas.hPen_red, wd.canvas.hBrush_white);

        wd.trajFrames.draw(hdc, wd.canvas.hPen_orng);
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
                drawCircle(hdc, p->hit, CircleRadius);
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

    wd.trajFrames.step(*wd.pStore, *wd.pRobo);
    // =============================
    if (wd.trajFrames.show)
        return;

    for (size_t i = 0; (wd.frames > 0) && (i < wd.pRobo->getVisitedRarity()); ++i)
    {
        wd.pRobo->step(wd.frames);
        ++wd.frames;
        wd.canvas.hDynamicBitmapChanged = true;

        if (wd.pRobo->moveEnd())
        {
            wd.frames = 0;
            break;
        }
    }
}
//-------------------------------------------------------------------------------
bool  repeatRoboMove(MyWindowData &wd)
{
    const Point &aim = wd.mouse.aim;
    // -------------------------------------------------
    auto p = wd.pStore->getClosestPoint(aim, wd.search.side);
    if (!p.first)
        throw std::runtime_error{ "repeatRoboMove: Empty adjacency" };
    // -------------------------------------------------
    if (boost_distance(p.second.hit, aim) <= wd.pTarget->precision())
    {
        /* Repeat Hand Movement */
        wd.pRobo->reset();
        wd.trajFrames.step(*wd.pStore, *wd.pRobo, boost::optional<Control>{p.second.controls});
        // -------------------------------------------------
        if (!wd.trajFrames.animation)
        {
            if (!boost::equal(wd.trajFrames.trajectory, p.second.trajectory))
            { throw std::exception("Incorrect Repeat Robo Move"); }
        }
        // -------------------------------------------------
        tstring text = getWindowTitleString(wd.canvas.hLabMAim);

        tstringstream ss;
        ss << text << _T("\r");
        for (const Control &c : p.second.controls)
        { ss << c << _T("  "); } // \r

        SendMessage(wd.canvas.hLabMAim, WM_SETTEXT, NULL, (LPARAM)ss.str().c_str());
        // -------------------------------------------------
        return true;
    }
    // -------------------------------------------------
    return false;
}
bool  makeRoboMove(MyWindowData &wd)
{
    // wd.adjPointsDB.clear ();
    wd.trajFrames.clear();
    wd.canvas.testingTrajsList.clear();
    // -------------------------------------------------
    if (!repeatRoboMove(wd))
    {
        const Point &aim = wd.mouse.aim;
        wd.pLM->gradientMethod_admixture(aim);
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
//-------------------------------------------------------------------------------
#include <ConfigJSON.h>
void MyWindowData::read_config(IN const tstring &filename)
{
    try
    {
        tptree root;
        tfstream fin(filename, std::ios::in);
        pt::read_json(fin, root);

        tstring robo_name;
        std::list<std::shared_ptr<Robo::JointInput>> robo_joints;
        Point robo_base;
        ConfigJSON::load(root, robo_name, robo_base, robo_joints);

        if (robo_name == _T("Hand-v3"))
        {
            pRobo = std::make_shared<Robo::NewHand::Hand>(robo_base, robo_joints);
        }
        else if (robo_name == _T("Tank-v1"))
        {
            pRobo = std::make_shared<Robo::Mobile::Tank>(robo_base, robo_joints);
        }
        else
        {
            //throw std::logic_error("Not implemented");
            //if (robo_name != robo->name())
            throw std::runtime_error("read_config: incorrect class Robo version");
        }

        ConfigJSON::TargetInput target;
        ConfigJSON::load(root, target);
        pTarget = std::make_shared<RecTarget>(target.vn_aims, target.hn_aims,
                                              target.left, target.right, target.top, target.bottom);
        
        auto &algo = root.get_child(_T("algo"));

        ConfigJSON::TourInput tour;
        ConfigJSON::load(algo, tour);

        ConfigJSON::StageInput1 stage1;
        ConfigJSON::StageInput2 stage2;
        ConfigJSON::StageInput3 stage3;
        ConfigJSON::load(algo, stage1, stage2, stage3);

        double precision = algo.get<double>(_T("precision"));
        if (precision <= 0)
            throw std::runtime_error("read_config: incorrect precision");

        pLM = std::make_shared<RoboPos::LearnMoves>(*pStore, *pRobo, *pTarget, precision, stage1, stage2, stage3);

        unsigned skip_show_steps = root.get<unsigned>(_T("env.skip_show_steps")); /// ???
        bool animation = root.get<bool>(_T("env.animation")); /// ???
    }
    catch (const std::exception &e)
    {
        CERROR(e.what());
        std::exit(1);
    }
}
void MyWindowData::write_config(IN const tstring &filename) const
{
    /// TODO: MyWindowData::write_config ???
    try
    {
        tptree root;
        std::list<std::shared_ptr<Robo::JointInput>> robo_joints; /// ???
        ConfigJSON::save(root, pRobo->name(), pRobo->jointPos(pRobo->jointsCount()), robo_joints);

        ConfigJSON::TargetInput target; /// ???
        ConfigJSON::save(root, target);

        tptree algo;
        {
            ConfigJSON::TourInput tour; /// ???
            ConfigJSON::save(algo, tour);

            ConfigJSON::StageInput1 stage1; /// ???
            ConfigJSON::StageInput2 stage2; /// ???
            ConfigJSON::StageInput3 stage3; /// ???
            ConfigJSON::save(algo, stage1, stage2, stage3);

            double precision; /// ???
            algo.get<double>(_T("precision"), precision);
        }
        root.put_child(_T("algo"), algo);
        
        tptree env;
        {
            bool animation; /// ???
            unsigned skip_show_steps; /// ???

            env.get<unsigned>(_T("skip_show_steps"), skip_show_steps);
            env.get<bool>(_T("animation"), animation);
        }
        root.put_child(_T("env"), env);

        tfstream fout(filename, std::ios::out);
        pt::write_json(fout, root);
    }
    catch (const std::exception &e)
    {
        CERROR(e.what());
    }
}
//-------------------------------------------------------------------------------
