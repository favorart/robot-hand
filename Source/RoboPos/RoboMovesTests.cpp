#include "StdAfx.h"
#include "RoboLearnMoves.h"
#include "RoboPhysics.h"
#include "RoboPosApprox.h"

using namespace Robo;
using namespace RoboPos;
using namespace RoboMoves;
//------------------------------------------------------------------------------
void init_thread_rand_seed()
{
    std::srand(112); //(unsigned int)clock()); //
}

//------------------------------------------------------------------------------
void RoboPos::testRandom(Store &store, RoboI &robo, size_t tries)
{
    init_thread_rand_seed();
    //-------------------------------
    try
    {
        robo.reset();
        Point pos_base = robo.position();
        //-------------------------------
        for (size_t i = 0; i < tries; ++i)
        {
            Control controls;
            controls.fillRandom(robo.musclesCount(), [&robo](muscle_t m) {
                return (robo.muscleMaxLasts(m)/* / 2*/);
            }, 50, 1, 8, true /*simil*/);
            //-------------------------------
            if (store.exactRecordByControl(controls))
            {
                if (i) --i;
                continue;
            }
            robo.move(controls);
            const Point& pos = robo.position();
            //-------------------------------
            store.insert(Record{ pos, pos_base, pos, controls, robo.trajectory() });
            // ==============================
            boost::this_thread::interruption_point();
            // ==============================
            robo.reset();
        }
    }
    catch (boost::thread_interrupted&)
    { CINFO("WorkingThread interrupted"); }
    catch (const std::exception &e)
    { SHOW_CERROR(e.what()); }
}

//------------------------------------------------------------------------------
void RoboPos::testCover(Store &store, RoboPhysics &robo)
{
    init_thread_rand_seed();
    //-------------------------------
    const frames_t lasts_min = 50U;
    const frames_t lasts_step = 10U;

    try
    {
        robo.reset();
        Point base = robo.position();

        /* Create the tree of possible passes */
        for (muscle_t muscle_i = 0; muscle_i < robo.musclesCount(); ++muscle_i)
        {
            for (frames_t last_i = lasts_min; last_i < robo.muscleMaxLasts(muscle_i); last_i += lasts_step)
            {
                robo.move(Control{ { muscle_i, 0, last_i } }, LastsInfinity);
                store.insert(Record(robo.position(), base, robo.position(),
                                    { muscle_i }, { 0 }, { last_i }, 1U, robo.trajectory()));

                //=================================================
                if (robo.jointsCount() < 2U)
                    continue;
                //=================================================
                for (muscle_t muscle_j = 0; muscle_j < robo.musclesCount(); ++muscle_j)
                {
                    if ((muscle_i == muscle_j) /*|| !robo.musclesValidUnion(muscle_i, muscle_j)*/)
                        continue;

                    for (frames_t last_j = lasts_min; last_j < robo.muscleMaxLasts(muscle_j); last_j += lasts_step)
                    {
                        auto tail_j = robo.traj().end();
                        --tail_j;

                        robo.move(Control{ { muscle_j, 0, last_j } }, LastsInfinity);
                        store.insert(Record(robo.position(), base, robo.position(),
                                            { muscle_i, muscle_j }, { 0, last_i }, { last_i, last_j },  2U, robo.trajectory()));

                        ++tail_j;
                        //=================================================
                        if (robo.jointsCount() < 3U)
                            continue;
                        //=================================================
                        for (muscle_t muscle_k = 1; muscle_k < robo.musclesCount(); ++muscle_k)
                        {
                            if ((muscle_i == muscle_k || muscle_k == muscle_j) /*|| !musclesValidUnion(muscle_i | muscle_j | muscle_k) */)
                                continue;

                            for (frames_t last_k = lasts_min; last_k < robo.muscleMaxLasts(muscle_k); last_k += lasts_step)
                            {
                                auto tail_k = robo.traj().end();
                                --tail_k;

                                robo.move(Control{ { muscle_k, 0, last_j } }, LastsInfinity);
                                store.insert(Record(robo.position(), base, robo.position(),
                                                    /* muscle */ { muscle_i, muscle_j, muscle_k },
                                                    /* start  */ { 0, last_i, last_i + last_j },
                                                    /* lasts  */ { last_i, last_j, last_k },
                                                    3U, robo.trajectory()));

                                ++tail_k;

                                robo.traj().erase(tail_k, robo.traj().end());
                                robo.resetJoint(RoboI::jointByMuscle(muscle_k));
                                boost::this_thread::interruption_point();
                            }
                        }
                        //=================================================
                        robo.traj().erase(tail_j, robo.traj().end());
                        robo.resetJoint(RoboI::jointByMuscle(muscle_j));
                    }
                }
                robo.traj().clear();
                robo.resetJoint(RoboI::jointByMuscle(muscle_i));
            }
        }
        robo.reset();
    }
    catch (boost::thread_interrupted&)
    {
        CINFO("WorkingThread interrupted");
    }
    catch (const std::exception &e)
    {
        SHOW_CERROR(e.what());
    }
}

//------------------------------------------------------------------------------
void RoboPos::testApprox(Store &store, RoboI &robo)
{
    init_thread_rand_seed();
    //-------------------------------
    try
    {
        for (int sizing = 1; sizing < 20; ++sizing)
        {
            CDEBUG(sizing << _T(" approx sizing=") << sizing * 2);

            Approx approx(store.size(), 8, Approx::noize, [sizing]() { return sizing * 2; });
            approx.constructXY(store.begin(), store.end(), store.size());
            //tcout << _T("writing") << std::endl;
            {
                //tfstream ofs("approx.txt", std::ios_base::out);
                //boost::archive::text_oarchive toa(ofs);
                //toa & approx;

                double sum_error = 0.;
                for (auto & rec : store)
                {
                    Point pred = approx.predict(rec.controls);
                    double err = boost_distance(rec.hit, pred);
                    //ofs << pred << " " << rec.hit << " " << err << std::endl;
                    sum_error += err;
                    // ===================================
                    boost::this_thread::interruption_point();
                    // ===================================
                }
                //ofs << std::endl << "sum_error=" << sum_error / store.size() << std::endl;
                //std::cout << std::endl << "sum_error=" << sum_error / store.size() << std::endl;
                CDEBUG(std::endl << "sum_error=" << sum_error / store.size());

                const int iterations = 1000;
                for (auto i = 0; i < iterations; ++i)
                {
                    Control c;
                    c.fillRandom(robo.musclesCount(), [&robo](muscle_t m) { return robo.muscleMaxLasts(m); });

                    robo.reset();
                    robo.move(c);
                    // ===================================
                    boost::this_thread::interruption_point();
                    // ===================================
                    Point pred = approx.predict(c);
                    double err = boost_distance(robo.position(), pred);
                    //ofs << pred << " " << robo.position() << " " << err << std::endl;
                    sum_error += err;
                }
                //ofs << std::endl << "sum_error=" << sum_error / iterations << std::endl;
                //std::cout << std::endl << "sum_error=" << sum_error / iterations << std::endl;
                CDEBUG(std::endl << "sum_error=" << sum_error / iterations);
            }
        }
    }
    catch (boost::thread_interrupted&)
    { CINFO("WorkingThread interrupted"); }
    catch (const std::exception &e)
    { SHOW_CERROR(e.what()); }
}

//------------------------------------------------------------------------------
void RoboPos::testRL(Store &store, RoboI &robo, TargetI &target, Robo::StateTrajectories &show_trajs)
{
    //init_thread_rand_seed(); // used std::random_device
    //-------------------------------
    try
    {
        //----------------------------------------
        RoboRL(store, robo, target, show_trajs);
        //----------------------------------------
    }
    catch (boost::thread_interrupted&)
    { CINFO("WorkingThread interrupted"); }
    catch (const std::exception &e)
    { SHOW_CERROR(e.what()); }
}
