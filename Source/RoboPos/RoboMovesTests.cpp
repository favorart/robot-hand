#include "StdAfx.h"
#include "RoboMovesStore.h"


using namespace Robo;
using namespace RoboMoves;
//------------------------------------------------------------------------------
void  RoboMoves::testRandom (IN OUT Store &store, IN RoboI &robo, IN size_t tries)
{
    try
    {
        /* Для нового потока нужно снова переинициализировать rand */
        std::srand((unsigned int)clock());

        for (size_t i = 0U; i < tries; ++i)
        {
            Control control;
            control.fillRandom(robo.musclesCount(), [&robo](muscle_t m) { return (robo.muscleMaxLast(m) / 2); });
            store.insert(robo, control);
            boost::this_thread::interruption_point();
        } // for tries
        robo.reset();
    }
    catch (boost::thread_interrupted&)
    { /* tcout << _T("WorkingThread interrupted") << std::endl; */ }
}

//------------------------------------------------------------------------------
void  RoboMoves::testCover  (IN OUT Store &store, IN RoboI &robo, IN size_t nesting)
{
    const frames_t lasts_min = 50U;
    const frames_t lasts_step = 10U;

    try
    {
        robo.reset();
        Point base = robo.position();

        /* Create the tree of possible passes */
        for (muscle_t muscle_i = 0; muscle_i < robo.musclesCount(); ++muscle_i)
        {
            for (frames_t last_i = lasts_min; last_i < robo.muscleMaxLast(muscle_i); last_i += lasts_step)
            {
                Trajectory trajectory;
                robo.move(Control{ { muscle_i, 0, last_i } }, trajectory);
                store.insert(Record(robo.position(), base, robo.position(),
                                    { muscle_i }, { 0 }, { last_i }, 1U, trajectory));

                //=================================================
                if (nesting < 2U) { continue; }
                //=================================================
                for (muscle_t muscle_j = 0; muscle_j < robo.musclesCount(); ++muscle_j)
                {
                    if ((muscle_i == muscle_j) /*|| !robo.musclesValidUnion(muscle_i, muscle_j)*/)
                        continue;

                    for (frames_t last_j = lasts_min; last_j < robo.muscleMaxLast(muscle_j); last_j += lasts_step)
                    {
                        Trajectory::iterator tail_j = trajectory.end();
                        --tail_j;

                        robo.move(Control{ { muscle_j, 0, last_j } }, trajectory);
                        store.insert(Record(robo.position(), base, robo.position(),
                                            { muscle_i, muscle_j }, { 0, last_i }, { last_i, last_j },  2U, trajectory));

                        ++tail_j;
                        //=================================================
                        if (nesting < 3U) { continue; }
                        //=================================================
                        for (muscle_t muscle_k = 1; muscle_k < robo.musclesCount(); ++muscle_k)
                        {
                            if ((muscle_i == muscle_k || muscle_k == muscle_j) /*|| !musclesValidUnion(muscle_i | muscle_j | muscle_k) */)
                                continue;

                            for (frames_t last_k = lasts_min; last_k < robo.muscleMaxLast(muscle_k); last_k += lasts_step)
                            {
                                Trajectory::iterator tail_k = trajectory.end();
                                --tail_k;

                                robo.move(Control{ { muscle_k, 0, last_j } }, trajectory);
                                store.insert(Record(robo.position(), base, robo.position(),
                                                    /* muscle */ { muscle_i, muscle_j, muscle_k },
                                                    /* start  */ { 0, last_i, last_i + last_j },
                                                    /* lasts  */ { last_i, last_j, last_k },
                                                    3U, trajectory));

                                ++tail_k;

                                trajectory.erase(tail_k, trajectory.end());
                                robo.resetJoint(muscle_k);
                                boost::this_thread::interruption_point();
                            }
                        }
                        //=================================================
                        trajectory.erase(tail_j, trajectory.end());
                        robo.resetJoint(muscle_j);
                        boost::this_thread::interruption_point();
                    }
                }
                robo.resetJoint(muscle_i);
                boost::this_thread::interruption_point();
            }
        }
        robo.reset();
    }
    catch (boost::thread_interrupted&)
    { /* tcout << _T("WorkingThread interrupted" )<< std::endl; */ }
    catch (...)
    { MessageBox(NULL, _T("Error"), _T("Error"), MB_OK); }
}
//------------------------------------------------------------------------------



