#include "StdAfx.h"
#include "RoboPos.h"


//------------------------------------------
RoboPos::NewHand::MainDirections RoboPos::NewHand::MainDirectionsFactory(IN Robo::RoboI &robo)
{
    Robo::Trajectory trajectory;

    robo.reset();
    auto oldRarity = robo.getVisitedRarity();
    robo.setVisitedRarity(1);

    RoboPos::NewHand::MainDirections MDs(robo);
    for (Robo::joint_t j = 0; j < robo.jointsCount(); ++j)
    {
        Robo::muscle_t muscle;

        robo.setJoints({ {j, 100.} });
        muscle = Robo::RoboI::muscleByJoint(j, true);
        robo.move(Robo::Control({ muscle, 0, robo.muscleMaxLast(muscle) }), trajectory);
        MDs.directions.push_back({ muscle, trajectory, robo.jointPos(j) });

        robo.setJoints({ { j, 0. } });
        muscle = Robo::RoboI::muscleByJoint(j, false);
        robo.move(Robo::Control({ muscle, 0, robo.muscleMaxLast(muscle) }), trajectory);
        MDs.directions.push_back({ muscle, trajectory, robo.jointPos(j) });

    }
    robo.setVisitedRarity(oldRarity);
    robo.reset();
    return MDs;
}
//------------------------------------------
RoboPos::NewHand::MainDirections::Direction::Direction(Robo::muscle_t m, Robo::Trajectory &trajectory, Point c) :
    muscle(m), center(c)
{
    //---------------------------------
    Point &front = trajectory.front();
    max_radius = boost_distance(center, front);
    min_radius = boost_distance(center, front);
    //---------------------------------
    shifts.reserve(trajectory.size());
    for (auto pt : trajectory)
    {
        double radius = boost_distance(center, pt);
        //---------------------------------
        if (radius > max_radius) max_radius = radius;
        if (radius < min_radius) min_radius = radius;
        //---------------------------------
        shifts.push_back(Point(pt.x - front.x, pt.y - front.y));
    }
}

//------------------------------------------
Point RoboPos::NewHand::MainDirections::predict(const Robo::Control &control)
{
    Point res;
    for (Robo::muscle_t m = 0; m < owner.musclesCount(); ++m)
    {
        const Direction &d = directions[m & control[0].muscle];
        if (control[0].last < d.shifts.size())
        {
            res.x += d.shifts[control[0].last].x;
            res.y += d.shifts[control[0].last].y;
        }
    }
    return res;
}
//------------------------------------------
bool  RoboPos::NewHand::MainDirections::shifting_gt(Robo::muscle_t m, unsigned int &inx, const Point &aim)
{
    bool changes = false;
    Point  prev, curr;

    auto &shifts = directions[m].shifts;

    curr.x = base.x + shifts[inx].x;
    curr.y = base.y + shifts[inx].y;
    do
    {
        prev = curr;

        curr.x = base.x + shifts[inx + 1].x;
        curr.y = base.y + shifts[inx + 1].y;

        ++inx;
    } while ((inx < (shifts.size() - 1U))
             && (boost_distance(aim, curr) < boost_distance(aim, prev))
             && (changes = true));
    return changes;
}
bool  RoboPos::NewHand::MainDirections::shifting_ls(Robo::muscle_t m, unsigned int &inx, const Point &aim)
{
    bool changes = false;
    Point  prev, curr;

    auto &shifts = directions[m].shifts;

    curr.x = base.x + shifts[inx].x;
    curr.y = base.y + shifts[inx].y;
    do
    {
        prev = curr;

        curr.x = base.x + shifts[inx - 1].x;
        curr.y = base.y + shifts[inx - 1].y;

        --inx;
    } while ((inx > 0)
             && (boost_distance(aim, curr) > boost_distance(aim, prev))
             && (changes = true));
    return changes;
}
//------------------------------------------
Robo::Control  RoboPos::NewHand::MainDirections::measure(const Point &aim)
{  /*  We have <musclesCount> rulers. They must be matched 
    *  that way to approximate the aim.
    *  System of Linear Equations
    */
    Robo::Control controls;
    std::vector<int> indices;

    bool changes = true;
    while (changes)
    {
        for (Robo::joint_t j = 0; j < owner.jointsCount(); ++j)
        {
            auto mo = Robo::RoboI::muscleByJoint(j, true);
            auto mc = Robo::RoboI::muscleByJoint(j, false);

            changes = false;
            // wcout << robo.joints_[ij] << endl;

            if (indices[j] >= 0)
            {
                unsigned int inx = indices[j];

                changes = shifting_gt(mo, inx, aim);
                if (!changes)
                    changes = shifting_ls(mo, inx, aim);

                indices[j] = inx;
            }

            if (indices[j] <= 0)
            {
                unsigned int inx = -indices[j];

                changes = shifting_gt(mo, inx, aim);
                if (!changes)
                    changes = shifting_ls(mo, inx, aim);

                indices[j] = -(int)inx;
            } // end if
        } // end for
    } // end while

    for (auto j = Robo::joint_t(owner.jointsCount()); j > 0U; --j)
    {
        auto mo = Robo::RoboI::muscleByJoint(j - 1, true);
        auto mc = Robo::RoboI::muscleByJoint(j - 1, false);

        if (indices[j - 1] > 0)
            controls.append({ mo, 0U, Robo::frames_t(indices[j - 1]) }); // stopping?
        else if (indices[j - 1] < 0)
            controls.append({ mc, 0U, Robo::frames_t(-indices[j - 1]) }); // stopping?
    }
    return controls;
}
//------------------------------------------

