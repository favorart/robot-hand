#include "StdAfx.h"
#include "RoboPos.h"


//------------------------------------------
RoboPos::MainDirections RoboPos::MainDirectionsFactory(IN Robo::RoboI &robo)
{
    Robo::Trajectory trajectory;

    robo.reset();
    auto oldRarity = robo.getVisitedRarity();
    robo.setVisitedRarity(1);

    RoboPos::MainDirections md(robo);
    for (Robo::joint_t j = 0; j < robo.jointsCount(); ++j)
    {
        Robo::muscle_t muscle;

        robo.setJoints({ {j, 100.} });
        muscle = Robo::RoboI::muscleByJoint(j, true);
        robo.move(Robo::Control({ muscle, 0, robo.muscleMaxLast(muscle) }), trajectory);
        md._directions.push_back({ muscle, trajectory, robo.jointPos(j) });

        robo.setJoints({ { j, 0. } });
        muscle = Robo::RoboI::muscleByJoint(j, false);
        robo.move(Robo::Control({ muscle, 0, robo.muscleMaxLast(muscle) }), trajectory);
        md._directions.push_back({ muscle, trajectory, robo.jointPos(j) });

    }
    robo.setVisitedRarity(oldRarity);
    robo.reset();
    return md;
}
//------------------------------------------
RoboPos::MainDirections::Direction::Direction(Robo::muscle_t m, Robo::Trajectory &trajectory, Point c) :
    muscle(m) //, center(c)
{
    shifts.resize(trajectory.size());
    std::copy(trajectory.begin(), trajectory.end(), shifts.begin());
    ////---------------------------------
    //Point &front = trajectory.front();
    //max_radius = boost_distance(center, front);
    //min_radius = boost_distance(center, front);
    ////---------------------------------
    //shifts.reserve(trajectory.size());
    //for (auto pt : trajectory)
    //{
    //    double radius = boost_distance(center, pt);
    //    //---------------------------------
    //    if (radius > max_radius) max_radius = radius;
    //    if (radius < min_radius) min_radius = radius;
    //    //---------------------------------
    //    shifts.push_back(Point(pt.x - front.x, pt.y - front.y));
    //}
}

//------------------------------------------
Point RoboPos::MainDirections::predict(Robo::muscle_t muscle, Robo::frames_t last)
{
    //for (Robo::muscle_t m = 0; m < _owner.musclesCount(); ++m)

    if (muscle >= _directions.size())
        throw std::logic_error("Invalid muscle=" + std::to_string(muscle));

    const Direction &d = _directions[muscle];

    if (last >= d.shifts.size())
        throw std::logic_error("last=" + std::to_string(last) + 
                               " >= direction.shifts.size=" + 
                               std::to_string(d.shifts.size()));

    return (d.shifts[last] - d.shifts[0]);
}

//------------------------------------------
Point RoboPos::MainDirections::predict(const Robo::Control &controls)
{
    //if (control.size() != 1)
    //    throw std::logic_error("md.predict: control.size != 1");
    //return predict(control[0].muscle, control[0].last);

    Point res = _base;
    unsigned reached = 0;
    // каждый такт векторно складываем все смещения - получам направление результирующей !!!
    for (Robo::frames_t time = 0; reached == controls.size(); ++time)
    {
        Point frame = { 0., 0. };
        for (auto &c : controls)
        {
            if (c.start >= time && (c.start + c.last) <= time)
            {
                const auto lasts = (time - c.start);
                const Direction &d = _directions[c.muscle];
                frame += (d.shifts[lasts] - (lasts ? d.shifts[lasts-1] : d.shifts[0]));
            }
            else if ((c.start + c.last) > time)
                reached++;
        }
        res += frame;
    }
    return res;
}

//------------------------------------------
bool  RoboPos::MainDirections::shifting_gt(Robo::muscle_t m, unsigned int &inx, const Point &aim)
{
    bool changes = false;
    Point  prev, curr;

    auto &shifts = _directions[m].shifts;

    curr.x = _base.x + shifts[inx].x;
    curr.y = _base.y + shifts[inx].y;
    do
    {
        prev = curr;

        curr.x = _base.x + shifts[inx + 1].x;
        curr.y = _base.y + shifts[inx + 1].y;

        ++inx;
    } while ((inx < (shifts.size() - 1U))
             && (boost_distance(aim, curr) < boost_distance(aim, prev))
             && (changes = true));
    return changes;
}
bool  RoboPos::MainDirections::shifting_ls(Robo::muscle_t m, unsigned int &inx, const Point &aim)
{
    bool changes = false;
    Point  prev, curr;

    auto &shifts = _directions[m].shifts;

    curr.x = _base.x + shifts[inx].x;
    curr.y = _base.y + shifts[inx].y;
    do
    {
        prev = curr;

        curr.x = _base.x + shifts[inx - 1].x;
        curr.y = _base.y + shifts[inx - 1].y;

        --inx;
    } while ((inx > 0)
             && (boost_distance(aim, curr) > boost_distance(aim, prev))
             && (changes = true));
    return changes;
}
//------------------------------------------
Robo::Control  RoboPos::MainDirections::measure(const Point &aim)
{  /*  We have <musclesCount> rulers. They must be matched 
    *  that way to approximate the aim.
    *  System of Linear Equations
    */
    Robo::Control controls;
    std::vector<int> indices;

    bool changes = true;
    while (changes)
    {
        for (Robo::joint_t j = 0; j < _owner.jointsCount(); ++j)
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

    for (auto j = Robo::joint_t(_owner.jointsCount()); j > 0U; --j)
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

