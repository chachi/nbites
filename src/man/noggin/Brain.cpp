
#include "Brain.h"

using boost::shared_ptr;

#include "ConcreteFieldObject.h"

Brain::Brain(shared_ptr<Vision> _vision,
             shared_ptr<LocSystem> _loc,
             shared_ptr<BallEKF> _ballLoc,
             shared_ptr<Sensors> _sensors,
             shared_ptr<Comm> _comm,
             MotionInterface * _motion,
             shared_ptr<RoboGuardian> _guardian,
             shared_ptr<Profiler> _profiler) :
    vision(_vision), loc(_loc), ballLoc(_ballLoc), sensors(_sensors),
    comm(_comm), motion(_motion), guardian(_guardian), profiler(_profiler),
    ball(), ygrp(YELLOW_GOAL_RIGHT_POST), yglp(YELLOW_GOAL_LEFT_POST),
    bgrp(BLUE_GOAL_RIGHT_POST), bglp(BLUE_GOAL_LEFT_POST), my()
{
    initFieldObjects();
    makeFieldObjectsRelative();
    initTeamMembers();
}

void Brain::initFieldObjects()
{

}

void Brain::makeFieldObjectsRelative()
{
    if (my.teamColor == TEAM_BLUE){
        oppGoalRightPost = &yglp;
        oppGoalLeftPost = &ygrp;

        myGoalRightPost = &bgrp;
        myGoalLeftPost = &bglp;
    } else {
        oppGoalRightPost = &bglp;
        oppGoalLeftPost = &bgrp;

        myGoalRightPost = &ygrp;
        myGoalLeftPost = &yglp;
    }

    oppGoalLeftPost->associateWithFieldObject(
        LANDMARK_OPP_GOAL_LEFT_POST_X,
        LANDMARK_OPP_GOAL_LEFT_POST_Y,
        FieldObjectInfo::OPP_GOAL_LEFT_POST_ID
        );

    oppGoalRightPost->associateWithFieldObject(
        LANDMARK_OPP_GOAL_RIGHT_POST_X,
        LANDMARK_OPP_GOAL_RIGHT_POST_Y,
        FieldObjectInfo::OPP_GOAL_RIGHT_POST_ID
        );

    myGoalLeftPost->associateWithFieldObject(
        LANDMARK_MY_GOAL_LEFT_POST_X,
        LANDMARK_MY_GOAL_LEFT_POST_Y,
        FieldObjectInfo::MY_GOAL_LEFT_POST_ID
        );

    myGoalRightPost->associateWithFieldObject(
        LANDMARK_MY_GOAL_RIGHT_POST_X,
        LANDMARK_MY_GOAL_RIGHT_POST_Y,
        FieldObjectInfo::MY_GOAL_RIGHT_POST_ID
        );
}

void Brain::initTeamMembers()
{

}

void Brain::run()
{

}

void Brain::updateLocalization()
{
    ball.updateLoc(*ballLoc);
}

void Brain::updateVisionInfo()
{
    ball.updateVision(*vision->ball);
}

void Brain::updateComm()
{

}

void Brain::broadcastInfo()
{

}


