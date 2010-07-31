
#include "Brain.h"

using boost::shared_ptr;
#include "ConcreteFieldObject.h"
#include "typeDefs/Landmarks.h"

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
    ball(vision->ball), ygrp(vision->ygrp), yglp(vision->yglp),
    bgrp(vision->bgrp), bglp(vision->bglp), my(),
    certificate(BirthCertificate::makeBirthCertificate())
{
    makeFieldObjectsRelative();
    initTeamMembers();
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
    ball.updateLoc(ballLoc);
}

void Brain::updateVisionInfo()
{
    ball.updateVision();
    ygrp.updateVision();
    yglp.updateVision();
    bgrp.updateVision();
    bglp.updateVision();
}

void Brain::updateComm()
{

}

void Brain::broadcastInfo()
{

}


