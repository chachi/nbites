#ifndef _Brain_h_DEFINED
#define _Brain_h_DEFINED

#include <boost/shared_ptr.hpp>


// Other subsystem includes
#include "Comm.h"
#include "localization/LocSystem.h"
#include "MotionInterface.h"
#include "RoboGuardian.h"
#include "Sensors.h"
#include "Vision.h"
#include "localization/LocSystem.h"


// Behavior includes
// #include "behavior/player/Player.h"
// #include "behavior/navigator/Navigator.h"
// #include "behavior/headTracker/HeadTracker.h"
// #include "behavior/fallController/FallController.h"
// #include "behavior/gameController/GameController.h"

// Noggin information classes
#include "typeDefs/BallInfo.h"
#include "typeDefs/Landmarks.h"
#include "typeDefs/MyInfo.h"



class Brain
{

public:
    Brain(boost::shared_ptr<Vision> _vision,
          boost::shared_ptr<LocSystem> _loc,
          boost::shared_ptr<BallEKF> _ballLoc,
          boost::shared_ptr<Sensors> _sensors,
          boost::shared_ptr<Comm> _comm,
          MotionInterface* _motion,
          boost::shared_ptr<RoboGuardian> _guardian,
          boost::shared_ptr<Profiler> _profiler);
    virtual ~Brain() { };

    // Private member functions
private:
    // Init methods
    void initFieldObjects();
    void makeFieldObjectsRelative();
    void initTeamMembers();

    // Main run loop methods
    void run();
    void updateLocalization();
    void updateVisionInfo();
    void updateComm();
    void broadcastInfo();

    // Private member variables
private:
    // References to other robot systems
    boost::shared_ptr<Vision>  vision;
    boost::shared_ptr<LocSystem> loc;
    boost::shared_ptr<BallEKF> ballLoc;
    boost::shared_ptr<Sensors> sensors;
    boost::shared_ptr<Comm>    comm;
    boost::shared_ptr<MotionInterface> motion;
    boost::shared_ptr<RoboGuardian> guardian;
    boost::shared_ptr<Profiler> profiler;

    // Behavior FSAs

    // Player player;
    // Navigator navigator;
    // HeadTracker tracker;
    // FallController fallController;
    // GameAwareness gameAwareness;
    // LedController leds;

    // Current info in Brain

    // Vision Info
    BallInfo ball;
    FieldObjectInfo ygrp, yglp, bgrp, bglp;
    FieldObjectInfo * myGoalRightPost, * myGoalLeftPost,
    * oppGoalRightPost,* oppGoalLeftPost;


    // Sensor info
    // Sonar sonar;

    // Self info
    MyInfo my;
    // CoA coa;
};
#endif /* _Brain_h_DEFINED */
