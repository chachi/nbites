#ifndef _Brain_h_DEFINED
#define _Brain_h_DEFINED

#include <boost/shared_ptr.hpp>


// Other subsystem includes and foward declarations
#include "Comm.h"
#include "Profiler.h"
class Comm;
class LocSystem;
class MotionInterface;
class RoboGuardian;
class RoboGuardian;
class Sensors;
class Vision;

// Behavior modules
// class Player;
// class Navigator;
// class HeadTracker;
// class FallController;

// Noggin information classes
#include "typeDefs/BallInfo.h"
#include "typeDefs/Landmarks.h"
#include "typeDefs/MyInfo.h"
#include "robots/BirthCertificate.h"


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
    virtual ~Brain();

    // Private member functions
private:
    // Init methods
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
    MotionInterface * motion;
    boost::shared_ptr<RoboGuardian> guardian;
    boost::shared_ptr<Profiler> profiler;

    // Behavior FSAs

    // Player player;
    // boost::shared_ptr<Navigator> navigator;
    // boost::shared_ptr<HeadTracker> tracker;
    // boost::shared_ptr<FallController> fallController;
    // boost::shared_ptr<GameAwareness> gameAwareness;
    // boost::shared_ptr<LedController> leds;

    // Current info in Brain

    // Vision Info
    BallInfo ball;
    FieldObjectInfo ygrp, yglp, bgrp, bglp;
    FieldObjectInfo * myGoalRightPost, * myGoalLeftPost,
        * oppGoalRightPost, * oppGoalLeftPost;

    // Sensor info
    // Sonar sonar;

    // Self info
    MyInfo my;
    BirthCertificate certificate;
};
#endif /* _Brain_h_DEFINED */
