#include "BallInfo.h"


BallInfo::BallInfo(VisualBall * _visualBall) : VisualObjectInfo(),
                                               visualBall(_visualBall)
{

}

void BallInfo::updateLoc(boost::shared_ptr<BallEKF> ball)
{
    x          = ball->getXEst();
    y          = ball->getYEst();
    velX       = ball->getXVelocityEst() ;
    velY       = ball->getYVelocityEst() ;
    uncertX    = ball->getXUncert() ;
    uncertY    = ball->getYUncert() ;
    uncertVelX = ball->getXVelocityUncert() ;
    uncertVelY = ball->getYVelocityUncert() ;
}

void BallInfo::updateVision()
{
    VisualObjectInfo::updateVision(*visualBall);
    confidence = visualBall->getConfidence();
}
