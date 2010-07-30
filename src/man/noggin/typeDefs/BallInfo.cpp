#include "BallInfo.h"


void BallInfo::updateLoc(const BallEKF& ball)
{
    x          = ball.getXEst();
    y          = ball.getYEst();
    velX       = ball.getXVelocityEst() ;
    velY       = ball.getYVelocityEst() ;
    uncertX    = ball.getXUncert() ;
    uncertY    = ball.getYUncert() ;
    uncertVelX = ball.getXVelocityUncert() ;
    uncertVelY = ball.getYVelocityUncert() ;
}

void BallInfo::updateVision(const VisualBall& ball)
{
    VisualObjectInfo::updateVision(ball);
    confidence = ball.getConfidence();
}
