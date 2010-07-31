#ifndef _BallInfo_h_DEFINED
#define _BallInfo_h_DEFINED

#include "VisualObjectInfo.h"
#include "boost/shared_ptr.hpp"

#include "localization/BallEKF.h"

class VisualBall;
class BallEKF;

class BallInfo : public VisualObjectInfo
{
public:
    BallInfo(VisualBall * _visualBall);
    virtual ~BallInfo();

    void updateLoc(boost::shared_ptr<BallEKF> ballLoc);
    virtual void updateVision();

public:
    float x, y, velX, velY, uncertX, uncertY,
        uncertVelX, uncertVelY, sd,
        locDist, locBearing, relVelX, relVelY;
    int confidence;

private:
    VisualBall * visualBall;
};

#endif /* _BallInfo_h_DEFINED */
