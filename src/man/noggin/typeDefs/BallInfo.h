#ifndef _BallInfo_h_DEFINED
#define _BallInfo_h_DEFINED


#include "VisualObjectInfo.h"

#include "VisualBall.h"
#include "localization/BallEKF.h"

class BallInfo : public VisualObjectInfo
{
public:
    BallInfo() : VisualObjectInfo() { }
    virtual ~BallInfo();

    void updateLoc(const BallEKF& ballLoc);
    virtual void updateVision(const VisualBall& ball);

public:
    float x, y, velX, velY, uncertX, uncertY,
        uncertVelX, uncertVelY, sd,
        locDist, locBearing, relVelX, relVelY;
    int confidence;
};

#endif /* _BallInfo_h_DEFINED */
