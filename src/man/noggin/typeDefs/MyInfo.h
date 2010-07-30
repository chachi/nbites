#ifndef _MyInfo_h_DEFINED
#define _MyInfo_h_DEFINED

#include "Location.h"

class MyInfo
{
public:
    MyInfo();
    virtual ~MyInfo();

    RobotLocation getLocation() { return myLocation; }

public:
    RobotLocation myLocation;
    float uncertX, uncertY, uncertH;
    int playerNumber, teamNumber;

    int teamColor;

    int locScoreXY, locScoreTheta, locScoreFramesBad;
};

#endif /* _MyInfo_h_DEFINED */
