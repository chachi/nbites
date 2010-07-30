#ifndef _VisualObjectInfo_h_DEFINED
#define _VisualObjectInfo_h_DEFINED

#include "VisualDetection.h"
#include <stdio.h>

class VisualObjectInfo
{
public:
    VisualObjectInfo();
    virtual ~VisualObjectInfo();
    virtual void updateVision(const VisualDetection& obj);

    friend std::ostream& operator<< (std::ostream &o,
                                     const VisualObjectInfo& i){
        return o << "Center (" << i.centerX << ", " << i.centerY << ") " <<
            "Dist: " << i.visDist << " Bearing: " << i.visBearing << std::endl;
    }

public:
    float centerX, centerY;
    float angleX, angleY;
    float width, height;
    float visDist, visBearing;
    float focDist;

    int framesOn, framesOff;

    bool on;

    float lastVisDist;
    float lastVisBearing;
    float lastCenterX;
    float lastCenterY;
    float lastAngleX;
    float lastAngleY;
};

#endif /* _VisualObjectInfo_h_DEFINED */
