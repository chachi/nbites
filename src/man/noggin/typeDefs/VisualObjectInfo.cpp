#include "VisualObjectInfo.h"
#include "VisualDetection.h"

void VisualObjectInfo::updateVision(const VisualDetection& obj)
{
    lastVisDist    = visDist;
    lastVisBearing = visBearing;
    lastCenterX    = centerX;
    lastCenterY    = centerY;
    lastAngleX     = angleX;
    lastAngleY     = angleY;

    centerX        = obj.getCenterX();
    centerY        = obj.getCenterY();
    width          = obj.getWidth();
    height         = obj.getHeight();
    focDist        = obj.getFocDist();
    visDist        = obj.getDistance();
    visBearing     = obj.getBearing();
    angleX         = obj.getAngleX();
    angleY         = obj.getAngleY();

    if (visDist > 0){
        on = true;
        framesOn++;
        framesOff = 0;
    } else {
        on = false;
        framesOff++;
        framesOn = 0;
    }
}
