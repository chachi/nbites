/**
 * Vision Ball class
 */

#include "VisualBall.h"
#include "math.h"

VisualBall::VisualBall()
{
    init();
    framesOn = 0;
    framesOff = 0;
}

/* Initializes Variables every frame. Make sure before using any of these
 * variables, to check to see if the ball is in frame by checking distance
 * first (if it's > 0)
 */
void VisualBall::init() {
    // Main Variables
    width = 0;
    height = 0;
    radius = 0;
    centerX = 0;
    centerY = 0;
    angleX = 0;
    angleY = 0;
    focDist = 0;
    distance = 0;
    bearing = 0;
    elevation = 0;
	heat = 0.0f;
    on = false;
}
