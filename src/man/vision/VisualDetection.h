/**
 * @file   VisualDetection.h
 * @author Tucker Hermans <Tucker@liechtenstein>
 * @date   Mon Feb 16 22:49:04 2009
 *
 * @brief The Abstract super class of many visual objects that can be detected
 * by the vision system
 *
 */

#ifndef VisualDetection_h_defined
#define VisualDetection_h_defined

#include "Structs.h"
#include "NBMath.h"
class VisualDetection {
public:
    // Constructor
    VisualDetection(const estimate& _est, int _x, int _y);
    // Copy constructor
    VisualDetection(const VisualDetection &);
    VisualDetection();
    // Destructor
    virtual ~VisualDetection();

    /* SETTERS */
    // Best Guesses
    void setEstimate(const estimate& _est){
        setDistance(_est.dist);
        setBearing(_est.bearing);
        setBearingVariance(_est.bearing_variance);
        setDistanceVariance(_est.distance_variance);
    }
    void setWidth(float w) { width = w; }
    void setHeight(float h) { height = h; }
    void setCenterX(int cx) { centerX = cx; }
    void setCenterY(int cy) { centerY = cy; }
    void setAngleX(float aX) { angleX = aX; }
    void setAngleY(float aY) { angleY = aY; }
    void setFocDist(float fd) { focDist = fd; }
    void setDistance(float d) { distance = d; }
    void setBearing(float b) { bearing = b; }
    void setElevation(float e) { elevation = e; }
    void setX(int x1) {x = x1;}
    void setY(int y1) {y = y1;}
    void setDistanceVariance(float _distVariance) {
        distanceVariance = _distVariance;
    }
    void setBearingVariance(float _bearingVariance) {
        bearingVariance = _bearingVariance;
    }

    void setOn(bool _on){ on = _on; }
    void setFramesOn(int numOn){ framesOn = numOn; }
    void setFramesOff(int numOff){ framesOff = numOff; }

    /* GETTERS */
    const int getX() const {return x;}
    const int getY() const {return y;}
    const point<int> getLocation() const { return point<int>(x, y); }
    const float getWidth() const { return width; }
    const float getHeight() const { return height; }
    const int getCenterX() const { return centerX; }
    const int getCenterY() const { return centerY; }
    const float getAngleX() const { return angleX; }
    const float getAngleY() const { return angleY; }
    const float getAngleXDeg() const { return angleX*TO_DEG; }
    const float getAngleYDeg() const { return angleY*TO_DEG; }
    const float getFocDist() const { return focDist; }
    const float getDistance() const { return distance; }
    const float getBearing() const { return bearing; }
    const float getBearingDeg() const { return bearing*TO_DEG; }
    const float getElevation() const { return elevation; }
    const float getElevationDeg() const { return elevation*TO_DEG; }
    const float getDistanceVariance() const { return distanceVariance; }
    const float getBearingVariance() const { return bearingVariance; }
    const float getDistanceSD() const { return sqrtf(distanceVariance); }
    const float getBearingSD() const { return sqrtf(bearingVariance); }

    const bool isOn() const{ return on; }
    int getFramesOn() { return framesOn; }
    int getFramesOff(){ return framesOff; }

protected:
    /* Best guessed Ball Variables */
    int x, y;
    float width, height;
    int centerX, centerY;
    float angleX, angleY;
    float focDist;
    float distance, bearing, elevation;
    // Standard deviation of measurements
    float distanceVariance, bearingVariance;
    bool on;
    int framesOn, framesOff;
};

#endif // VisualDetection_h_defined
