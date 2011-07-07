// VisualDetection.cpp
#include "VisualDetection.h"

VisualDetection::VisualDetection(int _x, int _y, float _distance,
                                 float _bearing) : x(_x), y(_y),
                                                   distance(_distance),
                                                   bearing(_bearing)
{
}

VisualDetection::VisualDetection(const estimate& _est, int _x, int _y) :
    x(_x), y(_y),
    distance(_est.dist), bearing(_est.bearing),
    distanceVariance(static_cast<float>(_est.distance_variance)),
    bearingVariance(static_cast<float>(_est.bearing_variance))
{
}

VisualDetection::VisualDetection(const VisualDetection& other)
    : x(other.x), y(other.y), width(other.width), height(other.height),
      centerX(other.centerX), centerY(other.centerY), angleX(other.angleX),
      angleY(other.angleY), focDist(other.focDist), distance(other.distance),
      bearing(other.bearing), elevation(other.elevation),
      distanceVariance(other.distanceVariance),
      bearingVariance(other.bearingVariance) {}

VisualDetection::~VisualDetection() {}
