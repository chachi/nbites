#ifndef _Location_h_DEFINED
#define _Location_h_DEFINED

#include <math.h>


class Location
{
public:
    Location(float _x, float _y);
    virtual ~Location();
    float distTo(const Location& other) const {
        return hypot(x - other.getX(),
                     y - other.getY() ); }

    float headingTo(const Location& other)  const{ atan2(other.getY() - y,
                                                         other.getX() - x); }
    bool inOppGoalBox() const;
    bool inMyGoalBox() const;
    bool inCenterOfField() const;
    bool inTopOfField() const;
    bool inBottomOfField() const;

    float getX() const { return x; };
    float getY() const { return y; };

private:
    float x, y;

};

class RobotLocation : public Location
{
public:
    RobotLocation(float _x, float _y, float _h);
    virtual ~RobotLocation();

    float getRelativeBearing(const Location& p) const;
    float spinDirToPoint(const Location& p) const;

    bool isFacingSideline() const;

    float getH() const { return h; }

private:
    float h;
};

class RelativeLocation
{
public:
    RelativeLocation(float _x, float _y);
    virtual ~RelativeLocation();

    float getRelX() const { return x; }
    float getRelY() const { return y; }

private:
    float x, y;
};

#endif /* _Location_h_DEFINED */
