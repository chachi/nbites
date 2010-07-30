#include "Location.h"
#include "FieldConstants.h"

#include "NBMath.h"

Location::Location(float _x, float _y) : x(_x), y(_y)
{

}

virtual ~Location::Location() {}

bool Location::inOppGoalBox() const
{
    return (OPP_GOALBOX_LEFT_X   < x &&
            OPP_GOALBOX_RIGHT_X  > x &&
            OPP_GOALBOX_TOP_Y    > y &&
            OPP_GOALBOX_BOTTOM_Y < y );
}

bool Location::inMyGoalBox() const
{
    return (MY_GOALBOX_RIGHT_X  > x &&
            MY_GOALBOX_TOP_Y    > y &&
            MY_GOALBOX_BOTTOM_Y < y);
}

bool Location::inCenterOfField() const
{
    return (FIELD_HEIGHT * 2./3. > y &&
            FIELD_HEIGHT /3.     < y);
}

bool Location::inTopOfField() const
{
    return (FIELD_HEIGHT * 2./3. < y);
}

bool Location::inBottomOfField() const
{
    return (FIELD_HEIGHT / 3. > y);
}


RobotLocation::RobotLocation(float _x, float _y, float _h) : Location(_x, _y),
                                                             h(_h)
{

}

virtual RobotLocation::~RobotLocation() {}

float RobotLocation::getRelativeBearing(const Location& p) const
{
    return NBMath::subPIAngle(headingTo(p) - h);
}

// @TODO IMPLEMENT THESE!
float RobotLocation::spinDirToPoint(const Location& p) const;
bool RobotLocation::isFacingSideline() const;


