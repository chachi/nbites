#include "HoughVisualLine.h"
#include <iostream>
#include "Utility.h"

using namespace std;

HoughVisualLine::HoughVisualLine(const HoughLine& a, const HoughLine& b) :
    mLines(a, b)
{
    calculateLineParameters();
}

void HoughVisualLine::calculateLinePoint()
{
    if (mLines.first.intersects(mLines.second, mPoint)) {
        mPoint.x += IMAGE_WIDTH/2;
        mPoint.y += IMAGE_HEIGHT/2;
    } else {
        HoughLine& hl_1 = mLines.first;
        HoughLine& hl_2 = mLines.second;
        point<double> onHL_1 = hl_1.getOriginPoint();

        point<double> onHL_2_1 = hl_2.getOriginPoint();

        // Get a point a distance from the first
        point<double> onHL_2_2(onHL_2_1.x + hl_2.getSinT() * 5,
                               onHL_2_1.x - hl_2.getCosT() * 5);

        // If they're parallel, then just get the point halfway
        // between the two lines
        point<int> onHL_2_close = Utility::getClosestPointOnLine(onHL_1,
                                                                  onHL_2_1,
                                                                  onHL_2_2);
        mPoint = point<double>((onHL_1.x + onHL_2_close.x)/2,
                               (onHL_1.y + onHL_2_close.y)/2);
    }
}

void HoughVisualLine::calculateSlope()
{
    if (mLines.first.intersects(mLines.second, mPoint)) {
        // Angle bisector of two lines:
        // Calculation from
// http://demonstrations.wolfram.com/AngleBisectorsOfTwoIntersectingLines/
        double dy_1 = -mLines.first.getCosT();
        double dx_1 =  mLines.first.getSinT();

        double dy_2 = -mLines.second.getCosT();
        double dx_2 =  mLines.second.getSinT();

        double m_1 = dy_1/dx_1;
        double m_2 = dy_2/dx_2;

        mDy = m_1*m_2 - 1 + sqrt((m_1*m_1 + 1)*(m_2*m_2 + 1));
        mDx = m_1 + m_2;

    } else {
        // If they're parallel, then just use the slope of one of the
        // lines
        mDy = -mLines.first.getCosT();
        mDx = mLines.first.getSinT();
    }
}

void HoughVisualLine::calculateLineParameters()
{
    calculateLinePoint();
    calculateSlope();
}

bool HoughVisualLine::intersects(const HoughVisualLine& other,
                                 point<int>& out) const
{
    point<int> a, b, c, d;
    bool allIntersecting =
        mLines.first.intersects(other.getHoughLines().first, a)  &&
        mLines.first.intersects(other.getHoughLines().second, b) &&
        mLines.second.intersects(other.getHoughLines().first, c) &&
        mLines.second.intersects(other.getHoughLines().second, d);

    if (allIntersecting) {
        out.x = (a.x + b.x + c.x + d.x)/4;
        out.y = (a.y + b.y + c.y + d.y)/4;
    }
    return allIntersecting;
}

double HoughVisualLine::getDy() const
{
    return mDy;
}

double HoughVisualLine::getDx() const
{
    return mDx;
}

point<int> HoughVisualLine::getPointOnLine() const
{
    return mPoint;
}
