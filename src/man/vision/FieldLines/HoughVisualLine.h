#ifndef _HOUGHVISUALLINE_H_
#define _HOUGHVISUALLINE_H_

#include "geom/HoughLine.h"

class HoughVisualLine
{
public:
    HoughVisualLine(const HoughLine& a, const HoughLine& b);

    virtual ~HoughVisualLine() {}

    std::pair<HoughLine, HoughLine> getHoughLines() const { return mLines; }
    bool intersects(const HoughVisualLine& other, point<int>& out) const;

    double getDx() const;
    double getDy() const;
    point<int> getPointOnLine() const;


private:
    void calculateLineParameters();
    void calculateLinePoint();
    void calculateSlope();

    std::pair<HoughLine, HoughLine> mLines;

    double mDx, mDy;
    point<int> mPoint;
};

#endif /* _HOUGHVISUALLINE_H_ */
