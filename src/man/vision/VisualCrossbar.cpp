#include "VisualCrossbar.h"
VisualCrossbar::VisualCrossbar() : VisualDetection()
{
    init();
}

VisualCrossbar::VisualCrossbar(const VisualCrossbar& o) : VisualDetection(o) {}

// Initialization, happens every frame.
void VisualCrossbar::init()
{
    width = 0;
    height = 0;
    setX(0);
    setY(0);
    centerX = 0;
    centerY = 0;
    angleX = 0;
    angleY = 0;
    focDist = 0;
    setDistance(0);
    setBearing(0);
    elevation = 0;
}

/*
 * As we saw with beacons, we tend to work with blobs for convenience.  So at
 * some point we need to transfer their contents over to the field object that
 * we have identified.
 *
 * @param b    the blob that contains the information we need
 */
void VisualCrossbar::updateCrossbar(blob * b)
{
    setLeftTopX(b->leftTop.x);
    setLeftTopY(b->leftTop.y);
    setLeftBottomX(b->leftBottom.x);
    setLeftBottomY(b->leftBottom.y);
    setRightTopX(b->rightTop.x);
    setRightTopY(b->rightTop.y);
    setRightBottomX(b->rightBottom.x);
    setRightBottomY(b->rightBottom.y);
    setX(b->leftTop.x);
    setY(b->leftTop.y);
    setWidth(dist(b->rightTop.x, b->rightTop.y, b->leftTop.x, b->leftTop.y));
    setHeight(dist(b->leftTop.x, b->leftTop.y,
                   b->leftBottom.x, b->leftBottom.y));
    setCenterX(getLeftTopX() + ROUND2(getWidth() / 2));
    setCenterY(getRightTopY() + ROUND2(getHeight() / 2));
    setDistance(1);
}
