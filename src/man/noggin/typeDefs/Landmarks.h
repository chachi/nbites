#ifndef _Landmarks_h_DEFINED
#define _Landmarks_h_DEFINED

#include "localization/LocSystem.h"
#include "VisualObjectInfo.h"
#include "VisualFieldObject.h"


class FieldObjectInfo : public VisualObjectInfo
{
public:
    FieldObjectInfo(int objectID);
    virtual ~FieldObjectInfo();

    void updateLocalization(const LocSystem& loc);
    void updateVision(const VisualFieldObject& obj );

    void associateWithFieldObject(float x, float y, int _relativeID);

public:
    int absoluteID, relativeID;
    float locBearing, locDist;
    int x, y;
    certainty IDCertainty;
    distanceCertainty distCertainty;

    enum relativeLandmarkID {
        MY_GOAL_LEFT_POST_ID,
        MY_GOAL_RIGHT_POST_ID,
        OPP_GOAL_LEFT_POST_ID,
        OPP_GOAL_RIGHT_POST_ID
    };
};

#endif /* _Landmarks_h_DEFINED */
