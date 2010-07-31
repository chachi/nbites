#ifndef _Landmarks_h_DEFINED
#define _Landmarks_h_DEFINED


#include "localization/LocSystem.h"
#include "VisualObjectInfo.h"

class VisualFieldObject;

class FieldObjectInfo : public VisualObjectInfo
{
public:
    FieldObjectInfo(VisualFieldObject * visualObject);
    virtual ~FieldObjectInfo();

    void updateLocalization(const LocSystem& loc);
    virtual void updateVision();

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

private:
    VisualFieldObject * associatedFieldObject;

};

#endif /* _Landmarks_h_DEFINED */
