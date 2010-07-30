#include "Landmarks.h"

FieldObjectInfo::FieldObjectInfo(int _ID) : VisualObjectInfo(), absoluteID(_ID)
{
    // @TODO: Should set its absolute x and y
}

void FieldObjectInfo::updateVision(const VisualFieldObject& obj)
{
    VisualObjectInfo::updateVision(obj);
    IDCertainty = obj.getIDCertainty();
}

void FieldObjectInfo::associateWithFieldObject(float _x,
                                               float _y,
                                               int _relativeID)
{
    x = _x;
    y = _y;
    relativeID = _relativeID;
}
