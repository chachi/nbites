#include "Landmarks.h"

FieldObjectInfo::FieldObjectInfo(VisualFieldObject * visualObject) :
    VisualObjectInfo(), absoluteID(visualObject->getID())
{

}

void FieldObjectInfo::updateVision()
{
    VisualObjectInfo::updateVision(*associatedFieldObject);
    IDCertainty = associatedFieldObject->getIDCertainty();
}

void FieldObjectInfo::associateWithFieldObject(float _x,
                                               float _y,
                                               int _relativeID)
{
    x = _x;
    y = _y;
    relativeID = _relativeID;
}
