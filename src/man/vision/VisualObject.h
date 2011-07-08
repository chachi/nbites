#ifndef _VisualObject_h_DEFINED
#define _VisualObject_h_DEFINED

#include "VisualDetection.h"
#include "VisualLandmark.h"

class VisualObject : public VisualLandmark, public VisualDetection
{
public:
    VisualObject(int _id,
                 const estimate _est,
                 int _x, int _y) : VisualLandmark(_id),
                                   VisualDetection(_est, _x, _y) { }

    VisualObject(int _id) : VisualLandmark(_id),
                            VisualDetection() { }

    VisualObject(const VisualObject& other) :
        VisualLandmark(other), VisualDetection(other) { }

    virtual ~VisualObject(){ }
};

#endif /* _VisualObject_h_DEFINED */
