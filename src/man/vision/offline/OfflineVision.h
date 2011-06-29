#ifndef _OfflineVision_h_DEFINED
#define _OfflineVision_h_DEFINED

#include <stdlib.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include "Vision.h"
#include "ColorParams.h"

class NaoPose;
class Sensors;
class Profiler;


/**
 * Honestly, a *hacked* together system to run our vision offline as an
 * executable.
 */
class OfflineVision
{
public:
    OfflineVision(int _iterations, int _first, int _last);
    virtual ~OfflineVision();

    int runOnDirectory(std::string path);

private:
    void initTable(string filename);

private:
    Vision * vision;
    boost::shared_ptr<Sensors> sensors;
    boost::shared_ptr<NaoPose> pose;
    boost::shared_ptr<Profiler> profiler;

    uint8 *table;
    ColorParams params;
    int numIterations, first, last;

    // COPIED FROM ALIMAGE_TRANSCRIBER
    enum {
        y0 = 0,
        u0 = 0,
        v0 = 0,

        y1 = 256,
        u1 = 256,
        v1 = 256,

        yLimit = 128,
        uLimit = 128,
        vLimit = 128,

        tableByteSize = yLimit * uLimit * vLimit
    };
};

#endif /* _OfflineVision_h_DEFINED */
