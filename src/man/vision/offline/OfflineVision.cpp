#include "OfflineVision.h"

#include <iostream>
#include <vector>

#include "Sensors.h"
#include "NaoPose.h"
#include "Profiler.h"
#include "Common.h"

using namespace std;
using boost::shared_ptr;

OfflineVision::OfflineVision(int _iterations, int _first, int _last) :
    table(new uint8[yLimit * uLimit * vLimit]),
    params(y0, u0, v0, y1, u1, v1, yLimit, uLimit, vLimit),
    numIterations(_iterations), first(_first), last(_last)
{
    assert(last >= first);
    sensors = shared_ptr<Sensors>(new Sensors(
                                      boost::shared_ptr<Speech>(new Speech())));
    pose = shared_ptr<NaoPose>(new NaoPose(sensors));
    profiler =
        shared_ptr<Profiler>(new Profiler(thread_micro_time));

    vision = new Vision(pose, profiler);

    // Could/should be an argument
    initTable("/home/jgmorris/robocup/nbites/data/tables/223-11.mtb");

#ifdef USE_TIME_PROFILING
    profiler->profiling = true;
    profiler->profileFrames((last-first+1) * numIterations);
    profiler->maxPrintDepth = 2;
#endif
}

OfflineVision::~OfflineVision()
{
    delete vision;
    delete table;
}

/**
 * Run the vision processing a set of directories given the path to
 * the directory and the bounds of image numbers on which to run.
 *
 * @note Only works with VERSIONED Nao images and requires that the
 *       images be numbered continuously from first to last
 */
int OfflineVision::runOnDirectory(std::string path)
{
    for (int c=0; c < numIterations; ++c){
        for (int i = first; i <= last; ++i){
            stringstream framePath;
            framePath << path << "/" << i << ".frm";
            sensors->loadFrame(framePath.str(), table, params);
            vision->notifyImage(sensors->getImage());
            cout << vision->fieldLines->getCorners()->size() << " " ;
            PROF_NFRAME(profiler);
        }
    }
    PROF_NFRAME(profiler);
    cout << endl;
    return 0;
}

void printUsage()
{
    cout << "Usage: ./OfflineVision <path-to-directory>"
         << " <# first frame> <# last frame>" << endl;
}

int main(int argv, char * argc[])
{
    if (argv < 4){
        printUsage();
        return 1;
    }

    int numIterations;
    if (argv == 4){
        numIterations = 1;
    } else {
        numIterations = atoi(argc[4]);
    }

    OfflineVision * off = new OfflineVision(numIterations,
                                            atoi(argc[2]), atoi(argc[3]));
    return off->runOnDirectory(argc[1]);
}

void OfflineVision::initTable(string filename)
{
    FILE *fp = fopen(filename.c_str(), "r");   //open table for reading

    if (fp == NULL) {
        printf("initTable() FAILED to open filename: %s", filename.c_str());
#ifdef OFFLINE
        exit(0);
#else
        return;
#endif
    }

    // actually read the table into memory
    // Color table is in VUY ordering
    int rval;
    for(int v=0; v < vLimit; ++v){
        for(int u=0; u< uLimit; ++u){
            rval = fread(&table[v * uLimit * yLimit + u * yLimit],
                         sizeof(unsigned char), yLimit, fp);
        }
    }

#ifndef OFFLINE
    printf("Loaded colortable %s\n",filename.c_str());
#endif

    fclose(fp);
}
