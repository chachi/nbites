#include "FieldLinesDetector.h"
#include <stdio.h>
#include <boost/make_shared.hpp>
#include "HoughSpace.h"
#include "EdgeDetector.h"
#include "Gradient.h"

using namespace std;
using boost::shared_ptr;

FieldLinesDetector::FieldLinesDetector() :
    mEdges(new EdgeDetector),
    mGradient(new Gradient),
    mHoughLines()
{
    mHough = HoughSpace::create();
}

/**
 * Detect field lines and their intersections (aka corners) in the given image
 */
void FieldLinesDetector::detect(int upperBound,
                                int* field_edge,
                                const uint16_t *img)
{
    // For safety (in case horizon is too low), scan from above the
    // given upperbound
    upperBound -= 10;

    // Only use values within the image
    upperBound = min(max(0, upperBound), IMAGE_HEIGHT-3);

    findHoughLines(upperBound, field_edge, img);
    findFieldLines();
}

/**
 * Computes the gradient over the Y Channel of the image and
 * runs a hough transform to find all the pairs of hough space lines
 * in an image.
 *
 * Side effects: Updates gradient with current image's gradient values,
 *               updates list of hough space lines
 */
void FieldLinesDetector::findHoughLines(int upperBound,
                                        int* field_edge,
                                        const uint16_t *img)
{
    mGradient->reset();
    mEdges->detectEdges(upperBound, field_edge, img, *mGradient);
    mHoughLines = mHough->findLines(*mGradient);
}

/**
 * Using the found hough lines and the gradient image, find the actual
 * field lines in the image.
 */
void FieldLinesDetector::findFieldLines()
{
    mLines.clear();
    list<pair<HoughLine, HoughLine> >::const_iterator hl;
    for(hl = mHoughLines.begin(); hl != mHoughLines.end(); ++hl){
        mLines.push_back(HoughVisualLine(hl->first, hl->second));
    }
    trimFieldLines();
}

bool calculateU(const HoughVisualLine& line, const AnglePeak& peak, double& u)
{
    pair<HoughLine, HoughLine> lines = line.getHoughLines();
    if ( lines.first.isOnLine(peak) ||
         lines.second.isOnLine(peak)){
        point<int> pt = line.getPointOnLine();
        pt.x -= IMAGE_WIDTH/2;
        pt.y -= IMAGE_HEIGHT/2;

        double dx = line.getDx();
        double dy = line.getDy();

        u = line.getDx() * (peak.x - pt.x) + line.getDy() * (peak.y - pt.y);
        return true;
    }
    return false;
}

void FieldLinesDetector::trimFieldLines()
{
    vector<double> uMean(mLines.size(), 0);
    vector<double> uMeanSq(mLines.size(), 0);
    vector<int> numPts(mLines.size(), 0);
    double u;

    for (int i = 0; mGradient->isPeak(i); ++i) {
        for (int j = 0; j < mLines.size(); ++j) {
            const HoughVisualLine& hvl = mLines[j];

            if (calculateU(hvl, mGradient->angles[i], u)) {
                uMean[j] += u;
                uMeanSq[j] += u*u;
                numPts[j]++;
            }
        }
    }

    for (int j = 0; j < mLines.size(); ++j) {
        HoughVisualLine& hvl = mLines[j];
        point<int> pt_0 = hvl.getPointOnLine();

        if (numPts[j] == 0) continue;

        cout << uMean[j] << endl;
        cout << uMeanSq[j] << endl;

        uMean[j]   /= numPts[j];
        uMeanSq[j] /= numPts[j];

        double uVar = uMeanSq[j] - uMean[j]*uMean[j];
        double uSd = std::sqrt(uVar);

        cout << "uMean[j]: " << uMean[j] << endl;
        cout << "uSd: " << uSd << endl;

        cout << "dx, dy: " << hvl.getDx() << ", " << hvl.getDy() << endl;

        point<int> meanPt(uMean[j] * hvl.getDx() + pt_0.x,
                          uMean[j] * hvl.getDy() + pt_0.y);
        cout << "pt: " << pt_0 << endl;
        cout << "mean: " << meanPt << endl;

        float xOff = hvl.getDx() * uSd * 1.5;
        float yOff = hvl.getDy() * uSd * 1.5;

        cout << "Offsets: " << xOff << " " << yOff << endl;

        // Package and return endpoints
        hvl.setEndpoints(point<int>(meanPt.x + xOff, meanPt.y + yOff),
                         point<int>(meanPt.x - xOff, meanPt.y - yOff));
        cout << endl << endl;
    }
}

void FieldLinesDetector::setEdgeThreshold(int thresh)
{
    mEdges->setThreshold(static_cast<uint8_t>(thresh));
}

void FieldLinesDetector::setHoughAcceptThreshold(int thresh)
{
    mHough->setAcceptThreshold(thresh);
}

list<HoughLine> FieldLinesDetector::getHoughLines() const
{
    list<HoughLine> lines;
    list<pair<HoughLine, HoughLine> >::const_iterator i;
    for(i = mHoughLines.begin(); i != mHoughLines.end(); ++i){
        lines.push_back(i->first);
        lines.push_back(i->second);
    }
    return lines;
}
