/**
 * Observation.h - The landmark observation class. Here we house all
 * those things needed to describe a single landmark sighting.
 * Observations can be of field objects, corners (line intersections),
 * lines, and possibly other things in the future (i.e. carpet edges)
 *
 * @author Tucker Hermans
 * @version %I%
 */

#ifndef Observation_h_DEFINED
#define Observation_h_DEFINED
#include <vector>
#include <list>

#include "NBMath.h"
#include "NogginStructs.h"
#include "VisualObject.h"
#include "VisualFieldObject.h"
#include "VisualCorner.h"
#include "VisualCross.h"

/**
 * The constructors for all point observations are identical based on
 * their types, but can't be templated because we don't want
 * Observation to have multiple different types.
 */
#define POINT_OBSERVATION_CONSTRUCTOR(VisT, ConcT)                      \
    Observation(const VisT &_object)                                    \
        : visDist(_object.getDistance()), visBearing(_object.getBearing()), \
          var_d(_object.getDistanceVariance()),                         \
          var_b(_object.getBearingVariance()),                          \
          id(_object.getID()), possibilities()                          \
    {                                                                   \
        const std::list <const ConcT *> * objList =                     \
            _object.getPossibilities();                                 \
        std::list<const ConcT*>::const_iterator i;                      \
        for( i = objList->begin(); i != objList->end(); ++i) {          \
            LandmarkT objectLandmark((**i).getFieldX(),                 \
                                     (**i).getFieldY());                \
            possibilities.push_back(objectLandmark);                    \
        }                                                               \
        if (var_b == 0.0 || var_d == 0.0){                          \
            std::cout << "Variance IS ZERO for obs: "    \
                      << *this << std::endl;                            \
        }                                                               \
    }



/**
 * @brief Class to hold the informations associated with the
 *        observation of a landmark.
 *
 *        Template argument defines the type of landmark which this
 *        Observation corresponds to.
 */
template<class LandmarkT>
class Observation
{
public:
    POINT_OBSERVATION_CONSTRUCTOR(VisualFieldObject, ConcreteFieldObject)
    POINT_OBSERVATION_CONSTRUCTOR(VisualCross, ConcreteCross)

    // We need a special constructor for corners since they have 3
    // arguments for their landmarks (rather than the two for points)
    Observation(const VisualCorner &_object)
    : visDist(_object.getDistance()), visBearing(_object.getBearing()),
        var_d(_object.getDistanceVariance()), var_b(_object.getBearingVariance()),
        id(_object.getID()), possibilities()
        {
            const std::list <const ConcreteCorner *> * objList =
                _object.getPossibilities();
            std::list<const ConcreteCorner*>::const_iterator i;
            for( i = objList->begin(); i != objList->end(); ++i) {
                LandmarkT objectLandmark((**i).getFieldX(),
                                         (**i).getFieldY(),
                                         (**i).getFieldAngle());
                possibilities.push_back(objectLandmark);
            }
        }

    virtual ~Observation() { }

    /**
     *
     * Visual information Getters
     *
     */
    float getVisDistance()    const { return visDist;                    }
    float getVisBearing()     const { return visBearing;                 }
    float getVisBearingDeg()  const { return TO_DEG * visBearing;        }
    float getDistanceVariance()     const { return var_d;                    }
    float getBearingVariance()      const { return var_b;                    }
    int getID()               const { return id;                         }
    int getNumPossibilities() const { return possibilities.size();       }
    bool isAmbiguous()        const { return  getNumPossibilities() > 1; }
    std::vector<LandmarkT> getPossibilities() const {
        return possibilities;
    }

    RangeBearingMeasurement getRangeBearingMeasurement() const {
        return RangeBearingMeasurement(visDist, visBearing, var_d, var_b);
    }

    /*
     *
     * Visual information SETTERS
     *
     */
    void setVisDistance(float _d)        { visDist = _d;    }
    void setVisBearing(float _b)         { visBearing = _b; }
    void setDistanceVariance(float _varD){ var_d = _varD;   }
    void setBearingVariance(float _varB) { var_b = _varB;   }
    void setID(int _id)                  { id = _id;        }

    /*
     * Helper functions
     */
    friend std::ostream& operator<< (std::ostream &o, const Observation &c) {
        return o << "Obs " << c.id << ": (" << c.visDist << ", " << c.visBearing
                 << ", " << c.var_d << ", " << c.var_b << ")";
    }

protected:
    // Vision information
    float visDist, visBearing;
    float var_d, var_b;

    // Identity information
    int id;
    std::vector<LandmarkT> possibilities;
};

// Shorthand for use elsewhere
typedef Observation<PointLandmark> PointObservation;

/**
 * Derived Corner Observation
 *
 * Gives corner orientation plus basic point Observation information
 */
class CornerObservation : public Observation<CornerLandmark>
{
public:
    CornerObservation(const VisualCorner& _c) :
        Observation<CornerLandmark>(_c),
        visOrientation(_c.getPhysicalOrientation()),
        var_o(_c.getPhysicalOrientationVariance()) {

        // Ensure that the variance is not zero (will cause errors later)
        if (var_o == 0.0){
            std::cout << "Zero variance is ZERO for obs: "
                      << *this << std::endl;
        }
    }

    virtual ~CornerObservation() { }

    float getVisOrientation() const { return visOrientation; };
    float getOrientationVariance()  const { return var_o;        };

    friend std::ostream& operator<< (std::ostream &o,
                                     const CornerObservation &c) {
        return o << "Obs " << c.id << ": (" << c.visDist << ", " << c.visBearing
                 << ", " << c.visOrientation << ", "
                 << c.var_d << ", " << c.var_b
                 << ", " << c.var_o <<  ")";
    }

private:
    float visOrientation;
    float var_o;
};

#endif // _Observation_h_DEFINED
