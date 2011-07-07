
/**
 * Generic class to define a step that will kick the ball.
 *
 * @author Nathan Merritt
 * @date July 2011
 */

#ifndef KICKING_STEP_H
#define KICKING_STEP_H

#include <boost/shared_ptr.hpp>

#include "Step.h"

class KickingStep : public Step {
public:
    typedef ptr boost::shared_ptr<Step> ptr;

    KickingStep(WalkVector &ball_location,
		const AbstractGait &gait,
		const Foot _foot);

    bool finished() { return finished; }

private:
    bool finished;
};

#endif
