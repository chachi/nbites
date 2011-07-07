
// This file is part of Man, a robotic perception, locomotion, and
// team strategy application created by the Northern Bites RoboCup
// team of Bowdoin College in Brunswick, Maine, for the Aldebaran
// Nao robot.
//
// Man is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Man is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser Public License for more details.
//
// You should have received a copy of the GNU General Public License
// and the GNU Lesser Public License along with Man.  If not, see
// <http://www.gnu.org/licenses/>.

#include <boost/shared_ptr.hpp>
using boost::shared_ptr;

#include "Sensors.h"
#include "NaoPose.h"
#include "WalkProvider.h"
using namespace std;

using namespace Kinematics;

//#define DEBUG_WALKPROVIDER
#define DEBUG_ODOMETRY

WalkProvider::WalkProvider(shared_ptr<Sensors> s,
                           shared_ptr<NaoPose> _pose)
    : MotionProvider(WALK_PROVIDER),
      sensors(s),
      pose(_pose),
      metaGait(),
      nextGait(DEFAULT_GAIT),
      startGait(DEFAULT_GAIT),
      stepGenerator(sensors, pose, &metaGait),
      pendingCommands(false),
      pendingStepCommands(false),
      pendingDestCommands(false),
      pendingGaitCommands(false),
      pendingStartGaitCommands(false),
	  calculatedOdoThisFrame(false),
	  odometryUpdate(vector<float>(3,0)),
      nextCommand(new WalkCommand(0,0,0)),
      nextDestCommand(new DestinationCommand(0,0,0))
{
    pthread_mutex_init(&walk_provider_mutex, NULL);

    setActive();
}

WalkProvider::~WalkProvider() {
    pthread_mutex_destroy(&walk_provider_mutex);
}

void WalkProvider::requestStopFirstInstance() {
    setCommand(WalkCommand::ptr( new WalkCommand(0.0f, 0.0f, 0.0f) ));
}

void WalkProvider::hardReset(){
    pthread_mutex_lock(&walk_provider_mutex);
    stepGenerator.resetHard();
    pendingCommands = pendingStepCommands = pendingDestCommands = false;
    setActive();
    pthread_mutex_unlock(&walk_provider_mutex);
}

void WalkProvider::calculateNextJointsAndStiffnesses() {
    PROF_ENTER(P_WALK);

#ifdef DEBUG_WALKPROVIDER
    cout << "WalkProvider::calculateNextJointsAndStiffnesses()"<<endl;
#endif
    pthread_mutex_lock(&walk_provider_mutex);
    if ( pendingGaitCommands){
        if(stepGenerator.isDone() && pendingStartGaitCommands){
            metaGait.setStartGait(startGait);
        }
        metaGait.setNewGaitTarget(nextGait);
    }
    pendingGaitCommands = false;
    pendingStartGaitCommands = false;
	// since multiple calls to getOdometry won't work
    calculatedOdoThisFrame = false;

    //The meta gait needs to be ticked BEFORE any commands are sent to stepGen
    metaGait.tick_gait();

    if(nextCommand){
        stepGenerator.setSpeed(nextCommand->x_mms,
                               nextCommand->y_mms,
                               nextCommand->theta_rads);
        if (nextDestCommand)
            nextDestCommand->finishedExecuting();
    }
    pendingCommands = false;
    nextCommand = WalkCommand::ptr();

    if(pendingStepCommands){
        stepGenerator.takeSteps(nextStepCommand->x_mms,
                                nextStepCommand->y_mms,
                                nextStepCommand->theta_rads,
                                nextStepCommand->numSteps);
        nextDestCommand->finishedExecuting();
    }
    pendingStepCommands=false;

    if (pendingDestCommands) {
        int framesToDest = stepGenerator.setDestination(nextDestCommand->x_mm,
                                                        nextDestCommand->y_mm,
                                                        nextDestCommand->theta_rads,
                                                        nextDestCommand->gain);
        nextDestCommand->framesRemaining(framesToDest);
    }
    pendingDestCommands = false;

    if(!isActive()){
        cout << "WARNING, I wouldn't be calling the Walkprovider while"
            " it thinks its DONE if I were you!" <<endl;
    }

	// get the odometry update from StepGenerator
	const vector<float> odoDelta = updateOdometryFromMotion();

    // advance the in-progress DestinationCommand
    if (nextDestCommand) {
        nextDestCommand->tick();
        nextDestCommand->tickOdometry(odoDelta[0],
                                      odoDelta[1],
                                      odoDelta[2]);
#ifdef DEBUG_ODOMETRY
		if (!nextDestCommand->isDoneExecuting()) {
			cout << " Ticked DestCommand (" << nextDestCommand->framesRemaining()
				 << " left) X: " << odometryUpdate[0] << " Y: " << odometryUpdate[1]
				 << " Theta: " << odometryUpdate[2] << " finished? ";
			nextDestCommand->isDoneExecuting() ? cout<< " yes" : cout << " no";
			cout << endl;
		}
#endif
    }

    //ask the step Generator to update ZMP values, com targets
    stepGenerator.tick_controller();

    // Now ask the step generator to get the leg angles
    PROF_ENTER(P_TICKLEGS);
    WalkLegsTuple legs_result = stepGenerator.tick_legs();
    PROF_EXIT(P_TICKLEGS);

    //Finally, ask the step generator for the arm angles
    WalkArmsTuple arms_result = stepGenerator.tick_arms();

    //Get the joints and stiffnesses for each Leg
    vector<float> lleg_joints = legs_result.get<LEFT_FOOT>().get<JOINT_INDEX>();
    vector<float> rleg_joints = legs_result.get<RIGHT_FOOT>().get<JOINT_INDEX>();
    vector<float> lleg_gains = legs_result.get<LEFT_FOOT>().get<STIFF_INDEX>();
    vector<float> rleg_gains = legs_result.get<RIGHT_FOOT>().get<STIFF_INDEX>();

    //grab the stiffnesses for the arms
    vector<float> larm_joints = arms_result.get<LEFT_FOOT>().get<JOINT_INDEX>();
    vector<float> rarm_joints = arms_result.get<RIGHT_FOOT>().get<JOINT_INDEX>();
    vector<float> larm_gains = arms_result.get<LEFT_FOOT>().get<STIFF_INDEX>();
    vector<float> rarm_gains = arms_result.get<RIGHT_FOOT>().get<STIFF_INDEX>();


    //Return the joints for the legs
    setNextChainJoints(LARM_CHAIN,larm_joints);
    setNextChainJoints(LLEG_CHAIN,lleg_joints);
    setNextChainJoints(RLEG_CHAIN,rleg_joints);
    setNextChainJoints(RARM_CHAIN,rarm_joints);

    //Return the stiffnesses for each joint
    setNextChainStiffnesses(LARM_CHAIN,larm_gains);
    setNextChainStiffnesses(LLEG_CHAIN,lleg_gains);
    setNextChainStiffnesses(RLEG_CHAIN,rleg_gains);
    setNextChainStiffnesses(RARM_CHAIN,rarm_gains);

    setActive();
    pthread_mutex_unlock(&walk_provider_mutex);
    PROF_EXIT(P_WALK);
}

// if the pointer type is generic, cast it as a WalkCommand
void WalkProvider::setCommand(const MotionCommand::ptr command) {
	pthread_mutex_lock(&walk_provider_mutex);

	setCommand(boost::dynamic_pointer_cast<WalkCommand>(command));

	pthread_mutex_unlock(&walk_provider_mutex);
}

void WalkProvider::setCommand(const WalkCommand::ptr command){
    //grab the velocities in mm/second rad/second from WalkCommand
    pthread_mutex_lock(&walk_provider_mutex);

    nextCommand = command;
    pendingCommands = true;
    setActive();

    pthread_mutex_unlock(&walk_provider_mutex);
}

void WalkProvider::setCommand(const DestinationCommand::ptr command){
    pthread_mutex_lock(&walk_provider_mutex);
    // mark the old command as finished, for Python
    if (nextDestCommand)
        nextDestCommand->finishedExecuting();

    nextDestCommand = command;
    pendingDestCommands = true;
    setActive();
    pthread_mutex_unlock(&walk_provider_mutex);
}

void WalkProvider::setCommand(const Gait::ptr command){
    pthread_mutex_lock(&walk_provider_mutex);
    nextGait = Gait(*command);
    pendingGaitCommands = true;
    pthread_mutex_unlock(&walk_provider_mutex);
}
void WalkProvider::setCommand(const StepCommand::ptr command){
    pthread_mutex_lock(&walk_provider_mutex);
    nextStepCommand = command;
    pendingStepCommands = true;
    setActive();
    pthread_mutex_unlock(&walk_provider_mutex);
}
void WalkProvider::setActive(){
    //check to see if the walk engine is active
    if(stepGenerator.isDone() && !pendingCommands && !pendingStepCommands
           && !pendingDestCommands){
        inactive();
    }else{
        active();
    }
}

std::vector<BodyJointCommand::ptr> WalkProvider::getGaitTransitionCommand()
{
    vector<float> curJoints = sensors->getMotionBodyAngles();

    pthread_mutex_lock(&walk_provider_mutex);
    vector<float> gaitJoints = stepGenerator.getDefaultStance(nextGait);

    startGait = nextGait;
    pendingStartGaitCommands = true;

    pthread_mutex_unlock(&walk_provider_mutex);

    float max_change = -M_PI_FLOAT*10.0f;

    for(unsigned int i = 0; i < gaitJoints.size(); i++){
        max_change = max(max_change,
                         fabs(gaitJoints.at(i) - curJoints.at(i+HEAD_JOINTS)));
    }

    // this is the max we allow, not the max the hardware can do
    const float  MAX_RAD_PER_SEC =  M_PI_FLOAT*0.3f;
    float time = max_change/MAX_RAD_PER_SEC;

    vector<BodyJointCommand::ptr> commands;

    if(time <= MOTION_FRAME_LENGTH_S){
        return commands;
    }

    //larm: (0.,90.,0.,0.)
    //rarm: (0.,-90.,0.,0.)
    float larm_angles[] = {0.9f, 0.3f,0.0f,0.0f};
    float rarm_angles[] = {0.9f,-0.3f,0.0f,0.0f};

    vector<float>safe_larm(larm_angles,
                           &larm_angles[ARM_JOINTS]);
    vector<float> safe_rarm(rarm_angles,
                            &rarm_angles[ARM_JOINTS]);

    // HACK @joho get gait stiffness params. nextGait.maxStiffness
    vector<float> stiffness(Kinematics::NUM_JOINTS,
                            0.85f);
    vector<float> stiffness2(Kinematics::NUM_JOINTS,
                             0.85f);

    vector<float> empty(0);
    if (time > MOTION_FRAME_LENGTH_S * 30){
        commands.push_back(
            BodyJointCommand::ptr (
                new BodyJointCommand(0.5f,safe_larm, empty,empty,safe_rarm,
                                     stiffness,
                                     Kinematics::INTERPOLATION_SMOOTH)) );
    }

    commands.push_back(
        BodyJointCommand::ptr (
            new BodyJointCommand(time, gaitJoints, stiffness2,
                                 Kinematics::INTERPOLATION_SMOOTH))  );
    pthread_mutex_unlock(&walk_provider_mutex);
    return commands;
}

MotionModel WalkProvider::getOdometryUpdate() {
	/* if we're active, we let calculateNextJointsAndStiffnesses() call this,
	   otherwise we have to do it here */
	if (!isActive()) {
		updateOdometryFromMotion();
	}

	MotionModel tUp = MotionModel(odometryUpdate[0]*MM_TO_CM,
								  odometryUpdate[1]*MM_TO_CM,
								  odometryUpdate[2]);
	// reset the odometry updates that we've collected locally
	odometryUpdate[0] = 0;
	odometryUpdate[1] = 0;
	odometryUpdate[2] = 0;

	return tUp;
}

/**
 * Handles caching of the odometry from StepGenerator. Neccesary because
 * StepGenerator::getOdometryUpdate() is not a const function, does calculations
 * and changes things so we don't want to call it more than once per frame.
 */
vector<float> WalkProvider::updateOdometryFromMotion() {
    if (!calculatedOdoThisFrame) {
		vector<float> thisUpdate = stepGenerator.getOdometryUpdate();

		odometryUpdate[0] += thisUpdate[0];
		odometryUpdate[1] += thisUpdate[1];
		odometryUpdate[2] += thisUpdate[2];

#ifdef DEBUG_ODO
		cout << " odo this tick-- X: " << odometryUpdate[0]
			 << " Y: " << odometryUpdate[1]
			 << " Z: " << odometryUpdate[2] << endl;
#endif
        calculatedOdoThisFrame = true;
		return thisUpdate;
    }
	return vector<float>(0);
}
