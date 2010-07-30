#include "ParentState.h"

ParentState::ParentState(int index, boost::shared_ptr<Brain> brain,
                         std::string name) :
    State(index, brain, name), forceSwitch(false)
{

}

void ParentState::run()
{
    updateStates();
    currentState->execute();
}

void ParentState::updateStates()
{
    lastState = currentState;
    currentState = forceSwitch ? nextState : getNextSubState();

    forceSwitch = false;

    if (currentState != lastState){
        lastDiffState = lastState;
        currentState->resetState();
    }
}

void ParentState::switchTo(State * next)
{
    forceSwitch = true;
    nextState = next;
}


