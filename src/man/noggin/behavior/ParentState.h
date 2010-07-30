#ifndef _ParentState_h_DEFINED
#define _ParentState_h_DEFINED

#include <list>
#include "State.h"

class ParentState : public State
{
public:
    ParentState(int index, boost::shared_ptr<Brain> brain, std::string name);
    virtual ~ParentState() {};
private:
    virtual State * getNextSubState() = 0;
    void run();

protected:
    void switchTo(State* next);
    void addSubState(State *);

private:
    void updateStates();

protected:
    std::list<State*> states;

    State* currentState;
    State* lastState;
    State* lastDiffState;

private:
    bool forceSwitch;
    State * nextState;

};

#endif

