/* -*- C++ -*- Tell editors this is a C++ file (despite it's .in extension) */
/* This is an auto generated file. Please do not edit it.*/
#ifndef _State_h_DEFINED
#define _State_h_DEFINED

#include <string>
#include "State.h"
#include <boost/shared_ptr.hpp>

#include "Brain.h"

#include "include/Common.h"

class State
{

public:

    State(int _index, boost::shared_ptr<Brain> _brain, std::string name);
    virtual ~State() { };

    // Public interface
public:
    void execute();
    void resetState();
    int index() { return stateIndex; }

private:

    void baseRun();

    virtual void preRun() {};
    virtual void postRun() {};

    virtual void run() = 0;
    virtual void reset() = 0;

protected:
    boost::shared_ptr<Brain> brain;
    std::string name;
    int counter;

    long long int stateTime;

private:
    long long int startTime;
    int stateIndex;

};

#endif
