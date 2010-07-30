#include <time.h>
#include <sys/time.h>
#include <iostream>
#include "State.h"

State::State(int _index, boost::shared_ptr<Brain> _brain, std::string _name) :
    brain(_brain), name(_name), counter(0), stateTime(0), startTime(0),
    index(_index)
{

}

void State::resetState()
{
    counter = 0;
    startTime = micro_time();
    reset();
}

void State::baseRun()
{
    stateTime = micro_time() - startTime;
    counter++;

    run();
}

void State::execute()
{
    preRun();
    baseRun();
    postRun();
}
