#ifndef AGENT_H
#define AGENT_H

#include "camera.h"
#include "structures.h"

#include <string>
#include <iostream>
#include <pthread.h>

struct TAIAgent {
	
	static const MIN_DISTANCE_TO_WALL = 200;
	
    pthread_t AgentThread;
    bool Running;
    bool Started;

    TCamera* Camera;
    TMove2* Move;

    int Start();
    int Stop();
    void* Run();

    enum TGoal{
        NOGOAL = 0,
        WALKTOWALL
    };

    TGoal Goal;

    TAIAgent(TCamera* camera,
             TMove2* move) :
        Camera(camera),
        Move(move),
        Goal(TGoal::NOGOAL) {}

    ~TAIAgent() {
        Stop();
    }
    
    void WalkToWall();
    
    void SetGoal(TGoal goal);
};

#endif
