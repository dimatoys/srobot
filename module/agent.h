#ifndef AGENT_H
#define AGENT_H

#include "camera.h"
#include "structures.h"

#include <string>
#include <iostream>
#include <pthread.h>

struct TAIAgent {
	
	static constexpr double MIN_DISTANCE_TO_WALL = 400;
	static constexpr double DISTANCE_ACCURACY = 20;
	
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
    
    void SetGoal(TGoal goal);

    void GetWallDistances(double* left_min, double* right_min);

    void WalkToWall();
    void NoGoal();    
};

#endif
