#include "agent.h"

#include <sys/time.h>
#include <chrono>
#include <iomanip>

using namespace std;

void* threadF(void* obj) {
    return ((TAIAgent*)obj)->Run();

}

int TAIAgent::Start() {
    if (!Started) {
        Running = true;
        pthread_create(&AgentThread, NULL, threadF, this);
        return 0;
    }
    return -1;
}

int TAIAgent::Stop() {
    if (Started) {
        Running = false;
        pthread_join(AgentThread, NULL);
        Started = false;
        return 0;
    }
    return -1;
}

void* TAIAgent::Run() {
    Started = true;
    auto start = std::chrono::high_resolution_clock::now();
    while(Running) {
        auto now = std::chrono::high_resolution_clock::now();
        int status =  Camera->GetWall();
        cout << "Agent: " << std::fixed << std::setprecision(3) <<((now - start).count() / 1000000000.0);
        cout << " " << status << "\t";
        switch(Goal) {
            case NOGOAL:
                cout << "no goal";
                break;
            case WALKTOWALL:
                WalkToWall();
                break;

        }
        cout << endl;
    }

    return NULL;
}

void TAIAgent::SetGoal(TGoal goal) {
	Goal = goal;
	cout << "A new goal is set:" << goal << endl;
}

void TAIAgent::WalkToWall() {
    double min = 10000;
    for(uint32_t i = 0; i < Camera->PARTS; ++i) {
        auto d = Camera->WallDist[i];
        if (d >= 0) {
            if (d < min) {
                min = d;
            }
        }
        //cout << std::fixed << std::setprecision(2) << std::setw(5) << d << "\t";
    }
    if (min >= MIN_DISTANCE_TO_WALL) {
		cout << "WTW min=" << min << " can walk";
		if ((Move->Command != TMove2::ECommand::CMD_FORWARD) ||
		    (Move->CurrentModel->Status != IMoveModel::STATUS_INCOMPLETE)) {

			Move->moveForward(-Move->MoveForwardModel->Skeleton->S);
		}
	} else {
		cout << "WTW min=" << min << " need to stop";
		if ((Move->CurrentModel == &Move->MoveForwardModel) && 
		    (Move->MoveForwardModel.Status == IMoveModel::STATUS_INCOMPLETE) &&
		    (Move->MoveForwardModel.LeftDistance < 0)) {
			
			Move->Stop();
		}
	}
}
