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

        //cout << "[";
        //for (uint32_t i = 0; i < Camera->PARTS; ++i) {
        //    cout << Camera->WallDist[i] << ", ";
        //}
        //cout << "]" << endl;


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
    double left_min = 10000;
    double right_min = 10000;
    for(uint32_t i = 0; i <= Camera->PARTS / 2; ++i) {
        auto d = Camera->WallDist[i];
        if (d >= 0) {
            if (d < left_min) {
                left_min = d;
            }
        }
    }
    for(uint32_t i = Camera->PARTS / 2; i < Camera->PARTS; ++i) {
        auto d = Camera->WallDist[i];
        if (d >= 0) {
            if (d < right_min) {
                right_min = d;
            }
        }
    }

    auto min = right_min < left_min ? right_min : left_min;

    cout << "left=" << left_min << " right_=" << right_min;

    if (min >= MIN_DISTANCE_TO_WALL + DISTANCE_ACCURACY) {
		cout << " WTW min=" << min << " can walk";

		if ((Move->Command != TMove2::ECommand::CMD_DIR) ||
		    (Move->CurrentModel->Status != IMoveModel::STATUS_INCOMPLETE) ||
            (Move->MoveDirModel.Dir != M_PI)) {

			Move->moveDir(Move->MoveDirModel.Skeleton->S, M_PI);
		}
	} else {
        if (min < MIN_DISTANCE_TO_WALL - DISTANCE_ACCURACY) {
		    cout << " WTW min=" << min << " need to go back ";
		    if ((Move->Command != TMove2::ECommand::CMD_DIR) ||
		        (Move->CurrentModel->Status != IMoveModel::STATUS_INCOMPLETE) || 
                (Move->MoveDirModel.Dir != 0)) {

			        Move->moveDir(Move->MoveDirModel.Skeleton->S, 0);
		    }

        } else {
            auto diff = right_min - left_min;
            if (abs(diff) > DISTANCE_ACCURACY) {
                double angle = diff > 0 ? -M_PI / 6 : M_PI / 6;
                cout << " diff=" << diff << " need turn=" << angle;

                Move->turn(angle);
            
            } else {
        		cout << " diff=" << diff << " can stop";
	        	if ((Move->CurrentModel == &Move->MoveDirModel) && 
		            (Move->MoveDirModel.Status == IMoveModel::STATUS_INCOMPLETE)) {

			            Move->Stop();
                }
            }
		}
	}
}
