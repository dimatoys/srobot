#include "module.h"
#include "structures.h"
#include "agent.h"
#include <cmath>
#include <iostream>
#include <unistd.h>

using namespace std;

int init(TModuleObject* module) {
    initMechanics();
    module->Skeleton = new TSkeleton2("spider",
    // 0
                        14, 1520, 650,
                        15, 1400, 670,
                        18, 1460, -670,
    
    // 1
                        10, 1470, -675,
                        9, 1370, -650,
                        4, 1500, -660,
    
    // 2
                        22, 1440, -670,
                        27, 1430, -660,
                        17, 1570, -660,
    
    // 3
                        0, 1410, 650,
                        5, 1540, 650,
                        6, 1520, 650,
    
    // 4
                        13, 1550, 670,
                        19, 1550, 650,
                        26, 1460, 650,

    // 5
                        16, 1560, -650,
                        20, 1390, -660,
                        21, 1420, 650);
    
    auto move = new TMove2((TSkeleton2*)module->Skeleton);
    move->initPosition();
    module->Move = move;
    auto camera = TCamera::getCamera();
    module->Camera = camera;
    module->CameraWidth = camera->Width;
    module->CameraHeight = camera->Height;
    module->CameraMaxRange = camera->MaxRange;
    module->Parts = TCamera::PARTS;

    auto agent = new TAIAgent(camera, move);
    agent->Start();
    module->Agent = agent;

    return 0;
}

void shutdown(TModuleObject* module) {

    if (module->Agent != NULL) {
        delete (TAIAgent*)module->Agent;
        module->Agent = NULL;
    }
    
    if (module->Camera != NULL) {
        delete (TCamera*)module->Camera;
        module->Camera = NULL;
    }

    if (module->Move != NULL) {
        delete (TMove2*)module->Move;
        module->Move = NULL;
    }

    if (module->Skeleton != NULL) {
        delete (TSkeleton2*)module->Skeleton;
        module->Skeleton = NULL;
    }

    stopMechanics();
}

int run_cmd(TModuleObject* module, const char* cmd, const char* arg) {
    if (cmd != NULL) {
        std::string command = (std::string)cmd;
        if (module->Move != NULL) {
            TMove2* move = (TMove2*)module->Move;

            if (command == "walk") {
                auto d = stod(arg);
                move->moveForward(d);
                return 0;
            }

            if (command == "turn") {
                auto a = stod(arg) * M_PI / 180;
                move->turn(a);
                return 0;
            }

            if (command == "dir") {
                int comma = -1;
                int end = 0;
                while(arg[++end] != 0) {
                    if (arg[end] == ',') {
                        comma = end;
                    }
                }

                if (comma >= 0) {
                    string distance(arg, arg + comma);
                    string direction(arg + comma + 1, end);
                    move->moveDir(stod(distance), stod(direction));
                    return 0;
                } else {
                    return -1;
                }

            }

            if (command == "speed") {
                move->setSpeed(stod(arg));
                return 0;
            }
            if (command == "neutral") {
                move->toNeutral();
                return 0;
            }
            if (command == "down") {
                move->toDown();
                return 0;
            }
            
            if (command == "stop") {
				move->Stop();
				return 0;
			}
            
            if (command == "exit") {
                shutdown(module);
                return -2;
            }
            if (command == "pic") {
                TCamera* cam = ((TCamera*)module->Camera);
                cam->makePicture("/home/pi/git/sprobot/static/depth.jpg",
                                 "/home/pi/git/sprobot/static/color.jpg");
                for (uint32_t i = 0; i < cam->PARTS; ++i) {
                    module->WallDist[i] = cam->WallDist[i];
                }
                return 0;
            }
            
            if (command == "walkuntillwall") {
				TAIAgent* agent = (TAIAgent*)module->Agent;
				agent->SetGoal(TAIAgent::TGoal::WALKTOWALL);
				return 0;
			}

            if (command == "stopgoal") {
				TAIAgent* agent = (TAIAgent*)module->Agent;
				agent->SetGoal(TAIAgent::TGoal::NOGOAL);
				return 0;
			}

        } else {
            std::cout << "module is not initialized" << std::endl;
            return -3;
        }
    } else {
        std::cout << "no cmd is passed" << std::endl;
    }
    return -1;
}
