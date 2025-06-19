#include "module.h"
#include "structures.h"
#include <cmath>
#include <iostream>
#include <unistd.h>

using namespace std;

int init(TModuleObject* module) {
    initMechanics();
    module->Robot = new TSpider("spider",

    174, // Wm between middle
    147, // Wk between angles
    206, // Hk between angles

    // an = [180-45, 45, 180, 0, -(180-45), -45]

    // 0
        M_PI / 4,
        14, 1520, 650,
        15, 1400, 670,
        18, 1460, 670,
    // 1
        M_PI / 4,
        2, 1470, -660,
        3, 1370, -650,
        4, 1500, -660,
    // 2
        0,
        22, 1470, -670,
        27, 1430, -660,
        17, 1570, -660,
    // 3
        0,
        13, 1550, 650,
        19, 1550, 650,
        26, 1460, 650,
    // 4
        -M_PI / 4,
        0, 1410, 650,
        5, 1520, 650,
        6, 1520, 650,
    // 5
        -M_PI / 4,
        16, 1530, -650,
        20, 1390, -660,
        21, 1420, -650);

    ((TSpider*)module->Robot)->initPosition();

    return 0;
}

void shutdown(TModuleObject* module) {
    if (module->Robot != NULL) {
        delete (TSpider*)module->Robot;
        module->Robot = NULL;
    }

    stopMechanics();
}

int run_cmd(TModuleObject* module, char* cmd, char* arg) {
    if (cmd != NULL) {
        std::string command = (std::string)cmd;
        if (module->Robot != NULL) {
            TSpider* robot = (TSpider*)module->Robot;
            if (command == "go") {
                robot->TaskDistance = stod(arg);
                robot->Task = TSpider::ETask::MOVE_FORWARD;
                robot->move();
                return 0;
            }

            if (command == "walk") {
                robot->TaskDistance = stod(arg);
                robot->Task = TSpider::ETask::MOVE_FORWARD2;
                robot->move();
                return 0;
            }

            if (command == "turn") {
                robot->TurnAngle = stod(arg);
                robot->Task = TSpider::ETask::MOVE_TURN;
                robot->move();
                return 0;
            }

            if (command == "stop") {
                robot->Task = TSpider::ETask::STAND;
                robot->move();
                return 0;
            }

            if (command == "gmax") {
                robot->Ym = stod(arg);
                return 0;
            }

            if (command == "wmax") {
                robot->Ym2 = stod(arg);
                return 0;
            }

            if (command == "speed") {
                robot->Speed = stod(arg);
                return 0;
            }
            if (command == "Aup") {
                robot->Aup = stod(arg);
                robot->Bup = -robot->Aup;
                cout << "Aup/Bup = " << robot->Aup << endl;
                return 0;
            }
            if (command == "init") {
                robot->initPosition();
                return 0;
            }
            if (command == "up1") {
                robot->up1();
                return 0;
            }


        } else {
            std::cout << "module is not initialized" << std::endl;
        }
    } else {
        std::cout << "no cmd is passed" << std::endl;
    }
    return -1;
}
