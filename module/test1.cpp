#include "mechanics.h"
#include "structures.h"
#include "module.h"
#include "camera.h"

#include <iostream>
#include <unistd.h>

#include <cmath>

using namespace std;


/*

leg0:
    (0)  Bottom: 14  0: 1520, 90: 870 (-650)
    (1)  Middle: 15  0: 1400, 90: 730, -90: 2070(-670)
    (2)  Top:    18  0: 1460, 90: 2130, -90: 790 (+670)

leg1:
    (3)  Bottom: 2  0: 1470, 90: 2130 (+660)
    (4)  Middle: 3  0: 1370, 90: 2020, -90: 720  (+650)
    (5)  Top:    4  0: 1500, 90: 840, -90: 2160 (-660)

leg2:
    (6)  Bottom: 22  0: 1470, 90: 2140 (+670)
    (7)  Middle: 27  0: 1430, 90: 2090, -90: 770 (+660)
    (8)  Top:    17  0: 1570, 90: 910, -90:2230  (-660)

leg3:
    (9)  Bottom: 13  0: 1550, 90: 900 (-650)
    (10) Middle: 19  0: 1550, 90: 2200, -90: 900 (+650)
    (11) Top:    26  0: 1460, 90: 2110, -90: 810 (+650)

leg4:
    (12) Bottom: 0   0: 1410  90: 760  (-650)
    (13) Middle: 5   0: 1520, 90: 2170, -90: 870 (+650)
    (14) Top:    6   0: 1520, 90: 2170, -90: 870 (+650)

leg5:
    (15) Bottom: 16   0: 1530, 90: 2180 (650)
    (16) Middle: 20   0: 1390, 90: 730, -90: 2050 (-660)
    (17) Top:    21   0: 1420, 90: 770, -90: 2070 (-650)
*/


void test1() {

    TServo** servos = new TServo*[18];
    // 0
    servos[0] = new TServo("bottom-0", 14, 1520, 650);
    servos[1] = new TServo("middle-0", 15, 1400, 670);
    servos[2] = new TServo("top-0", 18, 1460, -670);
    // 1
    servos[3] = new TServo("bottom-1", 2, 1470, -675);
    servos[4] = new TServo("middle-1", 3, 1370, -650);
    servos[5] = new TServo("top-1", 4, 1500, -660);
    // 2
    servos[6] = new TServo("bottom-2", 22, 1440, -670);
    servos[7] = new TServo("middle-2", 27, 1430, -660);
    servos[8] = new TServo("top-2", 17, 1570, -660);
    // 3
    servos[9] = new TServo("bottom-3", 0, 1410, 650);
    servos[10] = new TServo("middle-3", 5, 1540, 650);
    servos[11] = new TServo("top-3", 6, 1520, 650);
    // 4
    servos[12] = new TServo("bottom-4", 13, 1550, 670);
    servos[13] = new TServo("middle-4", 19, 1550, 650);
    servos[14] = new TServo("top-4", 26, 1460, 650);
    // 5
    servos[15] = new TServo("bottom-5", 16, 1560, -650);
    servos[16] = new TServo("middle-5", 20, 1390, -660);
    servos[17] = new TServo("top-5", 21, 1420, 650);

    TTimer::getTimer()->start(20000);
        
    int servo = 0;
    double speed = TOP_SPEED;
    
    while(true) {
        string cmd;
        cin >> cmd;
        if (cmd.length() == 0) {
            continue;
        }

        if (cmd == "exit") {
            return;
        }
        
        if (cmd == "o") {
            for (int i = 0; i < 6; ++i) {
                servos[i*3]->setTargetAngle(-30, speed, NULL);
                servos[i*3+1]->setTargetAngle(45, speed, NULL);
                servos[i*3+2]->setAngle(0);
            }
            continue;
        }
        
        if (cmd == "f") {
            for (int i = 0; i < 6; ++i) {
                servos[i*3+2]->setAngle(20);
            }
            continue;
        }

        if (cmd == "ot") {
            for (int i = 0; i < 18; ++i) {
                servos[i]->setTargetAngle(0, speed, NULL);
            }
            continue;
        }
        
        if (cmd == "ut") {
            for (int i = 0; i < 6; ++i) {
                servos[i*3]->setTargetAngle(45, speed, NULL);
                servos[i*3+1]->setTargetAngle(45, speed, NULL);
            }
            continue;
        }
        if (cmd == "ft") {
            for (int i = 0; i < 6; ++i) {
                servos[i*3+2]->setTargetAngle(20, speed, NULL);
            }
            continue;
        }

        char c = cmd[0];
        if (isdigit(c)) {
            int value = stoi(cmd);
            //servo.setValue(value);
            //cout << "(" << servo.Pin << ") = " << value << endl;
            servos[servo]->setValueAndTarget(value);
            cout << "value(" << servo << ") = " << value << endl;
            continue;
        }
        int value = stoi(cmd.substr(1));
        switch (c) {
        case 's':
            //servo.Pin = stoi(cmd.substr(1));
            //cout << "servo = " << servo.Pin << endl;
            servo = value;
            cout << "servo = " << servo << endl;
            break;
        case 'a':
            servos[servo]->setAngle(value);
            cout << "angle(" << servo << ") = " << value << endl;
            break;
        case 'z':
            servos[servo]->Zero = value;
            cout << "Zero(" << servo << ") = " << value << endl;
            break;
        case 'r':
            servos[servo]->Range = value / 90.0;
            cout << "Range(" << servo << ") = " << value << endl;
            break;
        case 'b': {
            for (int i = 0; i < 6; ++i) {
                servos[i*3]->setAngle(value);
            }
            break;
        }
        case 'm': {
            for (int i = 0; i < 6; ++i) {
                servos[i*3+1]->setAngle(value);
            }
            break;
        }
        case 't': {
            for (int i = 0; i < 6; ++i) {
                servos[i*3+2]->setAngle(value);
            }
            break;
        }
        case 'v':
            speed = value / 1000.0;
            break;
        }
    }
}

void test2() {
/*
    TLeg leg2("leg2", 0, 87, 0,
              22, 1470, -670,
              27, 1430, -660,
              17, 1570, -660);
*/
    TLeg** legs = new TLeg*[6];

    legs[0] = new TLeg( "leg0",
                        14, 1520, 650,
                        15, 1400, 670,
                        18, 1460, -670);
    
    legs[1] = new TLeg( "leg1",
                        2, 1470, -675,
                        3, 1370, -650,
                        4, 1500, -660);
    
    legs[2] = new TLeg( "leg2",
                        22, 1440, -670,
                        27, 1430, -660,
                        17, 1570, -660);
    
    legs[3] = new TLeg( "leg3",
                        0, 1410, 650,
                        5, 1540, 650,
                        6, 1520, 650);
    
    legs[4] = new TLeg( "leg4",
                        13, 1550, 670,
                        19, 1550, 650,
                        26, 1460, 650);

    legs[5] = new TLeg( "leg5",
                        16, 1560, -650,
                        20, 1390, -660,
                        21, 1420, 650);

    TTimer::getTimer()->start(20000);
    double speed = TOP_SPEED;
    int    leg = 0;

    while(true) {
        string cmd;
        cin >> cmd;
        if (cmd.length() == 0) {
            continue;
        }

        if (cmd == "exit") {
            break;
        }

        if (cmd == "o") {
            for (int i = 0; i < 6; ++i) {
               legs[i]->H = 0;
               legs[i]->X = TLeg::A + TLeg::B + TLeg::C;
               legs[i]->setHXY();

            }
            continue;
        }
        if (cmd == "ot") {
            for (int i = 0; i < 6; ++i) {
                legs[i]->BottomServo.setTargetAngle(TLeg::Bcorr, speed, NULL);
                legs[i]->MiddleServo.setTargetAngle(0, speed, NULL);
                legs[i]->TopServo.setTargetAngle(0, speed, NULL);
            }
            continue;
        }
        
        if (cmd == "ut") {
            for (int i = 0; i < 6; ++i) {
                legs[i]->BottomServo.setTargetAngle(45, speed, NULL);
                legs[i]->MiddleServo.setTargetAngle(45, speed, NULL);
            }
            continue;
        }

        char c = cmd[0];
        int value = stoi(cmd.substr(1));
        switch (c) {
        case 'l':
            leg = value;
            if (leg >= 0) {
                cout << leg << ": H = " << legs[leg]->H << " X = " << legs[leg]->X << " Y = " << legs[leg]->Y <<endl;
            } else {
                cout << "all legs:" << endl;
            }
            break;

        case 'h':
            if (leg>= 0) {
                legs[leg]->H = value;
                legs[leg]->setHXY();
            } else {
                for (int i = 0; i < 6; ++i) {
                    legs[i]->H = value;
                    legs[i]->setHXY();
                }
            }
            break;
        case 'x':
            if (leg >= 0) {
                legs[leg]->X = value;
                legs[leg]->setHXY();
            } else {
               for (int i = 0; i < 6; ++i) {
                    legs[i]->X = value;
                    legs[i]->setHXY();
                }                
            }
            break;
        case 'y':
            if (leg >= 0) {
                legs[leg]->Y = value;
                legs[leg]->setHXY();
            } else {
               for (int i = 0; i < 6; ++i) {
                    legs[i]->Y = value;
                    legs[i]->setHXY();
                }
            }
            break;
        case 'v':
            speed = value / 1000.0;
            break;
        }
    }

    TTimer::getTimer()->stop();

}

void test5() {
    TSkeleton2 skeleton("spider",
    // 0
                        14, 1520, 650,
                        15, 1400, 670,
                        18, 1460, -670,
    
    // 1
                        2, 1470, -675,
                        3, 1370, -650,
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

    TMove2 move(&skeleton);
    move.initPosition();
    move.setSpeed(0.05);

    while(true) {
        string cmd;
        cout << "CMD:";
        cin >> cmd;
        if (cmd.length() == 0) {
            continue;
        }

        if (cmd == "exit") {
            break;
        }
        
        if (cmd == "neutral") {
            cout << "---------------------------------------------------" << endl;
            move.toNeutral();
            continue;
        }

        if (cmd == "down") {
            cout << "---------------------------------------------------" << endl;
            move.toDown();
            continue;
        }

        if (cmd == "forward") {
            double distance;
            cout << "distance:";
            cin >> distance;
            cout << "---------------------------------------------------" << endl;
            move.moveForward(distance);
            continue;
        }

        if (cmd == "dir") {
            double distance;
            double direction;
            cout << "direction:";
            cin >> direction;
            cout << "distance:";
            cin >> distance;
            /*
            string arg;
            cout << "distance,direction:";
            cin >> arg;

            const char* carg = arg.c_str();

            int comma = -1;
            int end = 0;
            while(carg[++end] != 0) {
                if (carg[end] == ',') {
                    comma = end;
                }
            }

            if (comma >= 0) {
                string dist(carg, carg + comma);
                string dir(carg + comma + 1, end);
                cout << "dist=" << dist << " dir=" << dir << endl;
            }
            */

            cout << "---------------------------------------------------" << endl;
            move.moveDir(distance, direction * M_PI / 180);
            continue;
        }

        if (cmd == "turn") {
            double angle;
            cout << "angle:";
            cin >> angle;
            cout << "---------------------------------------------------" << endl;
            move.turn(angle * M_PI / 180);
            continue;
        }

        if (cmd == "speed") {
            double speed;
            cout << "[0..1000]:";
            cin >> speed;
            move.setSpeed(speed / 1000.0);
            continue;
        }

        if (cmd == "pic") {
            //makePictures("depth.jpg", "color.jpg");
            makePictures2("depth.jpg", "color.jpg", "ir.jpg");
            continue;
        }
    }
}

void test6() {
    TModuleObject module;
    init(&module);
    string cmd;
    string arg;
    do {
        string cmdarg;
        cout << "CMD:";
        cin >> cmd >> arg;
        cout << "cmd=" << cmd << " arg=" << arg << endl;

    } while(run_cmd(&module, cmd.c_str(), arg.c_str()) >= -1);

    cout << "Exited" << endl;
}

void test7() {
    TSkeleton2 skeleton("spider",
    // 0
                        14, 1520, 650,
                        15, 1400, 670,
                        18, 1460, -670,
    
    // 1
                        2, 1470, -675,
                        3, 1370, -650,
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

    
    TLeg* legs[] = {&skeleton.Leg0, &skeleton.Leg1, &skeleton.Leg2, &skeleton.Leg3, &skeleton.Leg4, &skeleton.Leg5, };
    
    int leg = 0;
    string op = "top";

    while(true) {
        string cmd;
        cout << "(" << leg << ")" << op << ":";
        cin >> cmd;
        if (cmd.length() == 0) {
            continue;
        }

        if (cmd == "exit") {
            break;
        }

        if (cmd == "leg") {
            cout << "leg:";
            cin >> leg;
            continue;
        }

        if (cmd == "top") {
            op = "top";
            continue;
        }

        if (cmd == "middle") {
            op = "middle";
            continue;
        }

        if (cmd == "bottom") {
            op = "bottom";
            continue;
        }

        const char* begin = cmd.c_str();
        char* end = (char*)begin;
        double v = strtod(begin, &end);
        if (begin != end) {
            if (op == "top") {
                legs[leg]->TopServo.setAngle(v);
                continue;
            }
            if (op == "middle") {
                legs[leg]->MiddleServo.setAngle(v);
                continue;
            }
            if (op == "bottom") {
                legs[leg]->BottomServo.setAngle(v);
                continue;
            }
        }
    };

}

int main(int argc, char *argv[]) {
	
    cout << "start" << endl;

    initMechanics();
	//test1();
    //test2();
    test5();
    //test7();
    cout << "stop" << endl;

    stopMechanics();


    //test6();

    cout << "exit" << endl;

}
