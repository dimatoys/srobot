#ifndef _STRUCTURES_H
#define _STRUCTURES_H

#include "mechanics.h"

#include <string>
#include <iostream>

struct TLeg  : public ICompletionListener, IProcess {

    static constexpr double A = 75;   // distance between bottom and middle sevos axis
    static constexpr double B = 163;  // distance from tip to bottom servo axis
    static constexpr double C = 63;   // distance between top and middle servos axis
    static constexpr double Bcorr = -7; // correction of anble b in degrees
    static constexpr double AABB = A * A + B * B;
    static constexpr double AAMBB = A * A - B * B;
    static constexpr double AB2 = A * B * 2;
    static constexpr double B2 = B * B;
    
    TServo BottomServo;
    TServo MiddleServo;
    TServo TopServo;

    // location of top motor axis
    double GX;  // horizontal (looking from top) coordonate of top servo axis
    double GY;  // vertical (looking  from top) coordonate of top servo axis

    double As; // angle of bottom servo
    double Bs; // angle of middle servo
    double Cs; // angle of top servo

    // position of leg tip
    double X; // horizontal axix (looking from top), side 
    double Y; // vertical axix (looking from top), front-back
    double H; // hight

    ICompletionListener* Complete;

    TLeg(std::string id, double gx, double gy,
            int pinBottom, int zeroBottom, double rangeBottom,
            int pinMiddle, int zeroMiddle, double rangeMiddle,
            int pinTop, int zeroTop, double rangeTop) :
        IProcess(id),
        BottomServo(id + "-bottom", pinBottom, zeroBottom, rangeBottom),
        MiddleServo(id + "-middle", pinMiddle, zeroMiddle, rangeMiddle),
        TopServo(id + "-top", pinTop, zeroTop, rangeTop),
        GX(gx),
        GY(gy) {

        Complete = NULL;

        H = TLeg::B;
        X = TLeg::A + TLeg::C;
        Y = 0;
    }

    void complete(IProcess* src);

    bool countabc(double& a, double& b, double& c);
    bool countABC();
    void setHXY();
    void setAngles();
    void setAngles(double speed, ICompletionListener* complete);
    void setAngles(double speed, int maxDistance, ICompletionListener* complete);
    int getMaxTargetDistance();
    bool countTurn(double angle);
};

struct TSkeleton : public ICompletionListener, IProcess {

     //static constexpr double FW = 
     //static constexpr double FH = 

     // H legs to be down
     static constexpr double Hdown = 20;
     static constexpr double Hdelta = 50;

     // min D
     static constexpr double D0 = 60;


     // base H (60..199)
     double H0;

     // Neutral X for edge legs
     double XN;
 
     // H group 1
     double H1;

     // H group 2
     double H2;

     // neutral X for edge, group 1
     double X1;

     // neutral X for edge, group 2
     double X2;

     // neutral X for middle group 1
     double XM1;

     // neutral X for middle group 2
     double XM2;

     // neutral Y for edge
     double Y;

     // front shift, group 1 and 2 
     double DY1;
     double DY2;

     // side shift
     double DX;

     // step size
     double S;

     // Speed
     double Speed;
    
     TLeg Leg0;
     TLeg Leg1;
     TLeg Leg2;
     TLeg Leg3;
     TLeg Leg4;
     TLeg Leg5;

     ICompletionListener* Complete;

     TSkeleton(std::string id,

            int pinBottom0, int zeroBottom0, double rangeBottom0,
            int pinMiddle0, int zeroMiddle0, double rangeMiddle0,
            int pinTop0,    int zeroTop0,    double rangeTop0,
            
            int pinBottom1, int zeroBottom1, double rangeBottom1,
            int pinMiddle1, int zeroMiddle1, double rangeMiddle1,
            int pinTop1,    int zeroTop1,    double rangeTop1,

            int pinBottom2, int zeroBottom2, double rangeBottom2,
            int pinMiddle2, int zeroMiddle2, double rangeMiddle2,
            int pinTop2,    int zeroTop2,    double rangeTop2,

            int pinBottom3, int zeroBottom3, double rangeBottom3,
            int pinMiddle3, int zeroMiddle3, double rangeMiddle3,
            int pinTop3,    int zeroTop3,    double rangeTop3,

            int pinBottom4, int zeroBottom4, double rangeBottom4,
            int pinMiddle4, int zeroMiddle4, double rangeMiddle4,
            int pinTop4,    int zeroTop4,    double rangeTop4,

            int pinBottom5, int zeroBottom5, double rangeBottom5,
            int pinMiddle5, int zeroMiddle5, double rangeMiddle5,
            int pinTop5,    int zeroTop5,    double rangeTop5) :
            
            IProcess(id),

            Leg0(id + "-L0", 0, 0,
                 pinBottom0, zeroBottom0, rangeBottom0,
                 pinMiddle0, zeroMiddle0, rangeMiddle0,
                 pinTop0,    zeroTop0,    rangeTop0),
            
            Leg1(id + "-L1", 0, 0,
                 pinBottom1, zeroBottom1, rangeBottom1,
                 pinMiddle1, zeroMiddle1, rangeMiddle1,
                 pinTop1,    zeroTop1,    rangeTop1),
            
            Leg2(id + "-L2", 0, 0,
                 pinBottom2, zeroBottom2, rangeBottom2,
                 pinMiddle2, zeroMiddle2, rangeMiddle2,
                 pinTop2,    zeroTop2,    rangeTop2),
            
            Leg3(id + "-L3", 0, 0,
                 pinBottom3, zeroBottom3, rangeBottom3,
                 pinMiddle3, zeroMiddle3, rangeMiddle3,
                 pinTop3,    zeroTop3,    rangeTop3),
            
            Leg4(id + "-L4", 0, 0,
                 pinBottom4, zeroBottom4, rangeBottom4,
                 pinMiddle4, zeroMiddle4, rangeMiddle4,
                 pinTop4,    zeroTop4,    rangeTop4),
            
            Leg5(id + "-L5", 0, 0,
                 pinBottom5, zeroBottom5, rangeBottom5,
                 pinMiddle5, zeroMiddle5, rangeMiddle5,
                 pinTop5,    zeroTop5,    rangeTop5),
                 
            Complete(NULL) {

          TTimer::getTimer()->start(20000);
          Speed = TOP_SPEED;

          countParams();
     }
     virtual ~TSkeleton() {
          TTimer::getTimer()->stop();
     }

     void complete(IProcess* src);

     void initPosition();
     bool count();
     void countParams();
     void setAngles();
     void setAngles(ICompletionListener* complete);
};

struct TMove : public ICompletionListener {
     TSkeleton* Skeleton;

     double A;
     double B;
     double C;

     // H group 1
     double H1;

     // H group 2
     double H2;

     // neutral X for edge, group 1
     double X1;

     // neutral X for edge, group 2
     double X2;

     // neutral X for middle, group 1
     double XM1;

     // neutral X for middle, group 2
     double XM2;

     // neutral Y
     double Y;

     // front shift, group 1 and 2 
     double DY1;
     double DY2;

     // side shift
     double DX;

     // Target Y;
     double TY;

     TMove(TSkeleton* skeleton);
     
     void complete(IProcess* src);
     bool run();

     bool move();
     bool moveXDYStayOn1();
     bool moveXDYStayOn2();
     bool liftGroup1();
     bool liftGroup2();

     double getMinHforXY(double X, double Y);

     bool toNeutral();
     bool toDown();
     bool moveForward(double distance);
};

struct TSpider : public ICompletionListener, IProcess {

     enum EMovementState {
          INIT = 0,
          UP1,
          FORWARD_UP1,
          FORWARD_DOWN1,
          BACK_UP2,
          UP2,
          FORWARD_UP2,
          FORWARD_DOWN2,
          BACK_UP1,

          BACK2,
          BACK_DOWN2,
          DOWN1,
          BACK1,
          BACK_DOWN1,
          DOWN2,

          TURN_UP1,   // group1 up and turned forward, group2 down and turned back   <- turn1
          TURN_DOWN1, // group1 down and turned forward, group2 down and turned back <- down1
          TURN2,      // group1 down and turned forward, group2 up and turned back   <- up2
          TURN_UP2,   // group1 down and turned back, group2 up and turned forward   <- turn2
          TURN_DOWN2, // group1 down and turned back, group2 down and turned forward <- down2
          TURN1       // group1 up and turned back, group2 down and turned forward   <- up1
     };

     enum ETask {
          TOINIT = 0,
          STAND,
          MOVE_FORWARD,
          MOVE_TURN,
          MOVE_FORWARD2
     };

     double H0;   // normal H
     double X0;  // normal X for side legs
     double Y0;    // normal Y for size legs
     double X1;  // normal X for angle legs: X0 / sqrt(2)
     double Y1;  // normal Y for angle legs: X0 / sqrt(2)
     double Aup;   // uplift for A
     double Bup;   // uulift for B
     double Ym;   // Y forward for middle leg
     double Ym2;   // Y forward2 for middle leg
     double Speed; // Speed
     double Tm;    // max turn angle per step

     TLeg Leg0;
     TLeg Leg1;
     TLeg Leg2;
     TLeg Leg3;
     TLeg Leg4;
     TLeg Leg5;

     EMovementState MovementState;
     ETask          Task;
     double         TaskDistance;
     double         TurnAngle;

     TSpider(std::string id,

            double Wm, // 174 # between middle
            double Wk, // 147 # between angles
            double Hk, // 206 # between angles

            double angle0,
            int pinBottom0, int zeroBottom0, double rangeBottom0,
            int pinMiddle0, int zeroMiddle0, double rangeMiddle0,
            int pinTop0,    int zeroTop0,    double rangeTop0,
            
            double angle1,
            int pinBottom1, int zeroBottom1, double rangeBottom1,
            int pinMiddle1, int zeroMiddle1, double rangeMiddle1,
            int pinTop1,    int zeroTop1,    double rangeTop1,

            double angle2,
            int pinBottom2, int zeroBottom2, double rangeBottom2,
            int pinMiddle2, int zeroMiddle2, double rangeMiddle2,
            int pinTop2,    int zeroTop2,    double rangeTop2,

            double angle3,
            int pinBottom3, int zeroBottom3, double rangeBottom3,
            int pinMiddle3, int zeroMiddle3, double rangeMiddle3,
            int pinTop3,    int zeroTop3,    double rangeTop3,

            double angle4,
            int pinBottom4, int zeroBottom4, double rangeBottom4,
            int pinMiddle4, int zeroMiddle4, double rangeMiddle4,
            int pinTop4,    int zeroTop4,    double rangeTop4,

            double angle5,
            int pinBottom5, int zeroBottom5, double rangeBottom5,
            int pinMiddle5, int zeroMiddle5, double rangeMiddle5,
            int pinTop5,    int zeroTop5,    double rangeTop5) :
            
            IProcess(id),

            Leg0(id + "-L0", Wk / 2, Hk / 2,
                 pinBottom0, zeroBottom0, rangeBottom0,
                 pinMiddle0, zeroMiddle0, rangeMiddle0,
                 pinTop0,    zeroTop0,    rangeTop0),
            
            Leg1(id + "-L1", Wk / 2, Hk / 2,
                 pinBottom1, zeroBottom1, rangeBottom1,
                 pinMiddle1, zeroMiddle1, rangeMiddle1,
                 pinTop1,    zeroTop1,    rangeTop1),
            
            Leg2(id + "-L2", Wm / 2, 0,
                 pinBottom2, zeroBottom2, rangeBottom2,
                 pinMiddle2, zeroMiddle2, rangeMiddle2,
                 pinTop2,    zeroTop2,    rangeTop2),
            
            Leg3(id + "-L3", Wm / 2, 0,
                 pinBottom3, zeroBottom3, rangeBottom3,
                 pinMiddle3, zeroMiddle3, rangeMiddle3,
                 pinTop3,    zeroTop3,    rangeTop3),
            
            Leg4(id + "-L4", Wk / 2, Hk / 2,
                 pinBottom4, zeroBottom4, rangeBottom4,
                 pinMiddle4, zeroMiddle4, rangeMiddle4,
                 pinTop4,    zeroTop4,    rangeTop4),
            
            Leg5(id + "-L5", Wk / 2, Hk / 2,
                 pinBottom5, zeroBottom5, rangeBottom5,
                 pinMiddle5, zeroMiddle5, rangeMiddle5,
                 pinTop5,    zeroTop5,    rangeTop5) {

          TTimer::getTimer()->start(20000);

          H0 = 120;   // normal H
          X0 = 150;  // normal X for side legs
          Y0 = 0;    // normal Y for size legs
          X1 = 106;  // normal X for angle legs: X0 / sqrt(2)
          Y1 = 106;  // normal Y for angle legs: X0 / sqrt(2)
          Aup = 20;
          Bup = -Aup;
          Ym  = 60;
          Ym2  = 40;
          Speed = TOP_SPEED;
          Tm = 15;

          MovementState = INIT;
          Task = STAND;
     }

     void setAngles1();
     void setAngles2();
     bool update1();
     bool update2();

     void initPosition();
     void up1();
     void down1();
     void up2();
     void down2();
     void neutral1();
     void neutral2();
     void forward1(double distance);
     void forward2(double distance);
     void turn1(double angle);
     void neutralTurn1();
     void neutralTurn2();

     void back1(double distance);
     void back2(double distance);

     void move();
     void moveToInit();
     void moveToStand();
     void moveForward();
     void moveTurn();
     void moveForward2();

     void stateMachineComplete();

    virtual ~TSpider() {
        TTimer::getTimer()->stop();
    }

    void complete(IProcess* src);
};

#endif
