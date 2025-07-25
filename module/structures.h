#ifndef _STRUCTURES_H
#define _STRUCTURES_H

#include "mechanics.h"

#include <string>
#include <iostream>
#include <cmath>


struct TPoint2D {
    // position of leg tip
    double X; // horizontal axix (looking from top), side 
    double Y; // vertical axix (looking from top), front-back

    TPoint2D() {}

    TPoint2D(TPoint2D& p) {
          X = p.X;
          Y = p.Y;
    }

    bool operator==(const TPoint2D& p) {
          return (X == p.X) && (Y == p.Y);
    }
};

struct TLeg  : public ICompletionListener, IProcess, TPoint2D {

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

    double As; // angle of bottom servo
    double Bs; // angle of middle servo
    double Cs; // angle of top servo

    double H; // hight

    // neutral position
    double XN;
    double YN;

    ICompletionListener* Complete;

    TLeg(std::string id, 
            int pinBottom, int zeroBottom, double rangeBottom,
            int pinMiddle, int zeroMiddle, double rangeMiddle,
            int pinTop, int zeroTop, double rangeTop) :
        IProcess(id),
        BottomServo(id + "-bottom", pinBottom, zeroBottom, rangeBottom),
        MiddleServo(id + "-middle", pinMiddle, zeroMiddle, rangeMiddle),
        TopServo(id + "-top", pinTop, zeroTop, rangeTop) {

        Complete = NULL;

        X = TLeg::A + TLeg::C;
        Y = 0;
        H = 0;
    }

    void complete(IProcess* src);

    bool countabc(double& a, double& b, double& c);
    bool countABC();
    void setHXY();
    void setAngles();
    void setAngles(double speed, ICompletionListener* complete);
    void setAngles(double speed, int maxDistance, ICompletionListener* complete);
    int getMaxTargetDistance();

    void setMove(double dx, double dy) {
          X = XN + dx;
          Y = YN + dy;
    }
};

struct TSkeleton2 : public ICompletionListener, IProcess {
     
     // Frame size, distance between edge legs vertical axies
     static constexpr double FW = 220;
     static constexpr double FH = 175;

     // H legs to be down
     static constexpr double Hdown = 20;

     // min D
     static constexpr double D0 =  + TLeg::C + 60;

     // Walking H
     // min H
     // base (walking) H (60..199)
     //H0 = sqrt(Leg0.B * Leg0.B - D0*D0) - Leg0.A;
     static constexpr double H0 = 100;

     // Neutral X for edge legs
     double XN;

     // neutral X for middle
     double XM;

     // step size
     double S;

     // neutral Y for edge
     double Y;

      // X for down position
     double XD;

     // Turn Radius of edge legs in neutral
     double RE;

     // Turn Radius of middle legs in neutral
     double RM;

     // Angle from center to tip of edge leg in neutral
     double AN;

     // max turn in degrees per step
     double Tmax;

     TLeg Leg0;
     TLeg Leg1;
     TLeg Leg2;
     TLeg Leg3;
     TLeg Leg4;
     TLeg Leg5;

     ICompletionListener* Complete;

     TSkeleton2(std::string id,

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
            
            Leg0(id + "-L0",
                 pinBottom0, zeroBottom0, rangeBottom0,
                 pinMiddle0, zeroMiddle0, rangeMiddle0,
                 pinTop0,    zeroTop0,    rangeTop0),
            
            Leg1(id + "-L1", 
                 pinBottom1, zeroBottom1, rangeBottom1,
                 pinMiddle1, zeroMiddle1, rangeMiddle1,
                 pinTop1,    zeroTop1,    rangeTop1),
            
            Leg2(id + "-L2",
                 pinBottom2, zeroBottom2, rangeBottom2,
                 pinMiddle2, zeroMiddle2, rangeMiddle2,
                 pinTop2,    zeroTop2,    rangeTop2),
            
            Leg3(id + "-L3",
                 pinBottom3, zeroBottom3, rangeBottom3,
                 pinMiddle3, zeroMiddle3, rangeMiddle3,
                 pinTop3,    zeroTop3,    rangeTop3),
            
            Leg4(id + "-L4",
                 pinBottom4, zeroBottom4, rangeBottom4,
                 pinMiddle4, zeroMiddle4, rangeMiddle4,
                 pinTop4,    zeroTop4,    rangeTop4),
            
            Leg5(id + "-L5",
                 pinBottom5, zeroBottom5, rangeBottom5,
                 pinMiddle5, zeroMiddle5, rangeMiddle5,
                 pinTop5,    zeroTop5,    rangeTop5),
                 
            Complete(NULL) {

          countInitParams();
          
          TTimer::getTimer()->start(20000);
     }
     virtual ~TSkeleton2() {
          TTimer::getTimer()->stop();
     }

     void complete(IProcess* src);
     void countInitParams();
     void setAngles();
     void setAngles(double speed, ICompletionListener* complete);
     bool countTargets();
};

// Actually Step model
struct IMoveModel : public ICompletionListener, IProcess {

     const int STATUS_COMPLETE = 0;
     const int STATUS_INCOMPLETE = 1;
     const int STATUS_ERROR_UNREACHABLE = -1;
     
     TSkeleton2* Skeleton;
     ICompletionListener* Complete;
     int Status;
     double Speed;
     double StepSpeed;

     IMoveModel(TSkeleton2* skeleton) :
          IProcess("move"),
          Skeleton(skeleton),
          Complete(NULL),
          Status(STATUS_COMPLETE),
          Speed(1) {}

     virtual void toNeutral(ICompletionListener* complete = NULL)=0;
     virtual bool setTargetsForLegs()=0;
     void move();
     void complete(IProcess* src);
};

struct IStepModel : public IMoveModel {

     struct TSteppingLeg : public TPoint2D {
          TLeg* Leg;
          const uint8_t Group;
     
          TSteppingLeg(TLeg& leg, uint8_t group) :
               Leg(&leg),
               Group(group) {}

          void step(bool& stateAchieved, bool& movement, bool* liftedGroups, int numGroups);
          void checkLifted(bool* liftedGroups);
          void keep() {
               keepX();
               keepY();
          }

          void keepX() {
               X = Leg->X;
          }

          void keepY() {
               Y = Leg->Y;
          }

          void setStep(double dx, double dy) {
               X = Leg->XN + dx;
               Y = Leg->YN + dy;
          }

     };

     static constexpr int GROUPS[4][6] = {{0, 0, 0, 0, 0, 0},
                                          {0, 0, 0, 0, 0, 0},
                                          {0, 1, 0, 1, 0, 1},
                                          {0, 1, 2, 2, 1, 0}};

     int NumGroups;

     TSteppingLeg Leg0;
     TSteppingLeg Leg1;
     TSteppingLeg Leg2;
     TSteppingLeg Leg3;
     TSteppingLeg Leg4;
     TSteppingLeg Leg5;

     IStepModel(TSkeleton2* skeleton, int numGroups) :
          IMoveModel(skeleton),
          NumGroups(numGroups),
          Leg0(skeleton->Leg0, GROUPS[numGroups][0]),
          Leg1(skeleton->Leg1, GROUPS[numGroups][1]),
          Leg2(skeleton->Leg2, GROUPS[numGroups][2]),
          Leg3(skeleton->Leg3, GROUPS[numGroups][3]),
          Leg4(skeleton->Leg4, GROUPS[numGroups][4]),
          Leg5(skeleton->Leg5, GROUPS[numGroups][5]) {}

     bool step(bool& movemnt);
     bool stepToNeutral();
};

struct TMoveDownModel : public IStepModel {
     
     enum EState {
          STATE_NEUTRAL = 0,
          STATE_UP_WIDE,
          STATE_DOWN
     };

     EState CurrentState;
     EState TargetState;

     TMoveDownModel(TSkeleton2* skeleton) :
          IStepModel(skeleton, 3) {}
     
     bool setTargetsForLegs();
     void setState(EState newState);

     void initPosition();

     void toNeutral(ICompletionListener* complete = NULL);
     void toDown(ICompletionListener* complete = NULL);
};

struct TMoveForwardModel : public IStepModel {

     double State;
     double LeftDistance;
     bool StopNeutral;

     TMoveForwardModel(TSkeleton2* skeleton) :
          IStepModel(skeleton, 2),
          State(0),
          LeftDistance(0),
          StopNeutral(true) {}

     bool groundedGroup1(double newState);
     bool groundedGroup2(double newState);
     bool setTargetsForLegs();
     void toNeutral(ICompletionListener* complete = NULL);
     void moveForward(double distance, ICompletionListener* complete = NULL);
};

struct TMoveDirModel : public IStepModel {

     double State;
     double LeftDistance;
     bool StopNeutral;
     
     double Dir;
     bool ResetState;

     TMoveDirModel(TSkeleton2* skeleton) :
          IStepModel(skeleton, 2),
          State(0),
          LeftDistance(0),
          StopNeutral(true) {}

     bool groundedGroup1(double newState);
     bool groundedGroup2(double newState);
     bool setTargetsForLegs();
     void toNeutral(ICompletionListener* complete = NULL);
     void moveDir(double distance, double direction, ICompletionListener* complete = NULL);
};

struct TTurnModel : public IStepModel {
     double State;
     double LeftAngle;
     bool StopNeutral;

     TTurnModel(TSkeleton2* skeleton) :
          IStepModel(skeleton, 2),
          State(0),
          LeftAngle(0),
          StopNeutral(true) {}

     bool groundedGroup1(double newState);
     bool groundedGroup2(double newState);
     bool setTargetsForLegs();
     void toNeutral(ICompletionListener* complete = NULL);
     void turnAngle(double angle, ICompletionListener* complete = NULL);
};

struct TMove2 : public ICompletionListener {

     enum ECommand {
          CMD_DOWN,
          CMD_FORWARD,
          CMD_TURN,
          CMD_DIR
     };
     
     TMoveDownModel MoveDownModel;
     TMoveForwardModel MoveForwardModel;
     double Distance;

     TTurnModel TurnModel;
     double Angle;

     TMoveDirModel MoveDirModel;

     ECommand Command;

     IMoveModel* CurrentModel;

     TMove2(TSkeleton2* skeleton) :
          MoveDownModel(skeleton),
          MoveForwardModel(skeleton),
          TurnModel(skeleton),
          MoveDirModel(skeleton)  {}

     void move();
     void complete(IProcess* src);
     void initPosition();
     void setSpeed(double speed);
     
     void toNeutral();
     void toDown();
     void moveForward(double distance);
     void turn(double angle);
     void moveDir(double distance, double direction);

     virtual ~TMove2() {}
};

#endif
