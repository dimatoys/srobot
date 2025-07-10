#include "structures.h"
    
#include <iostream>
#include <unistd.h>

using namespace std;

void TLeg::complete(IProcess* src) {

    //cout << Id << " complete:" << src->Id << endl;

    if ((Complete != NULL) &&
        (BottomServo.Complete == NULL) && (MiddleServo.Complete == NULL) && (TopServo.Complete == NULL)) {

            auto complete = Complete;
            Complete = NULL;
            complete->complete(this);
    }
}

bool TLeg::countabc(double& a, double& b, double& c) {
    
    c = atan2(Y, X);
   
    // AB distance watching from top
    auto r = sqrt(X*X + Y*Y) - C;
    auto r2 = r * r;
    auto ha = H + A;
    auto bha2 = B2 - ha* ha;
    if (r2 <= bha2) {
        // unreachable area
        a = M_PI / 2;
        //b = M_PI / 2 - acos(ha / B);
        b = asin(ha / B);
        auto cr = sqrt(bha2) + C;
        auto cx = cr * cos(c);
        auto cy = cr * sin(c);
        cout << Id << " close area: cx=" << cx << " cy=" << cy << " cr=" << cr << " r=" << (r + C)  << endl;

    } else {

        // AB^2 distance
        auto g2 = r2 + H * H;
        auto sinb = (g2 - AABB) / AB2;
        if(fabs(sinb) <= 1) {
            b = -asin(sinb);
        
            auto g = sqrt(g2);
            auto a1 = acos((AAMBB + g2)/(2 * A * g));
            auto a2 = atan2(H, r);
            a = a1 - a2;
        } else {
            // this is impossible
            // check [x=207.72, y=265.932, h=20 r=274.443 sinb=1.78015] out of rage

            cout << Id <<" [x=" << X << ", y=" << Y << ", h=" << H << " r=" << r << " sinb=" << sinb << "] out of rage" << endl;
            return false;
        }
    }
    
    a = a * 180 / M_PI;
    b = b * 180 / M_PI + Bcorr;
    c = c * 180 / M_PI;

    cout << Id << " [x=" << X << ", y=" << Y << ", h=" << H << "] -> a=" << a << " b=" << b << " c=" << c << endl;
    return true;
}

bool TLeg::countABC() {
    return countabc(As, Bs, Cs);
}

void TLeg::setHXY() {
    countABC();
    setAngles();
}

void TLeg::setAngles() {
    BottomServo.setAngle(Bs);
    MiddleServo.setAngle(As);
    TopServo.setAngle(Cs);
}

void TLeg::setAngles(double speed, ICompletionListener* complete) {
    cout << Id << " [a=" << As << ", b=" << Bs << ", c=" << Cs << "]" << endl;
    Complete = complete;
    BottomServo.setTargetAngle(Bs, speed, this);
    MiddleServo.setTargetAngle(As, speed, this);
    TopServo.setTargetAngle(Cs, speed, this);
}

void TLeg::setAngles(double speed, int maxDistance, ICompletionListener* complete) {
    //cout << Id << " (" << As << ", " << Bs << ", " << Cs << ") <- ("
    //     << MiddleServo.getAngle() << ", " << BottomServo.getAngle() << ", " << TopServo.getAngle() << ") ["
    //     << MiddleServo.getTargetDistanceToAngle(As) << ", " << BottomServo.getTargetDistanceToAngle(Bs) << ", " << TopServo.getTargetDistanceToAngle(Cs) << "]"
    //     << " max=" << maxDistance << " speed=" << speed << endl;
    Complete = complete;
    BottomServo.setTargetAngle(Bs, speed * BottomServo.getTargetDistanceToAngle(Bs) / maxDistance, this);
    MiddleServo.setTargetAngle(As, speed * MiddleServo.getTargetDistanceToAngle(As) / maxDistance, this);
    TopServo.setTargetAngle(Cs, speed * TopServo.getTargetDistanceToAngle(Cs) / maxDistance, this);
}


int TLeg::getMaxTargetDistance() {
    auto Bd = BottomServo.getTargetDistanceToAngle(Bs);
    auto Ad = MiddleServo.getTargetDistanceToAngle(As);
    auto Cd = TopServo.getTargetDistanceToAngle(Cs);

    return Bd > Ad ? (Bd > Cd ? Bd : Cd) : (Ad > Cd ? Ad : Cd);
}

void TSkeleton2::complete(IProcess* src){
    if (Complete != NULL) {
        if ((Leg0.Complete == NULL) &&
            (Leg1.Complete == NULL) &&
            (Leg2.Complete == NULL) &&
            (Leg3.Complete == NULL) &&
            (Leg4.Complete == NULL) &&
            (Leg5.Complete == NULL)) {
                auto complete = Complete;
                Complete = NULL;
                complete->complete(this);
        }
    }
}

void TSkeleton2::countInitParams() {
    
    // H for top position of a leg
    // Hdown = 20

    // min D
    // D0 = 60
    
    // Walking H
    // base (walking) H (60..199)
    // H0 = sqrt(Leg0.B * Leg0.B - D0*D0) - Leg0.A;

    // Alpha
    // double A2 = 25 * M_PI / 180;
    
    // max distance leg can reach for H0
    double D = Leg0.C + sqrt((Leg0.A + Leg0.B) * (Leg0.A + Leg0.B) - H0 * H0) * 0.7;

    // new: step size
    S = D - D0;

    // Neutral X for edge legs
    // XN = D0 * cos(A2);
    // max Y for edge leg
    //double Ymax = sqrt(D*D - XN*XN);
    // min Y for edge leg
    //double Ymin = XN * tan(A2);
    // step size (max for edge leg, should be less or equal middle: 2 * D * sin(A2))
    //S = Ymax - Ymin - 100;
    // X for middle
    //XM = S / (2 * tan(A2));
    // Neutral Y for edge
    //Y = (Ymax + Ymin) / 2;

    S = D - D0;
    XM = S / 2 + D0;
    double A2 = asin(S / XM / 2);
    XN = XM * cos(A2 * 2);
    Y = XM * sin(A2 * 2);

    // X for down position
    XD = Leg0.C + sqrt(Leg0.B * Leg0.B - Leg0.A * Leg0.A);

    // coordinates of edge leg tip in neutral position relatevly to center of spider
    auto CX = FW / 2 + XN;
    auto CY = FH / 2 + Y;

     // Turn Radius of edge legs in neutral
    RE = sqrt(CX * CX + CY * CY);

     // Turn Radius of middle legs in neutral
    RM = FW / 2 + XN;

    // Angle from center to tip of edge leg in neutral
    AN = atan2(CY, CX);

    cout << "H0=" << H0 << "D =" << D << " S=" << S << " XN=" << XN << " XM=" << XM << " Y=" << Y
         << " RE=" << RE << " RM=" << RM << " AN=" << (AN * 180 / M_PI) << endl;
}

void TSkeleton2::setAngles() {
    Leg0.setAngles();
    Leg1.setAngles();
    Leg2.setAngles();
    Leg3.setAngles();
    Leg4.setAngles();
    Leg5.setAngles();
}

void TSkeleton2::setAngles(double speed, ICompletionListener* complete) {
    auto dmax = Leg0.getMaxTargetDistance();
    auto d  = Leg1.getMaxTargetDistance();
    if (d > dmax) { dmax = d; }
    d  = Leg2.getMaxTargetDistance();
    if (d > dmax) { dmax = d; }
    d  = Leg3.getMaxTargetDistance();
    if (d > dmax) { dmax = d; }
    d  = Leg4.getMaxTargetDistance();
    if (d > dmax) { dmax = d; }
    d  = Leg5.getMaxTargetDistance();
    if (d > dmax) { dmax = d; }

    Complete = complete;
    Leg0.setAngles(speed, dmax, this);
    Leg1.setAngles(speed, dmax, this);
    Leg2.setAngles(speed, dmax, this);
    Leg3.setAngles(speed, dmax, this);
    Leg4.setAngles(speed, dmax, this);
    Leg5.setAngles(speed, dmax, this);
}

bool TSkeleton2::countTargets() {
    return Leg0.countABC() &&
           Leg1.countABC() &&
           Leg2.countABC() &&
           Leg3.countABC() &&
           Leg4.countABC() &&
           Leg5.countABC();
}

void IMoveModel::complete(IProcess* src) {
    cout << "Complete" << endl;
/*
    string s;
    cout << "y/n:";
    cin >> s;
    if (s == "n") {
        return;
    }
*/
    move();
}

void IMoveModel::move() {
    Status = STATUS_INCOMPLETE;
    StepSpeed = Speed;
    if (setTargetsForLegs()) {
        if (Skeleton->countTargets()) {
            Skeleton->setAngles(StepSpeed, this);
            return;
        } else {
            cout << "Unreachable position" << endl;
            Status = STATUS_ERROR_UNREACHABLE;
        }
    } else {
        cout << "Target reached" << endl;
        Status = STATUS_COMPLETE;
    }

    if (Complete != NULL) {
        auto complete = Complete;
        Complete = NULL;
        complete->complete(this);
    }
}

void IStepModel::TSteppingLeg::step(bool& stateAchieved,
                                    bool& movement,
                                    bool* liftedGroups,
                                    int numGroups) {
    // check if leg is fully lifted
    if (Leg->H > TSkeleton2::Hdown) {
        // leg is not fully lifted
        if ((Leg->X == X) && (Leg->Y == Y)) {
            // leg on the right place
            // just make sure it is on the ground
            Leg->H = TSkeleton2::H0;
        } else {
            // leg is not on the right place
            // need to lift it first
            stateAchieved = false;
            // check if any other groups are already lifted
            bool otherLifted = false;
            for (int i = 0; i < numGroups; ++i) {
                if (i != Group) {
                    otherLifted = otherLifted || liftedGroups[i];
                }
            }
            if (!otherLifted) {
                // can lift, as the other group is on the ground
                // so lift it
                Leg->H = TSkeleton2::Hdown;
                liftedGroups[Group] = true;
            } // else cannot lift, as the other group is already lifted
        }
    } else {
        // leg is lifted
        // check if it is on the right place
        if ((Leg->X == X) && (Leg->Y == Y)) {
            // it is on the right place, just move it down
            Leg->H = TSkeleton2::H0;
        } else {
            // not on the right place, need to move
            stateAchieved = false;
            movement = true;
            Leg->X = X;
            Leg->Y = Y;
        }
    }
}

void IStepModel::TSteppingLeg::checkLifted(bool* liftedGroups) {
    liftedGroups[Group] = liftedGroups[Group] || (Leg->H < TSkeleton2::H0);
}

bool IStepModel::step(bool& movement) {

    bool stateAchieved = true;
    bool liftedGroups[] = {false, false, false, false, false, false};
    Leg0.checkLifted(liftedGroups);
    Leg1.checkLifted(liftedGroups);
    Leg2.checkLifted(liftedGroups);
    Leg3.checkLifted(liftedGroups);
    Leg4.checkLifted(liftedGroups);
    Leg5.checkLifted(liftedGroups);

    Leg0.step(stateAchieved, movement, liftedGroups, NumGroups);
    Leg1.step(stateAchieved, movement, liftedGroups, NumGroups);
    Leg2.step(stateAchieved, movement, liftedGroups, NumGroups);
    Leg3.step(stateAchieved, movement, liftedGroups, NumGroups);
    Leg4.step(stateAchieved, movement, liftedGroups, NumGroups);
    Leg5.step(stateAchieved, movement, liftedGroups, NumGroups);

    return stateAchieved;
}

bool IStepModel::stepToNeutral() {
    Leg0.X = Skeleton->XN;
    Leg0.Y = Skeleton->Y;
    Leg1.X = Skeleton->XM;
    Leg1.Y = 0;
    Leg2.X = Skeleton->XN;
    Leg2.Y = -Skeleton->Y;
    Leg3.X = Skeleton->XN;
    Leg3.Y = Skeleton->Y;
    Leg4.X = Skeleton->XM;
    Leg4.Y = 0;
    Leg5.X = Skeleton->XN;
    Leg5.Y = -Skeleton->Y;
    bool movement = false;
    return step(movement);
}

void TMoveDownModel::setState(EState newState) {

    bool stateUpdated = true;
    bool movement = false;
    
    switch(newState) {
        case STATE_DOWN:
            Skeleton->Leg0.H = Skeleton->Hdown;
            Skeleton->Leg1.H = Skeleton->Hdown;
            Skeleton->Leg2.H = Skeleton->Hdown;
            Skeleton->Leg3.H = Skeleton->Hdown;
            Skeleton->Leg4.H = Skeleton->Hdown;
            Skeleton->Leg5.H = Skeleton->Hdown;
            StepSpeed = 0.1;
            break;
        case STATE_UP_WIDE:
            Leg0.X = Skeleton->XD;
            Leg0.keepY();
            Leg1.X = Skeleton->XD;
            Leg1.keepY();
            Leg2.X = Skeleton->XD;
            Leg2.keepY();
            Leg3.X = Skeleton->XD;
            Leg3.keepY();
            Leg4.X = Skeleton->XD;
            Leg4.keepY();
            Leg5.X = Skeleton->XD;
            Leg5.keepY();
            stateUpdated = step(movement);
            break;
        case STATE_NEUTRAL:
            stateUpdated = stepToNeutral();
            break;
        default:
            cout << "Incorrect state: " << newState << endl;
            return;
    }
    if (stateUpdated) {
        CurrentState = newState;
    }
}

bool TMoveDownModel::setTargetsForLegs() {

    if (CurrentState == TargetState) {
        return false;
    }
    
    if (CurrentState < TargetState) {
        setState(static_cast<EState>(static_cast<int>(CurrentState) + 1));
    } else {
        setState(static_cast<EState>(static_cast<int>(CurrentState) - 1));
    }

    return true;
}

void TMoveDownModel::initPosition() {

    Skeleton->Leg0.X = Skeleton->XD;
    Skeleton->Leg1.X = Skeleton->XD;
    Skeleton->Leg2.X = Skeleton->XD;
    Skeleton->Leg3.X = Skeleton->XD;
    Skeleton->Leg4.X = Skeleton->XD;
    Skeleton->Leg5.X = Skeleton->XD;

    Skeleton->Leg0.Y = Skeleton->Y;
    Skeleton->Leg1.Y = 0;
    Skeleton->Leg2.Y = -Skeleton->Y;
    Skeleton->Leg3.Y = Skeleton->Y;
    Skeleton->Leg4.Y = 0;
    Skeleton->Leg5.Y = -Skeleton->Y;

    setState(STATE_DOWN);

    if (Skeleton->countTargets()) {
        Skeleton->setAngles();
    } else {
            cout << "Unreachable position" << endl;
    }
    
}

void TMoveDownModel::toNeutral(ICompletionListener* complete) {
    TargetState = STATE_NEUTRAL;
    Complete = complete;
    move();
}

void TMoveDownModel::toDown(ICompletionListener* complete) {
    TargetState = STATE_DOWN;
    Complete = complete;
    move();
}

bool TMoveForwardModel::groundedGroup1(double newState) {
    Leg0.keep();
    Leg1.keepX();
    Leg1.Y = -newState;
    Leg2.keep();
    Leg3.keepX();
    Leg3.Y = Skeleton->Y - newState;
    Leg4.keep();
    Leg5.keepX();
    Leg5.Y = -Skeleton->Y - newState;

    bool movement = false;
    bool stateAchieved = step(movement);
            
    if (movement) {
        Skeleton->Leg0.Y = Skeleton->Y + newState;
        Skeleton->Leg2.Y = -Skeleton->Y + newState;
        Skeleton->Leg4.Y = newState;
    }
    return stateAchieved;
}

bool TMoveForwardModel::groundedGroup2(double newState) {
    Leg0.keepX();
    Leg0.Y = Skeleton->Y + newState;
    Leg1.keep();
    Leg2.keepX();
    Leg2.Y = -Skeleton->Y + newState;
    Leg3.keep();
    Leg4.keepX();
    Leg4.Y = newState;
    Leg5.keep();

    bool movement = false;
    bool stateAchieved = step(movement);
            
    if (movement) {
        Skeleton->Leg1.Y = - newState;
        Skeleton->Leg3.Y = Skeleton->Y - newState;
        Skeleton->Leg5.Y = -Skeleton->Y - newState;
    }
    return stateAchieved;
}

bool TMoveForwardModel::setTargetsForLegs() {

    cout << "forward: distance=" << LeftDistance << " state=" << State << endl;

    if (LeftDistance == 0) {
        if (StopNeutral && (State != 0)) {
            if (stepToNeutral()) {
                State = 0;
            }
        } else {
            return false;
        }
    } else {
        if (State >= 0) {
            auto d = Skeleton->S / 2 + State;
            if (d > abs(LeftDistance)) {
                d = abs(LeftDistance);
            }
            auto newState = State - d;
            cout << "newState(>)=" << newState << endl;
            if (LeftDistance > 0) {
                if (groundedGroup1(newState)) {
                    State = newState;
                    LeftDistance -= d;
                }
            } else {
                if (groundedGroup2(newState)) {
                    State = newState;
                    LeftDistance -= -d;
                }
            }
        } else {
            auto d = Skeleton->S / 2 - State;
            if (d > abs(LeftDistance)) {
                d = abs(LeftDistance);
            }
            auto newState = State + d;
            cout << "newState(<)=" << newState << endl;
            if (LeftDistance > 0) {
                if (groundedGroup2(newState)) {
                    State = newState;
                    LeftDistance -= d;
                }
            } else {
                if (groundedGroup1(newState)) {
                    State = newState;
                    LeftDistance -= -d;
                }
            }
        }
    }
    return true;
}

bool TMoveDirModel::setTargetsForLegs() {

    cout << "dir: distance=" << LeftDistance << " state=" << State << endl;

    if (LeftDistance == 0) {
        if (StopNeutral && (State != 0)) {
            if (stepToNeutral()) {
                State = 0;
            }
        } else {
            return false;
        }
    } else {
        auto d = Skeleton->S / 2 + State;
        if (d > abs(LeftDistance)) {
            d = abs(LeftDistance);
        }
        auto newState = State - d;
        cout << "newState(>)=" << newState << endl;        Leg0.keep();
        Leg1.X = Skeleton->XM + newState;
        Leg1.keepY();
        Leg2.keep();
        Leg3.X = Skeleton->XN + newState;
        Leg3.keepY();
        Leg4.keep();
        Leg5.X = Skeleton->XN + newState;
        Leg5.keepY();

        bool movement = false;
        bool stateAchieved = step(movement);
/*            
        if (movement) {
            Skeleton->Leg0.X = Skeleton->RE * cos(Skeleton->AN + newState) - Skeleton->FW / 2;
            Skeleton->Leg0.Y = Skeleton->RE * sin(Skeleton->AN + newState) - Skeleton->FH / 2;
            Skeleton->Leg2.X = Skeleton->RE * cos(Skeleton->AN - newState) - Skeleton->FW / 2;
            Skeleton->Leg2.Y = -(Skeleton->RE * sin(Skeleton->AN - newState) - Skeleton->FH / 2);
            Skeleton->Leg4.X = Skeleton->RM * cos(-newState) - Skeleton->FW / 2;
            Skeleton->Leg4.Y = Skeleton->RM * sin(-newState);
        }
*/
        if (stateAchieved) {
            State = newState;
            LeftDistance -= d;
        }

        return stateAchieved;

        /*
        if (State >= 0) {
            auto d = Skeleton->S / 2 + State;
            if (d > abs(LeftDistance)) {
                d = abs(LeftDistance);
            }
            auto newState = State - d;
            cout << "newState(>)=" << newState << endl;
            if (LeftDistance > 0) {
                if (groundedGroup1(newState)) {
                    State = newState;
                    LeftDistance -= d;
                }
            } else {
                if (groundedGroup2(newState)) {
                    State = newState;
                    LeftDistance -= -d;
                }
            }
        } else {
            auto d = Skeleton->S / 2 - State;
            if (d > abs(LeftDistance)) {
                d = abs(LeftDistance);
            }
            auto newState = State + d;
            cout << "newState(<)=" << newState << endl;
            if (LeftDistance > 0) {
                if (groundedGroup2(newState)) {
                    State = newState;
                    LeftDistance -= d;
                }
            } else {
                if (groundedGroup1(newState)) {
                    State = newState;
                    LeftDistance -= -d;
                }
            }
        }*/
    }
    return true;
}

void TMoveForwardModel::toNeutral(ICompletionListener* complete) {
    LeftDistance = 0;
    Complete = complete;
    StopNeutral = true;
    move();
}

void TMoveForwardModel::moveForward(double distance, ICompletionListener* complete) {
    LeftDistance = distance;
    Complete = complete;
    StopNeutral = false;
    move();
}


void TMoveDirModel::toNeutral(ICompletionListener* complete) {
    LeftDistance = 0;
    Complete = complete;
    StopNeutral = true;
    move();
}

void TMoveDirModel::moveDir(double distance, ICompletionListener* complete) {
    LeftDistance = distance;
    Complete = complete;
    StopNeutral = false;
    move();
}

bool TTurnModel::groundedGroup1(double newState) {
    Leg0.keep();
    Leg1.X = Skeleton->RM * cos(-newState) - Skeleton->FW / 2;
    Leg1.Y = Skeleton->RM * sin(-newState);
    Leg2.keep();
    Leg3.X = Skeleton->RE * cos(Skeleton->AN + newState) - Skeleton->FW / 2;
    Leg3.Y = Skeleton->RE * sin(Skeleton->AN + newState) - Skeleton->FH / 2;
    Leg4.keep();
    Leg5.X = Skeleton->RE * cos(Skeleton->AN - newState) - Skeleton->FW / 2;
    Leg5.Y = -(Skeleton->RE * sin(Skeleton->AN - newState) - Skeleton->FH / 2);

    bool movement = false;
    bool stateAchieved = step(movement);
            
    if (movement) {
        Skeleton->Leg0.X = Skeleton->RE * cos(Skeleton->AN + newState) - Skeleton->FW / 2;
        Skeleton->Leg0.Y = Skeleton->RE * sin(Skeleton->AN + newState) - Skeleton->FH / 2;
        Skeleton->Leg2.X = Skeleton->RE * cos(Skeleton->AN - newState) - Skeleton->FW / 2;
        Skeleton->Leg2.Y = -(Skeleton->RE * sin(Skeleton->AN - newState) - Skeleton->FH / 2);
        Skeleton->Leg4.X = Skeleton->RM * cos(-newState) - Skeleton->FW / 2;
        Skeleton->Leg4.Y = Skeleton->RM * sin(-newState);
        if (Speed > 0.5) {
            StepSpeed = 0.5;
        }
    }
    return stateAchieved;
}

bool TTurnModel::groundedGroup2(double newState) {
    Leg0.X = Skeleton->RE * cos(Skeleton->AN + newState) - Skeleton->FW / 2;
    Leg0.Y = Skeleton->RE * sin(Skeleton->AN + newState) - Skeleton->FH / 2;
    Leg1.keep();
    Leg2.X = Skeleton->RE * cos(Skeleton->AN - newState) - Skeleton->FW / 2;
    Leg2.Y = -(Skeleton->RE * sin(Skeleton->AN - newState) - Skeleton->FH / 2);
    Leg3.keep();
    Leg4.X = Skeleton->RM * cos(-newState) - Skeleton->FW / 2;
    Leg4.Y = Skeleton->RM * sin(-newState);
    Leg5.keep();

    bool movement = false;
    bool stateAchieved = step(movement);
            
    if (movement) {
        Skeleton->Leg1.X = Skeleton->RM * cos(-newState) - Skeleton->FW / 2;
        Skeleton->Leg1.Y = Skeleton->RM * sin(-newState);
        Skeleton->Leg3.X = Skeleton->RE * cos(Skeleton->AN + newState) - Skeleton->FW / 2;
        Skeleton->Leg3.Y = Skeleton->RE * sin(Skeleton->AN + newState) - Skeleton->FH / 2;
        Skeleton->Leg5.X = Skeleton->RE * cos(Skeleton->AN - newState) - Skeleton->FW / 2;
        Skeleton->Leg5.Y = -(Skeleton->RE * sin(Skeleton->AN - newState) - Skeleton->FH / 2);
        if (Speed > 0.5) {
            StepSpeed = 0.5;
        }
    }
    return stateAchieved;
}

bool TTurnModel::setTargetsForLegs() {
    cout << "turn: angle=" << (LeftAngle * 180 / M_PI) << " state=" << (State * 180 / M_PI) << endl;

    if (LeftAngle == 0) {
        if (StopNeutral && (State != 0)) {
            if (stepToNeutral()) {
                State = 0;
            }
        } else {
            return false;
        }
    } else {
        if (State >= 0) {
            auto d = Skeleton->Tmax + State;
            if (d > abs(LeftAngle)) {
                d = abs(LeftAngle);
            }
            auto newState = State - d;
            cout << "newState(>0)=" << (newState * 180 / M_PI) << endl;
            if (LeftAngle > 0) {
                if(groundedGroup1(newState)) {
                    LeftAngle -= d;
                    State = newState;
                }
            } else {
                if (groundedGroup2(newState)) {
                    LeftAngle -= -d;
                    State = newState;
                }
            }
        } else {
            auto d = Skeleton->Tmax - State;
            if (d > abs(LeftAngle)) {
                d = abs(LeftAngle);
            }
            auto newState = State + d;
            cout << "newState(<0)=" << (newState * 180 / M_PI) << endl;
            if (LeftAngle > 0){
                if (groundedGroup2(newState)) {
                    LeftAngle -= d;
                    State = newState;
                }
            } else {
                if(groundedGroup1(newState)) {
                    LeftAngle -= -d;
                    State = newState;
                }
            }
        }
    }

    return true;
}

void TTurnModel::toNeutral(ICompletionListener* complete) {
    LeftAngle = 0;
    Complete = complete;
    StopNeutral = true;
    move();
}

void TTurnModel::turnAngle(double angle, ICompletionListener* complete) {
    LeftAngle = angle;
    Complete = complete;
    StopNeutral = false;
    move();
}

void TMove2::move() {
    switch(Command) {
    case CMD_DOWN:
        CurrentModel = &MoveDownModel;
        MoveDownModel.toDown();
        break;
    case CMD_FORWARD:
        CurrentModel = &MoveForwardModel;
        MoveForwardModel.moveForward(Distance);
        break;
    case CMD_TURN:
        CurrentModel = &TurnModel;
        TurnModel.turnAngle(Angle);
        break;
    case CMD_DIR:
        CurrentModel = &MoveDirModel;
        MoveDirModel.moveDir(Distance);
        break;
    }
}

void TMove2::complete(IProcess* src) {
    move();
}


void TMove2::initPosition() {
    CurrentModel = &MoveDownModel;
    MoveDownModel.initPosition();
}

void TMove2::toNeutral() {
    CurrentModel->toNeutral();
}

void TMove2::toDown() {
    Command = CMD_DOWN;
    if (CurrentModel == &MoveDownModel) {
        move();
    } else {
        CurrentModel->toNeutral(this);
    }
}

void TMove2::moveForward(double distance) {
    Command = CMD_FORWARD;
    Distance = distance;
    if (CurrentModel == &MoveForwardModel) {
        move();
    } else {
        CurrentModel->toNeutral(this);
    }
}

void TMove2::turn(double angle) {
    Command = CMD_TURN;
    Angle = angle;
    if (CurrentModel == &TurnModel) {
        move();
    } else {
        CurrentModel->toNeutral(this);
    }
}

void TMove2::setSpeed(double speed) {
    MoveDownModel.Speed = speed;
    MoveForwardModel.Speed = speed;
    TurnModel.Speed = speed;
}

void TMove2::moveDir(double distance) {
    Command = CMD_DIR;
    Distance = distance;
    if (CurrentModel == &MoveDirModel) {
        move();
    } else {
        CurrentModel->toNeutral(this);
    }
}

