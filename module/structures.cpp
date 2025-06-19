#include "structures.h"
    
#include <iostream>
#include <unistd.h>

#include <cmath>

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
        cout << "close area: cx=" << cx << " cy=" << cy << " cr=" << cr << " r=" << (r + C)  << endl;

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
    cout << Id << " [a=" << As << ", b=" << Bs << ", c=" << Cs << "]" << endl;
    Complete = complete;
    BottomServo.setTargetAngle(Bs, speed * BottomServo.getTargetDistance(Bs) / maxDistance, this);
    MiddleServo.setTargetAngle(As, speed * MiddleServo.getTargetDistance(As) / maxDistance, this);
    TopServo.setTargetAngle(Cs, speed * TopServo.getTargetDistance(Cs) / maxDistance, this);
}


int TLeg::getMaxTargetDistance() {
    auto Bd = BottomServo.getTargetDistance(Bs);
    auto Ad = MiddleServo.getTargetDistance(As);
    auto Cd = TopServo.getTargetDistance(Cs);

    return Bd > Ad ? (Bd > Cd ? Bd : Cd) : (Ad > Cd ? Ad : Cd);
}

bool TLeg::countTurn(double angle) {
    double gx = GX + X;
    double gy = GY + Y; 
    double r = sqrt(gx * gx + gy * gy);
    double a = atan2(gy, gx);
    double a2 = a + angle * M_PI / 180.0;

    double x2 = r * cos(a2) - GX;
    double y2 = r * sin(a2) - GY;

    cout << "countTurn " << Id << " ("<< X << ", " << Y << ") g=(" << gx << ", " << gy <<
         ") r=" << r << " a=" << (a * 180 / M_PI) << " a2=" << (a2 * 180 / M_PI) << " new=(" << x2 <<
         ", " << y2 << ") " << endl;

    X = x2;
    Y = y2;
    return countABC();
}

void TSkeleton::complete(IProcess* src){
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

void TSkeleton::initPosition() {

    DX = 0;
    DY1 = 0;
    DY2 = 0;

    H1 = Hdown;
    H2 = H1;

    X1 = Leg0.C + sqrt(Leg0.B*Leg0.B - Leg0.A*Leg0.A);
    X2 = X1;

    if (count()) {
        setAngles();
    }
}

void TSkeleton::countParams() {
    
    // Alpha
    double A2 = 25 * M_PI / 180;
    
    // Walking H
    // min H
    //H0 = sqrt(Leg0.B * Leg0.B - D0*D0) - Leg0.A;
    H0 = 100;

    // Neutral X for edge legs
    XN = (D0 + Leg0.C) * cos(A2);

    // max distance leg can reach for H0
    double D = Leg0.C + sqrt((Leg0.A + Leg0.B) * (Leg0.A + Leg0.B) - H0 * H0);

    // max Y for edge leg
    double Ymax = sqrt(D*D - XN*XN);

    // min Y for edge leg
    double Ymin = XN * tan(A2);

    // step size (max for edge leg, should be less or equal middle: 2 * D * sin(A2))
    S = Ymax - Ymin - 100;

    // X for middle
    XM1 = S / (2 * tan(A2));
    XM2 = XM1;

    // Neutral Y for edge
    Y = (Ymax + Ymin) / 2;

    cout << "H0=" << H0 << " S=" << S << " XN=" << XN << " XM=" << XM1 << " Y=" << Y << endl;
}

bool TSkeleton::count() {
    Leg0.H = H1;
    Leg0.X = X1 + DX;
    Leg0.Y = Y + DY1;

    Leg1.H = H2;
    Leg1.X = XM2 + DX;
    Leg1.Y = DY2;

    Leg2.H = H1;
    Leg2.X = X1 + DX;
    Leg2.Y = -Y + DY1;
 
    Leg3.H = H2;
    Leg3.X = X2 - DX;
    Leg3.Y = Y + DY2;

    Leg4.H = H1;
    Leg4.X = XM1 - DX;
    Leg4.Y = DY1;
 
    Leg5.H = H2;
    Leg5.X = X2 + -DX;
    Leg5.Y = -Y + DY2;

    return Leg0.countABC() &&
           Leg1.countABC() &&
           Leg2.countABC() &&
           Leg3.countABC() &&
           Leg4.countABC() &&
           Leg5.countABC();
}

void TSkeleton::setAngles() {
    Leg0.setAngles();
    Leg1.setAngles();
    Leg2.setAngles();
    Leg3.setAngles();
    Leg4.setAngles();
    Leg5.setAngles();
}

void TSkeleton::setAngles(ICompletionListener* complete) {
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
    Leg0.setAngles(Speed, dmax, this);
    Leg1.setAngles(Speed, dmax, this);
    Leg2.setAngles(Speed, dmax, this);
    Leg3.setAngles(Speed, dmax, this);
    Leg4.setAngles(Speed, dmax, this);
    Leg5.setAngles(Speed, dmax, this);
}

TMove::TMove(TSkeleton* skeleton) {
    Skeleton = skeleton;

    A = Skeleton->Leg0.A;
    B = Skeleton->Leg0.B;
    C = Skeleton->Leg0.C;

    H1 = skeleton->H1;
    H2 = skeleton->H2;
    X1 = skeleton->X1;
    X2 = skeleton->X2;
    XM1 = skeleton->XM1;
    XM2 = skeleton->XM2;
    Y = skeleton->Y;
    DY1 = skeleton->DY1;
    DY2 = skeleton->DY2;
    DX = skeleton->DX;

    TY = 0.0;

    cout << "Init: H1=" << H1 << " H2=" << H2 << " X1=" << X1 << " XM1=" << XM1 << " Y=" << Y << endl;
}


void TMove::complete(IProcess* src) {
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

double TMove::getMinHforXY(double X, double Y) {
    auto D = sqrt(X*X + Y*Y) - C;
    auto AH2 = B*B - D * D;
    cout << "X=" << X << " Y=" << Y << " D=" << D << " AH2=" << AH2 << " A2=" << (A*A) << endl; 
    if (AH2 > A*A) {
        return sqrt(AH2) - A;
    } else {
        return 0;
    }

}

bool TMove::run() {
    if (Skeleton->count()) {
        Skeleton->setAngles(this);
        return true;
    } else {
        cout << "Calculation Error" << endl;
        return false;
    }
}

bool TMove::moveXDYStayOn2() {
    // stay on group 2, need to move
    if (X1 != Skeleton->X1 || DY1 != Skeleton->DY1) {
        cout << "need to move X1,DY1/DY2" << endl;
        Skeleton->X1 = X1;
        Skeleton->DY1 = DY1;
        Skeleton->DY2 = DY2;
        return run();
    } else {
        if (H1 != Skeleton->H1) {
            cout << "X1/DY1 achieved, need to plase group 1 on the graund" << endl;
            Skeleton->H1 = H1;
            return run();
        } else {
            cout << "need to move only X2,DY2, switch the ground group (shold neven happen)" << endl;
            Skeleton->H1 = Skeleton->H2;
            return run();
        }
    }
}

bool TMove::moveXDYStayOn1() {
    // stay on group 1, need to move
    if (X2 != Skeleton->X2 || DY2 != Skeleton->DY2) {
        cout << "need to move X2,DY2/DY1" << endl;
        Skeleton->X2 = X2;
        Skeleton->DY1 = DY1;
        Skeleton->DY2 = DY2;
        return run();
    } else {
        if (H2 != Skeleton->H2) {
            Skeleton->H2 = H2;
            return run();
        } else {
            cout << "need to move only X1,DY1, switch the ground group (shold neven happen)" << endl;
            Skeleton->H2 = Skeleton->H1;
            return run();
        }
    }
}

bool TMove::liftGroup1() {
    Skeleton->H1 = Skeleton->Hdown;
    return run();
}

bool TMove::liftGroup2() {
    Skeleton->H2 = Skeleton->Hdown;
    return run();    
}

bool TMove::move() {
    // H1 - H group 1
    // H2 - H group 2
    // DY1 - front shift, group 1
    // DY2 - front shift, group 2
    // X1,X2 - X for group 1 and 2
    // TY  - target Y

    if ((H1 == Skeleton->H1) &&
        (H2 == Skeleton->H2) &&
        (X1 == Skeleton->X1) &&
        (X2 == Skeleton->X2) &&
        (DY1 == Skeleton->DY1) &&
        (DY2 == Skeleton->DY2)) {
        
        if (TY > 0.0) {
            // can move on group 1
            auto g1 = Skeleton->S / 2 - Skeleton->DY1;
            // can move on group 2
            auto g2 = Skeleton->S / 2 - Skeleton->DY2;
            cout << "Need to move TY=" << TY << " g1=" << g1 << " g2=" << g2 << endl;
            if (Skeleton->H1 > Skeleton->Hdown) {
                // group 1 is not up
                if (Skeleton->H1 == Skeleton->H2) {
                    cout << "stay on all legs" << endl;
                    if (g1 <= g2) {
                        // g1 has less or equal space to move
                        if (g1 > 0) {
                            cout << "0 < g1 <= g2" << endl;
                            if (TY <= g1) {
                                cout << "no need to walk, just move TY" << endl;
                                DY1 = Skeleton->DY1 + TY;
                                DY2 = Skeleton->DY2 + TY;
                                TY = 0;
                            } else {
                                cout << "need to walk, move just on distance g1 now" << endl;
                                DY1 = Skeleton->DY1 + g1;
                                DY2 = Skeleton->DY2 + g1;
                                TY -= g1;
                            }
                        } else {
                            cout << "g1 does not have space to move, need to move it back" << endl;
                            H1 = Skeleton->Hdown;
                            H2 = Skeleton->H0;
                        }
                    } else {
                        // g2 has less space to move
                        if (g2 > 0) {
                            cout << "0 < g2 < g1" << endl;
                            if (TY <= g2) {
                                cout << "no need to walk, just move TY" << endl;
                                DY1 = Skeleton->DY1 + TY;
                                DY2 = Skeleton->DY2 + TY;
                                TY = 0;
                            } else {
                                cout << "need to walk, move just on g2 distance now" << endl;
                                DY1 = Skeleton->DY1 + g2;
                                DY2 = Skeleton->DY2 + g2;
                                TY -= g2;
                            }
                        } else {
                            cout << "g2 does not have space to move, need to move it  back" << endl;
                            H1 = Skeleton->H0;
                            H2 = Skeleton->Hdown;
                        }
                    }
                } else {
                    if (Skeleton->H1 > Skeleton->H2) {
                        cout << "stay on group 1" << endl;
                        if (g1 > 0) {
                            cout << "group 1 can move" << endl;
                            if (TY <= g1) {
                                cout << "no need to walk, just move TY" << endl;
                                DY1 = Skeleton->DY1 + TY;
                                DY2 = Skeleton->DY2 - TY;
                                TY = 0;
                            } else {
                                cout << "need to walk, move on group 1, moving back group 2" << endl;
                                DY1 = Skeleton->DY1 + g1;
                                DY2 = -Skeleton->S / 2;
                                TY -= g1;
                            }
                        } else {
                            // no space to move on group 1
                            if (g2 == Skeleton->S) {
                                cout << "no space to move group 1, group 2 is fully back, so just switch groups" << endl;
                                H1 = Skeleton->H0;
                                H2 = Skeleton->H0;
                            } else {
                                cout << "no space to move group 1, move group 2 back" << endl;
                                H1 = Skeleton->H0;
                                H2 = Skeleton->Hdown;
                                DY2 = -Skeleton->S / 2;
                            }
                        }
                    } else {
                        cout << "stay on group 2" << endl;
                        if (g2 > 0) {
                            cout << "group 2 can move" << endl;
                            if (TY <= g2) {
                                cout << "no need to walk, just move TY" << endl;
                                DY1 = Skeleton->DY1 - TY;
                                DY2 = Skeleton->DY2 + TY;
                                TY = 0;
                            } else {
                                cout << "need to walk, move on group 2, moving back group 1" << endl;
                                DY1 = -Skeleton->S / 2;
                                DY2 = Skeleton->DY2 + g2;
                                TY -= g2;
                            }
                        } else {
                            // no space to move on group 2
                            if (g1 == Skeleton->S) {
                                cout << "no space to move group 2, group 1 is fully back, so just switch groups" << endl;
                                H1 = Skeleton->H0;
                                H2 = Skeleton->H0;
                            } else {
                                cout << "no space to move group 2, move group 1 back" << endl;
                                H1 = Skeleton->Hdown;
                                H2 = Skeleton->H0;
                                DY1 = -Skeleton->S / 2;
                            }
                        }
                    }
                }
            } else {
                // group 1 is up
                if (Skeleton->H2 > Skeleton->Hdown) {
                    cout << "stay on group 2" << endl;
                    if (g2 > 0) {
                        cout << "group 2 can move" << endl;
                        if (TY <= g2) {
                            cout << "no need to walk, just move TY" << endl;
                            DY1 = Skeleton->DY1 - TY;
                            DY2 = Skeleton->DY2 + TY;
                            TY = 0;
                        } else {
                            cout << "need to walk, move on group 2, moving back group 1" << endl;
                            DY1 = -Skeleton->S / 2;
                            DY2 = Skeleton->DY2 + g2;
                            TY -= g2;
                        }
                    } else {
                        // no space to move on group 2
                        if (g1 == Skeleton->S) {
                            cout << "no space to move group 2, group 1 is fully back, so just switch groups" << endl;
                            H1 = Skeleton->H0;
                            H2 = Skeleton->H0;
                        } else {
                            cout << "no space to move group 2, move group 1 back" << endl;
                            H1 = Skeleton->Hdown;
                            H2 = Skeleton->H0;
                            DY1 = -Skeleton->S / 2;
                        }
                    }
                } else {
                    cout << "down position, lets. lift" << endl;
                    H1 = Skeleton->H0;
                    H2 = Skeleton->H0;
                }
            }

/*
            if (Skeleton->H1 == Skeleton->H0 && 
                Skeleton->H2 == Skeleton->H0 &&
                Skeleton->X1 == Skeleton->XN &&
                Skeleton->X2 == Skeleton->XN) {

                // ready to walk
                
                    cout << "Need to walk DY1=" << DY1 << " DY2=" << DY2 << endl;
                    if (Skeleton->DY1 < Skeleton->DY2) {
                        TY -= Skeleton->S / 2 - Skeleton->DY1;
                        DY1 = Skeleton->S / 2;
                        DY2 = - Skeleton->S / 2;
                    } else {
                        TY -= Skeleton->S / 2 - Skeleton->DY2;
                        DY2 = Skeleton->S / 2;
                        DY1 = - Skeleton->S / 2;
                    }
                }
            } else {
                cout << "need to move to a walking position first" << endl;
                H1 = Skeleton->H0;
                H2 = Skeleton->H0;
                X1 = Skeleton->XN;
                X2 = Skeleton->XN;
            }
*/
        } else {
            cout << "Everything is approached" << endl;
            return true;
        }
    } else {
        cout << "need to finalize position: H1,H2, X1,X2, DY1 or DY2 are not finished" << endl;
    }

    if (X1 != Skeleton->X1 || X2 != Skeleton->X2 || DY1 != Skeleton->DY1 || DY2 != Skeleton->DY2) {
        //cout << "Need to move X1, X2, DY1 or DY2" << endl;
        if (Skeleton->H1 > Skeleton->Hdown) {
            // group 1 is not in upper position
            if (Skeleton->H2 > Skeleton->Hdown) {
                // legs are not in upper porsition
                if (Skeleton->H1 == Skeleton->H2) {
                    cout << "all legs on the ground" << endl;
                    if (X1 == Skeleton->X1 && X2 == Skeleton->X2 && ((DY1 - Skeleton->DY1) == (DY2 - Skeleton->DY2))) {
                        cout << "both groups move togather" << endl;
                        Skeleton->DY1 = DY1;
                        Skeleton->DY2 = DY2;
                        return run();
                    } else {
                        if (X1 != Skeleton->X1 || DY1 != Skeleton->DY1) {
                            cout << "Need to move X1 or DY1/DY2, lift group 1" << endl;
                            return liftGroup1();
                        } else {
                            cout << "Need to move X2 or DY2, lift group 2" << endl;
                            return liftGroup2();
                        }
                    }
                } else {
                    if (Skeleton->H1 > Skeleton->H2) {
                        cout << "stay on group 1" << endl;
                        return moveXDYStayOn1();
                    } else {
                        cout << "stay on group 2" << endl;
                        return moveXDYStayOn2();
                    }
                }
            } else {
                cout << "stay on group 1" << endl;
                return moveXDYStayOn1();
            }
        } else {
            // group 1 is in upper position
            if (Skeleton->H2 > Skeleton->Hdown) {
                cout << "stay on group 2" << endl;
                return moveXDYStayOn2();
            } else {
                cout << "all legs are in upper position" << endl;
                double Hmin1 = getMinHforXY(X1,Y + DY1);
                double Hmin2 = getMinHforXY(X2,Y + DY2);
                if (Hmin1 > Skeleton->Hdown || Hmin2 > Skeleton->Hdown) {
                    cout << "need to go up before moving X1,Y1 and X2,Y2 Hmin1=" << Hmin1 << " Hmin2=" << Hmin2 << endl;
                    Skeleton->H1 = H1;
                    Skeleton->H2 = H2;
                    return run();
                } else {
                    cout << "X1,DY1,X2,Dy2 can be moved now, Hmin1=" << Hmin1 << " Hmin2=" << Hmin2 << endl;
                    Skeleton->X1 = X1;
                    Skeleton->DY1 = DY1;
                    Skeleton->X2 = X2;
                    Skeleton->DY2 = DY2;
                    return run();
                }
            }
        }
    } else {
        //cout << "No need to move X1,Y1 or X2,Y2" << endl;
        if (H1 != Skeleton->H1 || H2 != Skeleton->H2) {
            if (H1 != Skeleton->H1) {
                cout << "Need to change H1" << endl;
                Skeleton->H1 = H1;    
            }
            if (H2 != Skeleton->H2) {
                cout << "Need to change H2" << endl;
                Skeleton->H2 = H2;
            }
            return run();
        } else {
            cout << "No need to change anything (should not be reached)" << endl;
            return true;
        }
    }

    return false;
}


bool TMove::toNeutral() {
    H1 = Skeleton->H0;
    H2 = Skeleton->H0;
    X1 = Skeleton->XN;
    X2 = Skeleton->XN;
    DY1 = 0;
    DY2 = 0;
    TY = 0;
    return move();
}

bool TMove::toDown() {
    H1 = Skeleton->Hdown;
    H2 = Skeleton->Hdown;
    X1 = C + sqrt(B*B - A*A);
    X2 = X1;
    TY = 0;
    return move();
}

bool TMove::moveForward(double distance) {
    TY = distance;
    return move();
}
