#include <iostream>
#include <unistd.h>

#include <cmath>

using namespace std;

void countab(double x, double h, double& a, double& b) {
    double A = 75;
    double B = 167;
    b = asin((x*x + h*h - A*A -B*B)/(2*A*B));
    a = asin(x / sqrt(A*A + B*B + 2 * A * B * sin(b))) - atan((A + B*sin(b))/(B* cos(b)));
}

void test2() {
    double a;
    double b;
    
    countab(75, 167, a, b);
    cout << "a=" << (a*180/M_PI) << " b=" << (b*180/M_PI) << endl;
    countab(75, 150, a, b);
    cout << "a=" << (a*180/M_PI) << " b=" << (b*180/M_PI) << endl;
}

int main(int argc, char *argv[]) {
	
	test2();
	//test3();
	//test4();
}
