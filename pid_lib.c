#include "pid_lib.h"

volatile double Kp;
volatile double Ki;
volatile double Kd;
volatile double stepSize;

double K;
double zeta;
double wn;

void PID_Config(PID sample_pid){
	Kp = sample_pid.pid_kp;
	Kd = sample_pid.pid_kd;
	Ki = sample_pid.pid_ki;
	stepSize = sample_pid.stepsize;
	
}
/*
void Simulation_Config(SystemSimulation sample_sys){
	K = sample_sys.sim_K;
	zeta = sample_sys.sim_zeta;
	wn = sample_sys.sim_wn;
}
*/
double Derivative(double ek, double ek_1)
{
 return (ek - ek_1) / stepSize;
}
double Integral(double ek, double ek_1)
{
 return (ek + ek_1) * (stepSize / 2);
}
double PID_Function(double r, double y)
{
 static double ek_1 = 0, integralSum = 0;
 double outputPID;
 double ek = r - y;
 integralSum += Integral(ek, ek_1);
 outputPID = Kp * ek + Ki * integralSum + Kd * Derivative(ek, ek_1);
 ek_1 = ek;
 return outputPID;
}
/*
double eq1(double y1, double y2, double u)
{
 return y2;
}
double eq2(double y, double y1, double u)
{
 return (K*wn*wn*u)-(2*zeta*wn*y)-(wn*wn*y1);
}
double ExplicitEulerEq1(double (*f)(double,double,double), double y2, double ts, double uk){
 static double yk = 0;
 double yk1 = yk + ts * f(yk, y2, uk);
 yk = yk1;
 return yk1;
}
double ExplicitEulerEq2(double (*f)(double,double,double), double y1, double ts, double uk){
 static double yk = 0; // Previous value of the output
 double yk1 = yk + ts * f(yk, y1, uk);
 yk = yk1;
 return yk1;
}
*/
