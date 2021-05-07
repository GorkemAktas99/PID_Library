typedef struct pidStruct{
	
	volatile double pid_kp;
	volatile double pid_kd;
	volatile double pid_ki;
	volatile double stepsize;
	
}PID;
/*
typedef struct systemSimulationStruct{
	double sim_K;
	double sim_zeta;
	double sim_wn;
	
}SystemSimulation;
*/
void PID_Config(PID sample_pid);
//void Simulation_Config(SystemSimulation sample_sys);
double Derivative(double ek, double ek_1);
double Integral(double ek, double ek_1);
double PID_Function(double r, double y);
/*
double eq1(double y1, double y2, double u);
double eq2(double y, double y1, double u);
double ExplicitEulerEq1(double (*f)(double,double,double), double y2, double ts, double uk);
double ExplicitEulerEq2(double (*f)(double,double,double), double y1, double ts, double uk);
double System_Simulator();
*/
/*
NOTLAR:
-27Nisan2021 : Kütüphanenin PID kismi simdilik tamamlandi. Gerekli türev ve integral alicilar tanimlandi
Simülasyon kismi için de sablonlar hazirlandi. Tamamlanacak
*/
