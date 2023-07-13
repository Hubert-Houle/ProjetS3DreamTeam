

#ifndef DT_PID_H
#define DT_PID_H 

class DT_pid
{
	public:

		DT_pid(double* IN_SP,double* IN_PV,double IN_KP=0,double IN_KI=0,double IN_KD=0);

		double erreur_P;
		double erreur_I;
		double erreur_D;
		
		double response_Sum();


	private:
		double* SP;
		double* PV;
		double CV;
		
		double KP;
		double KI;
		double KD;



		void erreurFCN();
		double P();
		double I();
		double D();
		
};


#endif
