

#ifndef DT_PID_H
#define DT_PID_H 

#include "stdlib.h"

class DT_pid
{
	public:

		DT_pid(float* IN_SP,float* IN_PV,float IN_KP=0,float IN_KI=0,float IN_KD=0);

		float erreur_P;
		float erreur_I;
		float erreur_D;
		
		float response_Sum();
		float response_Sum_Oscille();

	private:

		float* SPk;
		float* PV;
		float CV;
		
		float KP;
		float KI;
		float KD;



		void erreurFCN();
		void erreurFCN2();
		float P();
		float I();
		float D();
		
};


#endif
