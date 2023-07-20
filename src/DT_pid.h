

#ifndef DT_PID_H
#define DT_PID_H 

class DT_pid
{
	public:

		DT_pid(float* IN_SP,float* IN_PV,float IN_KP=0,float IN_KI=0,float IN_KD=0);

		float erreur_P;
		float erreur_I;
		float erreur_D;
		
		float response_Sum();
		void setSP(float IN_SPk);


	private:

		float* SPk;
		float* PV;
		float CV;
		
		float KP;
		float KI;
		float KD;



		void erreurFCN();
		float P();
		float I();
		float D();
		
};


#endif
