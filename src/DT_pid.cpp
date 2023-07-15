
#include "DT_pid.h"

DT_pid::DT_pid(float* IN_SPk,float* IN_PV,float IN_KP,float IN_KI,float IN_KD)
{
	SPk=IN_SPk;
	PV=IN_PV;
	
	KP=IN_KP;
	KI=IN_KI;
	KD=IN_KD;

	erreur_P=0;
	erreur_I=0;
	erreur_D=0;	
}
	
	

void DT_pid::erreurFCN()
{
	erreur_D=erreur_P-(*SPk-*PV);
	erreur_P=*SPk-*PV;	
	erreur_I+=erreur_P;
	
		
}



float DT_pid::P()
{

	return KP*erreur_P;
}

float DT_pid::I()
{

	return KI*erreur_I;
}

float DT_pid::D()
{

	return KD*erreur_D;
}

float DT_pid::response_Sum()
{


	erreurFCN();

	CV=P()*(KP!=0)+I()*(KI!=0)+D()*(KD!=0);
	return CV;
	
}
