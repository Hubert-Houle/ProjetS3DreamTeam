
#include "DT_pid.h"

DT_pid::DT_pid(double* IN_SP,double* IN_PV,double IN_KP,double IN_KI,double IN_KD)
{
	SP=IN_SP;
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
	erreur_D=erreur_P-(SP-PV);
	erreur_P=SP-PV;	
	erreur_I+=erreur_P;
	
		
}



double DT_pid::P()
{

	return KP*erreur_I;
}

double DT_pid::I()
{

	return KI*erreur_P;
}

double DT_pid::D()
{

	return KD*erreur_D;
}

double DT_pid::response_Sum()
{


	erreurFCN();

	CV=P()*(KP!=0)+I()*(KI!=0)+D()*(KD!=0);
	return CV;
	
}
