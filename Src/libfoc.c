/*
 * svpwm.c
 *
 *  Created on: 21 Jun 2017
 *      Author: PhuongLe
 */

#include <IQmathLib.h>
#include <libfoc.h>

void svpwm_calculate(SVGENDQ* sv)
{
	_iq Va, Vb, Vc, t1, t2, temp_sv1, temp_sv2;
	uint16_t Sector = 0; // Sector is treated as Q0 - independently with global Q

	Sector = 0;
	temp_sv1 = _IQdiv2(sv->Ubeta); /*divide by 2*/
	temp_sv2 = _IQmpy(_IQ(0.8660254), sv->Ualpha); /* 0.8660254 = sqrt(3)/2*/

	/* Inverse clarke transformation */
	Va = sv->Ubeta;
	Vb = -temp_sv1 + temp_sv2;
	Vc = -temp_sv1 - temp_sv2;
	/* 60 degree Sector determination */
	if (Va > _IQ(0))
		Sector = 1;
	if (Vb > _IQ(0))
		Sector = Sector + 2;
	if (Vc > _IQ(0))
		Sector = Sector + 4;
	/* X,Y,Z (Va,Vb,Vc) calculations X = Va, Y = Vb, Z = Vc */
	Va = sv->Ubeta;
	Vb = temp_sv1 + temp_sv2;
	Vc = temp_sv1 - temp_sv2;
	/* Sector 0: this is special case for (Ualpha,Ubeta) = (0,0)*/

	switch (Sector)
	{
	case 0:
		sv->Ta = _IQ(0.5);
		sv->Tb = _IQ(0.5);
		sv->Tc = _IQ(0.5);
		break;
	case 1: /*Sector 1: t1=Z and t2=Y (abc ---> Tb,Ta,Tc)*/
		t1 = Vc;
		t2 = Vb;
		sv->Tb = _IQdiv2((_IQ(1)-t1-t2));
		sv->Ta = sv->Tb + t1; /* taon = tbon+t1		*/
		sv->Tc = sv->Ta + t2; /* tcon = taon+t2		*/
		break;
	case 2: /* Sector 2: t1=Y and t2=-X (abc ---> Ta,Tc,Tb)*/
		t1 = Vb;
		t2 = -Va;
		sv->Ta = _IQdiv2((_IQ(1)-t1-t2));
		sv->Tc = sv->Ta + t1; /*  tcon = taon+t1		*/
		sv->Tb = sv->Tc + t2; /*  tbon = tcon+t2		*/
		break;
	case 3: /* Sector 3: t1=-Z and t2=X (abc ---> Ta,Tb,Tc)*/
		t1 = -Vc;
		t2 = Va;
		sv->Ta = _IQdiv2((_IQ(1)-t1-t2));
		sv->Tb = sv->Ta + t1; /*	tbon = taon+t1		*/
		sv->Tc = sv->Tb + t2; /*	tcon = tbon+t2		*/
		break;
	case 4: /* Sector 4: t1=-X and t2=Z (abc ---> Tc,Tb,Ta)*/
		t1 = -Va;
		t2 = Vc;
		sv->Tc = _IQdiv2((_IQ(1)-t1-t2));
		sv->Tb = sv->Tc + t1; /*	tbon = tcon+t1		*/
		sv->Ta = sv->Tb + t2; /*	taon = tbon+t2		*/
		break;
	case 5: /* Sector 5: t1=X and t2=-Y (abc ---> Tb,Tc,Ta)*/
		t1 = Va;
		t2 = -Vb; /*	tbon = (1-t1-t2)/2	*/
		sv->Tb = _IQdiv2((_IQ(1)-t1-t2));
		sv->Tc = sv->Tb + t1; /*	taon = tcon+t2		*/
		sv->Ta = sv->Tc + t2;
		break;
	case 6: /* Sector 6: t1=-Y and t2=-Z (abc ---> Tc,Ta,Tb)*/
		t1 = -Vb;
		t2 = -Vc;
		sv->Tc = _IQdiv2((_IQ(1)-t1-t2));
		sv->Ta = sv->Tc + t1; /*	taon = tcon+t1		*/
		sv->Tb = sv->Ta + t2; /*	tbon = taon+t2		*/
		break;
	}
	/*  Convert the unsigned GLOBAL_Q format (ranged (0,1)) ->.. */
	/* 	..signed GLOBAL_Q format (ranged (-1,1))*/
	sv->Ta = _IQmpy2(sv->Ta-_IQ(0.5));
	sv->Tb = _IQmpy2(sv->Tb-_IQ(0.5));
	sv->Tc = _IQmpy2(sv->Tc-_IQ(0.5));
}

void svpwm_calculate_dutycycle(SV_PWMGEN* v)
{
	/* Compute the timer period (Q0) from the period modulation input (Q15)*/

	int32_t Tmp;
//	int32_t Tmp = (int32_t) v->PeriodMax * (int32_t) v->MfuncPeriod; /* Q15 = Q0*Q15	*/
//	int32_t MPeriod = (int16_t) (Tmp >> 16) + (int16_t) (v->PeriodMax >> 1); /*Q0 = (Q15->Q0)/2 + (Q0/2)*/
	uint16_t MPeriod = v->PeriodMax;

	/*Compute the compare value (Q0) from the related duty cycle ratio (Q15)*/

	Tmp = (int32_t) MPeriod * (int32_t) v->MfuncC1; /* Q15 = Q0*Q15	*/
	v->PWM1out = (int16_t) (Tmp >> 16) + (int16_t) (MPeriod >> 1); /*Q0 = (Q15->Q0)/2 + (Q0/2)*/

	/*Compute the compare value (Q0) from the related duty cycle ratio (Q15)*/

	Tmp = (int32_t) MPeriod * (int32_t) v->MfuncC2; /* Q15 = Q0*Q15	*/
	v->PWM2out = (int16_t) (Tmp >> 16) + (int16_t) (MPeriod >> 1); /*Q0 = (Q15->Q0)/2 + (Q0/2)*/

	/*Compute the compare value (Q0) from the related duty cycle ratio (Q15)*/

	Tmp = (int32_t) MPeriod * (int32_t) v->MfuncC3; /* Q15 = Q0*Q15	*/
	v->PWM3out = (int16_t) (Tmp >> 16) + (int16_t) (MPeriod >> 1); /*Q0 = (Q15->Q0)/2 + (Q0/2)*/
}
