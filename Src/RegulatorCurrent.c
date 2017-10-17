/*
 * RegulatorCurrent.c
 *
 *  Created on: 6 Apr, 2017
 *      Author: PhuongLe
 */

#include <stdio.h>
#include "main.h"
#include "pmsm_model.h"
#include "RegulatorCurrent.h"
#include "motor_settings.h"

void init_regulatorCurrent_FS(PMSM_PARAM *p, RegulatorCurrent_FeedbackState *r){
	uint16_t maxOmega = p->pp * BASE_OMEGA;

	r->phi11 = _myIQtoIQ(_myIQ(1) - p->Ts * p->R / p->Ld);
	r->phi12 = _myIQtoIQ(p->Ts * p->Lq / p->Ld * maxOmega);
	r->phi21 = _myIQtoIQ(-p->Ts * p->Ld / p->Lq * maxOmega);
	r->phi22 = _myIQtoIQ(_myIQ(1) - p->Ts * p->R / p->Lq);
	r->H11 = _myIQtoIQ(_myIQdiv(p->Ts, p->Ld) * BASE_VOLTAGE / BASE_CURRENT);
	r->H22 = _myIQtoIQ(_myIQdiv(p->Ts, p->Lq) * BASE_VOLTAGE / BASE_CURRENT);
	r->h2 = _myIQtoIQ(-p->Ts * p->psiM / p->Lq * maxOmega / BASE_CURRENT);

//	r->phi11 = _IQ(1 - p->Ts * p->R / p->Ld);
//	r->phi12 = _IQ(p->Ts * p->Lq / p->Ld * maxOmega);
//	r->phi21 = _IQ(-p->Ts * p->Ld / p->Lq * maxOmega);
//	r->phi22 = _IQ(1 - p->Ts * p->R / p->Lq);
//	r->H11 = _IQ(p->Ts / p->Ld * BASE_VOLTAGE / BASE_CURRENT);
//	r->H22 = _IQ(p->Ts / p->Lq * BASE_VOLTAGE / BASE_CURRENT);
//	r->h2 = _IQ(-p->Ts * p->psiM / p->Lq * maxOmega / BASE_CURRENT);

	r->Ts = _myIQtoIQ(p->Ts);
	r->pp = p->pp;
	r->id = 0;
	r->iq = 0;
	r->id1p = 0;
	r->iq1p = 0;
	r->udOut = 0;
	r->uqOut = 0;
	r->udOut1 = 0;
	r->uqOut1 = 0;
	r->omega = 0;
	r->idRef = 0;
	r->iqRef = 0;
}

void calc_regulatorCurrent_FS(RegulatorCurrent_FeedbackState *r){
	_iq um;
	// update u(k)
	r->udOut = r->udOut1;
	r->uqOut = r->uqOut1;
	// limiting u(k)
	um = _IQsqrt(_IQmpy(r->udOut , r->udOut) + _IQmpy(r->uqOut , r->uqOut));
	if (um > _IQ(1)) {
		r->udOut = _IQdiv(r->udOut , um);
		r->uqOut = _IQdiv(r->uqOut , um);
	}

	//predict i(k+1) from i(k),u(k), omega
	r->id1p = _IQmpy(r->phi11 , r->id) + _IQmpy(_IQmpy(r->omega , r->phi12) , r->iq) + _IQmpy(r->H11 , r->udOut);
	r->iq1p = _IQmpy(_IQmpy(r->omega , r->phi21) , r->id) + _IQmpy(r->phi22 , r->iq) + _IQmpy(r->H22 , r->uqOut) + _IQmpy(r->omega , r->h2);

	// calculate ud(k+1)
	r->udOut1 = _IQdiv(r->idRef - _IQmpy(r->phi11 , r->id1p) - _IQmpy(_IQmpy(r->omega , r->phi12) , r->iq1p) , r->H11);
	r->uqOut1 = _IQdiv(r->iqRef - _IQmpy(_IQmpy(r->omega , r->phi21) , r->id1p) - _IQmpy(r->phi22 , r->iq1p) - _IQmpy(r->omega , r->h2) , r->H22);
}
