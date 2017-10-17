/*
 * pmsm_model.c
 *
 *  Created on: 5 Apr, 2017
 *      Author: PhuongLe
 */

#include <stdio.h>
#include "main.h"
#include "IQmathLib.h"
#include <math.h>
#include "motor_settings.h"
#include "pmsm_model.h"

void init_model(PMSM_PARAM *p, PMSM_MODEL *m){
	m->param = p;

	m->phi11 = _myIQ(1) - p->Ts * p->R / p->Ld;
	m->phi12 = p->Ts * p->Lq / p->Ld;
	m->phi21 = - p->Ts * p->Ld / p->Lq;
	m->phi22 = _myIQ(1) - p->Ts * p->R / p->Lq;
	m->H11 = _myIQdiv(p->Ts, p->Ld);//((p->Ts << myQ) / p->Ld);
	m->H22 = _myIQdiv(p->Ts, p->Lq);//((p->Ts << myQ) / p->Lq);
	m->h2 = - p->Ts * p->psiM / p->Lq;

	m->id = 0;
	m->iq = 0;
	m->id1	= 0;
	m->iq1 = 0;
	m->ud = 0;
	m->uq = 0;
	m->omega = 0;
	m->theta = 0;
	m->Mc = 0;
}

// call this every loop (with sample time Ts)
void calc_model(PMSM_MODEL *m){
	_myiq dw;
	_myiq we;
	// update id(k),iq(k)
	m->id = m->id1;
	m->iq = m->iq1;

	// electric model: calc id(k+1), iq(k+1)
	we = m->omega * m->param->pp;
	m->id1 = _myIQmpy(m->phi11 , m->id) + _myIQmpy(_myIQmpy(we,m->phi12), m->iq) + _myIQmpy(m->H11, m->ud);
	m->iq1 = _myIQmpy(_myIQmpy(we, m->phi21) , m->id) + _myIQmpy(m->phi22 , m->iq) + _myIQmpy(m->H22, m->uq) + _myIQmpy(we , m->h2);

	// mechanical model calc with id(k), iq(k)
	m->Me = 3*m->param->pp/2*(_myIQmpy(m->param->psiM , m->iq) + _myIQmpy(_myIQmpy(m->param->Ld - m->param->Lq , m->id) ,m->iq));
	m->Mfr = _myIQmpy(m->param->fr , m->omega);

	// delta omega (change of speed)
	dw = _myIQdiv(_myIQmpy(m->Me - m->Mc - m->Mfr , m->param->Ts) , m->param->J);
	m->omega+=dw;

	// change of angle
	m->theta +=_myIQmpy(m->omega , m->param->Ts);
	m->thetaNorm = _myIQdiv(m->theta % _myIQ(2 * PI) , _myIQ(2*PI));
}
