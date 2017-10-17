/*
 * pmsm_model.h
 *
 *  Created on: 5 Apr, 2017
 *      Author: PhuongLe
 */

#ifndef PMSM_MODEL_H_
#define PMSM_MODEL_H_

#include "IQmathLib.h"
#include <math.h>

#define myQ 20 //IQ precision level for this model
typedef _iq20 _myiq;
#define _myIQ(A) _IQ20(A)
#define _myIQdiv(A,B) _IQ20div(A,B)
#define _myIQmpy(A,B) _IQ20mpy(A,B)
#define _myIQtoIQ(A) _IQ20toIQ(A)
#define _IQtomyIQ(A) _IQtoIQ20(A)

typedef struct {
	_myiq R;
	_myiq Ld;
	_myiq Lq;
	_myiq psiM;
	_myiq J;
	_myiq fr;
	_myiq Ts;
	uint16_t pp;
} PMSM_PARAM;

typedef struct {
	// params
	PMSM_PARAM *param;

	// matrix value
	_myiq phi11;
	_myiq phi12;
	_myiq phi21;
	_myiq phi22;
	_myiq H11;
	_myiq H22;
	_myiq h2;

	// state
	_myiq id;
	_myiq iq;
	_myiq id1;//id(k+1)
	_myiq iq1;//iq(k+1)
	_myiq ud;
	_myiq uq;
	_myiq Me;
	_myiq Mc;
	_myiq Mfr;
	_myiq omega;
	_myiq theta;
	_myiq thetaNorm;//from 0 to 1
} PMSM_MODEL;

void init_model(PMSM_PARAM *p, PMSM_MODEL *m);
void calc_model(PMSM_MODEL *m);

#endif /* PMSM_MODEL_H_ */
