/*
 * RegulatorCurrent.h
 *
 *  Created on: 6 Apr, 2017
 *      Author: PhuongLe
 */

#ifndef REGULATORCURRENT_H_
#define REGULATORCURRENT_H_

#include "IQmathLib.h"
#include <math.h>

//#define myQ 20 //IQ precision level for this model
//typedef _iq20 _myiq;
//#define _myIQ(A) _IQ20(A)
//#define _myIQdiv(A,B) _IQ20div(A,B)
//#define _myIQmpy(A,B) _IQ20mpy(A,B)
//#define _myIQtoIQ(A) _IQ20toIQ(A)
//#define _IQtomyIQ(A) _IQtoIQ20(A)
//
//typedef struct {
//	_myiq R;
//	_myiq Ld;
//	_myiq Lq;
//	_myiq psiM;
//	_myiq J;
//	_myiq fr;
//	_myiq Ts;
//	uint16_t pp;
//} PMSM_PARAM;

// all is pu [-1;1]
typedef struct  {
	// param
	_iq phi11;
	_iq phi12;
	_iq phi21;
	_iq phi22;
	_iq H11;//H11~
	_iq H22;//H22~ for pu
	_iq h2;//h2~ for pu
	_iq Ts;
	uint16_t pp;

	// input
	_iq id;
	_iq iq;
	_iq idRef;
	_iq iqRef;
	_iq omega;

	// internal
	_iq id1p;//i(k+1) predict
	_iq iq1p;//i(k+1) predict
	_iq udOut1;
	_iq uqOut1;

	// output
	_iq udOut;
	_iq uqOut;
} RegulatorCurrent_FeedbackState;

void init_regulatorCurrent_FS(PMSM_PARAM *p, RegulatorCurrent_FeedbackState *r);
void calc_regulatorCurrent_FS(RegulatorCurrent_FeedbackState *r);

#endif /* REGULATORCURRENT_H_ */
