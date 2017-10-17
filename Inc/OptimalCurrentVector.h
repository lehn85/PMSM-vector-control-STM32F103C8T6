/*
 * OptimalCurrentVector.h
 *
 *  Created on: 8 Apr, 2017
 *      Author: PhuongLe
 */

#ifndef OPTIMALCURRENTVECTOR_H_
#define OPTIMALCURRENTVECTOR_H_

#include "IQmathLib.h"

// data to choose id, iq from M, omega
// all in per unit system (pu)
// table index: 0 1/Sc 2/Sc ... Sc/Sc=1
typedef struct {
	int16_t* tblId;//table of id, IQ15. Row is torque, column is speed
	int16_t* tblIq;//table of iq, IQ15. Row is torque, column is speed
	//int16* tblMaxTorque;//array of max torque by speed (size=Sc)
	uint16_t Sc;//width of table - speed point count
	uint16_t Tc;//height of table - torque point count
} LookupTableIdIq;

void init_optimalVectorCurrent(LookupTableIdIq *tbl);
void calc_optimalCurrentVector(LookupTableIdIq *tbl, _iq s, _iq t, _iq* id, _iq* iq);

#endif /* OPTIMALCURRENTVECTOR_H_ */
