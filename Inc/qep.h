/*
 * qep.h
 *
 *  Created on: 27 Jun 2017
 *      Author: PhuongLe
 */

#ifndef QEP_H_
#define QEP_H_

#include <IQmathLib.h>
#include "stm32f1xx_hal.h"

typedef struct {
	TIM_TypeDef *tim_encoder;//Parameter: timer input capture encoder
	uint32_t pulsePerRev; //Parameter: number of pulse per revolution
	uint16_t pairPoles; // Parameter: number of pair of poles
	uint16_t syncCount; //Parameter: count will be assigned when hallA-pulse is rising (for sync)
	uint16_t pulsePerElecRev; // internal
	uint16_t pulseXResPerElecRev; //internal
	uint16_t xRes; //Parameter: resolution x2 (TI1 or TI2) or x4 (TI1+TI2)
	uint8_t direction;//output: direction
	_iq angleM; // output: angle rotated depends on A,B counter (0;1) as 0..2pi
	_iq angleE; // output: angle rotated (0;1) electrical as 0..2pi
	_iq cosineE; // cosine of angle E
	_iq sineE; // sine of angle E
	uint32_t mainIRQFreq; //mainIRQ freq
	int32_t speed; // output: speed rpm
} QEPCAP;

// init
void QEPCAP_init(QEPCAP* qep);
// clear counter on Ext interrupt
void QEPCAP_clearCounter(QEPCAP* qep);
// calculate angle from counter
void QEPCAP_process(QEPCAP *qep);

#endif /* QEP_H_ */
