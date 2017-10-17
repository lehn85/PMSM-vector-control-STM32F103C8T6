/*
 * svpwm.h
 *
 *  Created on: 21 Jun 2017
 *      Author: PhuongLe
 */

#ifndef LIBFOC_H_
#define LIBFOC_H_

#include <IQmathLib.h>
#include <stdio.h>

// park convert
#define PARK_MACRO(Alpha,Beta,Cosine,Sine,Ds,Qs)	\
	Ds = _IQmpy(Alpha,Cosine) + _IQmpy(Beta,Sine);		\
    Qs = _IQmpy(Beta,Cosine) - _IQmpy(Alpha,Sine);

// ipart convert
#define IPARK_MACRO(Ds,Qs,Cosine,Sine,Alpha,Beta)	\
    Alpha = _IQmpy(Ds,Cosine) - _IQmpy(Qs,Sine);		\
    Beta = _IQmpy(Qs,Cosine) + _IQmpy(Ds,Sine);

#define CLARKE_MACRO(a,b,alpha,beta)											\
alpha = a;													\
beta = _IQmpy((a +_IQmpy2(b)),_IQ(0.57735026918963));	\

// struct for convert
typedef struct
{
	_iq Ualpha; 			// Input: reference alpha-axis phase voltage
	_iq Ubeta;			// Input: reference beta-axis phase voltage
	_iq Ta;				// Output: reference phase-a switching function
	_iq Tb;				// Output: reference phase-b switching function
	_iq Tc;				// Output: reference phase-c switching function
} SVGENDQ;

typedef struct
{
	uint16_t PeriodMax;   // Parameter: PWM Half-Period in CPU clock cycles (Q0)
	//int16_t MfuncPeriod;    // Input: Period scaler (Q15)
	int16_t MfuncC1;        // Input: A duty cycle (Q15)
	int16_t MfuncC2;        // Input: B duty cycle (Q15)
	int16_t MfuncC3;        // Input: C duty cycle (Q15)
	int16_t PWM1out;
	int16_t PWM2out;
	int16_t PWM3out;
} SV_PWMGEN;

/**
 * Calculate Ta,Tb,Tc from Ualpha, Ubeta
 * All are pu [-1;1], IQ24 (by default)
 */
void svpwm_calculate(SVGENDQ* sv);

/**
 * Calculate actual duty cycle for PWM hardware from Ta, Tb, Tc (IQ15)
 * normally: dutycycle = T*periodMax/2+periodMax/2
 */
void svpwm_calculate_dutycycle(SV_PWMGEN* v);

#endif /* LIBFOC_H_ */
