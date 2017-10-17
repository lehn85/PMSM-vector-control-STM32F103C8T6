/*
 * six_step.h
 *
 *  Created on: 25 Jun 2017
 *      Author: PhuongLe
 */

#ifndef SIX_STEP_H_
#define SIX_STEP_H_

#include <stdio.h>
#include <stm32f1xx_hal.h>

typedef struct {
     	uint16_t CmtnPointer;    	// Input: Commutation (or switching) state pointer input (Q0)
   		uint16_t PeriodMax;    	// Parameter: Maximum period (Q0)
   		int16_t DutyFunc;   	    // Input: PWM period modulation input (Q15)
   		uint16_t PwmActive;     	// Parameter: 0 = PWM active low, 1 = PWM active high (0 or 1)
   		TIM_HandleTypeDef* htim;	// Parameter: timer of normal outputs
   		TIM_HandleTypeDef* htim_comp; // Parameter: timer of complementary outputs //maybe null if htim has complementary

} SIXSTEP_PWMGEN;

void SixStep_GeneratePWM(SIXSTEP_PWMGEN* v, uint8_t fwd);

#endif /* SIX_STEP_H_ */
