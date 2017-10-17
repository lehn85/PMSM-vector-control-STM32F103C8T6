/*
 * six_step.c
 *
 *  Created on: 25 Jun 2017
 *      Author: PhuongLe
 */

#include "six_step.h"
#include "stm32f1xx_hal.h"
#include <IQmathLib.h>

#define b_cc1e 1 //1<<0
#define b_cc1p 2 //1<<1
#define b_cc1ne 4 //1<<2
#define b_cc1np 8 //1<<3
#define b_cc2e 16 //1<<4
#define b_cc2p 32 //1<<5
#define b_cc2ne 64 //1<<6
#define b_cc2np 128 //1<<7
#define b_cc3e 256 //1<<8
#define b_cc3p 512 //1<<9
#define b_cc3ne 1024 //1<<10
#define b_cc3np 2048 //1<<11
#define b_cc4e 4096 //1<<12
#define b_cc4p 8192 //1<<13

#define phab b_cc1e+b_cc1p+b_cc2np+b_cc4e+b_cc4p
#define phac b_cc1e+b_cc1p+b_cc3np+b_cc4e+b_cc4p
#define phbc b_cc2e+b_cc2p+b_cc3np+b_cc4e+b_cc4p
#define phba b_cc2e+b_cc2p+b_cc1np+b_cc4e+b_cc4p
#define phca b_cc3e+b_cc3p+b_cc1np+b_cc4e+b_cc4p
#define phcb b_cc3e+b_cc3p+b_cc2np+b_cc4e+b_cc4p

#define ACTIVE_HIGH

#define pwmmode_1 0x6
#define pwmmode_2 0x7
#define force_low 0x4
#define force_high 0x5

#ifdef ACTIVE_HIGH
#define force_inactive force_low
#define force_active force_high
#define pwmmode pwmmode_1
#else
#define force_inactive force_high
#define force_active force_low
#define pwmmode pwmmode_2
#endif

// lookup tables --------------------------------------------------
//// lookup table for timer with extension (TIM1 or TIM8), which has complementary outputs
// in anycase high side is enable, and set between pwm and force inactive
// low side is disable all the time, and with bit OSSR = 1, it is = bit CCxNP
// For active low, need to set others bridge CCxNP=1, also use force_inactive as force_high
// and pwmmode_2 with bit CCxP = 0 (active high) or pwmmode_1 with bit CCxP = 1 (active low)
// For active high, need to set others bridge CCxNP=0, this bridge CCxNP=1, also use force_inactive as force_low
// and pwmmode_1 with bit CCxP = 0 (active high) or pwmmode_2 with bit CCxP = 1 (active low)
#ifdef ACTIVE_HIGH
const unsigned short ccermask_timex[8] =
		{
				0,
				b_cc1e + b_cc2e + b_cc3e + b_cc3np + b_cc4e + b_cc4p, //ac
				b_cc1e + b_cc2e + b_cc3e + b_cc1np + b_cc4e + b_cc4p, //ba
				b_cc1e + b_cc2e + b_cc3e + b_cc3np + b_cc4e + b_cc4p, //bc
				b_cc1e + b_cc2e + b_cc3e + b_cc2np + b_cc4e + b_cc4p, //cb
				b_cc1e + b_cc2e + b_cc3e + b_cc2np + b_cc4e + b_cc4p, //ab
				b_cc1e + b_cc2e + b_cc3e + b_cc1np + b_cc4e + b_cc4p, //ca
				0
		};
#else
const unsigned short ccermask_timex[8] =
{
	0,
	b_cc1e + b_cc2e + b_cc3e + b_cc1np + b_cc2np + b_cc4e + b_cc4p, //ac
	b_cc1e + b_cc2e + b_cc3e + b_cc2np + b_cc3np + b_cc4e + b_cc4p,//ba
	b_cc1e + b_cc2e + b_cc3e + b_cc1np + b_cc2np + b_cc4e + b_cc4p,//bc
	b_cc1e + b_cc2e + b_cc3e + b_cc1np + b_cc3np + b_cc4e + b_cc4p,//cb
	b_cc1e + b_cc2e + b_cc3e + b_cc1np + b_cc3np + b_cc4e + b_cc4p,//ab
	b_cc1e + b_cc2e + b_cc3e + b_cc2np + b_cc3np + b_cc4e + b_cc4p,//ca
	0
};
#endif

const unsigned short timex_ccmr1_mask[8] =
		{
				0,
				(pwmmode << TIM_CCMR1_OC1M_Pos) + (force_inactive << TIM_CCMR1_OC2M_Pos), //ac//1
				(force_inactive << TIM_CCMR1_OC1M_Pos) + (pwmmode << TIM_CCMR1_OC2M_Pos), //ba//2
				(force_inactive << TIM_CCMR1_OC1M_Pos) + (pwmmode << TIM_CCMR1_OC2M_Pos), //bc//3
				(force_inactive << TIM_CCMR1_OC1M_Pos) + (force_inactive << TIM_CCMR1_OC2M_Pos), //cb//4
				(pwmmode << TIM_CCMR1_OC1M_Pos) + (force_inactive << TIM_CCMR1_OC2M_Pos), //ab//5
				(force_inactive << TIM_CCMR1_OC1M_Pos) + (force_inactive << TIM_CCMR1_OC2M_Pos), //ca//6
				0,
		};

// phase C high side (channel3 as C)
const unsigned short timex_ccmr2_mask[8] =
		{
				0,
				(force_inactive << TIM_CCMR2_OC3M_Pos) + (pwmmode_1 << TIM_CCMR2_OC4M_Pos), //ac
				(force_inactive << TIM_CCMR2_OC3M_Pos) + (pwmmode_1 << TIM_CCMR2_OC4M_Pos), //ba
				(force_inactive << TIM_CCMR2_OC3M_Pos) + (pwmmode_1 << TIM_CCMR2_OC4M_Pos), //bc
				(pwmmode << TIM_CCMR2_OC3M_Pos) + (pwmmode_1 << TIM_CCMR2_OC4M_Pos), //cb
				(force_inactive << TIM_CCMR2_OC3M_Pos) + (pwmmode_1 << TIM_CCMR2_OC4M_Pos), //ab
				(pwmmode << TIM_CCMR2_OC3M_Pos) + (pwmmode_1 << TIM_CCMR2_OC4M_Pos), //ca
				0,

		};

///0 001    010   011   100   101   110

////// lookup table for tim1 - pwm output to high side, tim2 - pwm output to lowside
// phase A,B high side (channel 1 as A, channel 2 as B)
const unsigned short tim1_ccmr1_mask[8] =
		{
				0,
				(pwmmode_1 << TIM_CCMR1_OC1M_Pos) + (force_inactive << TIM_CCMR1_OC2M_Pos), //ac//1
				(force_inactive << TIM_CCMR1_OC1M_Pos) + (pwmmode_1 << TIM_CCMR1_OC2M_Pos), //ba//2
				(force_inactive << TIM_CCMR1_OC1M_Pos) + (pwmmode_1 << TIM_CCMR1_OC2M_Pos), //bc//3
				(force_inactive << TIM_CCMR1_OC1M_Pos) + (force_inactive << TIM_CCMR1_OC2M_Pos), //cb//4
				(pwmmode_1 << TIM_CCMR1_OC1M_Pos) + (force_inactive << TIM_CCMR1_OC2M_Pos), //ab//5
				(force_inactive << TIM_CCMR1_OC1M_Pos) + (force_inactive << TIM_CCMR1_OC2M_Pos), //ca//6
				0,
		};

// phase C high side (channel3 as C)
const unsigned short tim1_ccmr2_mask[8] =
		{
				0,
				(force_inactive << TIM_CCMR2_OC3M_Pos), //ac
				(force_inactive << TIM_CCMR2_OC3M_Pos), //ba
				(force_inactive << TIM_CCMR2_OC3M_Pos), //bc
				(pwmmode_1 << TIM_CCMR2_OC3M_Pos), //cb
				(force_inactive << TIM_CCMR2_OC3M_Pos), //ab
				(pwmmode_1 << TIM_CCMR2_OC3M_Pos), //ca
				0,

		};

// phase A,B low side
const unsigned short tim2_ccmr1_mask[8] =
		{
				0,
				(force_inactive << TIM_CCMR1_OC1M_Pos) + (force_inactive << TIM_CCMR1_OC2M_Pos), //ac
				(force_active << TIM_CCMR1_OC1M_Pos) + (force_inactive << TIM_CCMR1_OC2M_Pos), //ba
				(force_inactive << TIM_CCMR1_OC1M_Pos) + (force_inactive << TIM_CCMR1_OC2M_Pos), //bc
				(force_inactive << TIM_CCMR1_OC1M_Pos) + (force_active << TIM_CCMR1_OC2M_Pos), //cb
				(force_inactive << TIM_CCMR1_OC1M_Pos) + (force_active << TIM_CCMR1_OC2M_Pos), //ab
				(force_active << TIM_CCMR1_OC1M_Pos) + (force_inactive << TIM_CCMR1_OC2M_Pos), //ca
				0,

		};

// phase C low side (tim2) + pwm for channel 4
const unsigned short tim2_ccmr2_mask[8] =
		{
				0,
				(force_active << TIM_CCMR2_OC3M_Pos) + (pwmmode_1 << TIM_CCMR2_OC4M_Pos), //ac
				(force_inactive << TIM_CCMR2_OC3M_Pos) + (pwmmode_1 << TIM_CCMR2_OC4M_Pos), //ba
				(force_active << TIM_CCMR2_OC3M_Pos) + (pwmmode_1 << TIM_CCMR2_OC4M_Pos), //bc
				(force_inactive << TIM_CCMR2_OC3M_Pos) + (pwmmode_1 << TIM_CCMR2_OC4M_Pos), //cb
				(force_inactive << TIM_CCMR2_OC3M_Pos) + (pwmmode_1 << TIM_CCMR2_OC4M_Pos), //ab
				(force_inactive << TIM_CCMR2_OC3M_Pos) + (pwmmode_1 << TIM_CCMR2_OC4M_Pos), //ca
				0,

		};

uint16_t lastCmd = 0;
uint16_t lastDuty = 0;

void SixStep_GeneratePWM(SIXSTEP_PWMGEN* v, uint8_t fwd)
{
	uint16_t d = v->DutyFunc * v->PeriodMax >> 15;

	// if htim has complementary outputs, and we don't use other timer as complementary outputs
	if (v->htim_comp == 0)
	{
		// forward
		if (v->CmtnPointer != lastCmd)
		{
			uint8_t pointer = v->CmtnPointer;
			if (!fwd)
				pointer = ~pointer & 0x7;

			v->htim->Instance->CCER = ccermask_timex[pointer]; //set pwm according to cmtnpointer

			// change mode for high side. CCMR1 - A,B; CCMR2 - C
			v->htim->Instance->CCMR1 &= (~TIM_CCMR1_OC1M_Msk) + (~TIM_CCMR1_OC2M_Msk); // clear mode bit
			v->htim->Instance->CCMR2 &= (~TIM_CCMR2_OC3M_Msk) + (~TIM_CCMR2_OC4M_Msk); // clear mode bit
			v->htim->Instance->CCMR1 |= timex_ccmr1_mask[pointer];
			v->htim->Instance->CCMR2 |= timex_ccmr2_mask[pointer];
			//v->htim->Instance->SR |= TIM_SR_COMIF;

			//save last pointer so not set again
			lastCmd = v->CmtnPointer;
		}

		if (d != lastDuty)
		{
			__HAL_TIM_SET_COMPARE(v->htim, TIM_CHANNEL_1, d);
			__HAL_TIM_SET_COMPARE(v->htim, TIM_CHANNEL_2, d);
			__HAL_TIM_SET_COMPARE(v->htim, TIM_CHANNEL_3, d);
			lastDuty = d;
		}
	}
	else //use 2 timer, 1 as normal, 1 as complementary output
	{
		if (v->CmtnPointer != lastCmd)
		{
			uint8_t pointer = v->CmtnPointer;
			if (!fwd)
				pointer = ~pointer & 0x7;

			// change mode for high side. CCMR1 - A,B; CCMR2 - C
			v->htim->Instance->CCMR1 &= (~TIM_CCMR1_OC1M_Msk) + (~TIM_CCMR1_OC2M_Msk); // clear mode bit
			v->htim->Instance->CCMR2 &= (~TIM_CCMR2_OC3M_Msk) + (~TIM_CCMR2_OC4M_Msk); // clear mode bit
			v->htim->Instance->CCMR1 |= tim1_ccmr1_mask[pointer];
			v->htim->Instance->CCMR2 |= tim1_ccmr2_mask[pointer];

			// change mode for low side. CCMR1 - A,B; CCMR2 - C
			v->htim_comp->Instance->CCMR1 &= (~TIM_CCMR1_OC1M_Msk) + (~TIM_CCMR1_OC2M_Msk); // clear mode bit
			v->htim_comp->Instance->CCMR2 &= (~TIM_CCMR2_OC3M_Msk) + (~TIM_CCMR2_OC4M_Msk); // clear mode bit
			v->htim_comp->Instance->CCMR1 |= tim2_ccmr1_mask[pointer];
			v->htim_comp->Instance->CCMR2 |= tim2_ccmr2_mask[pointer];

			//save last pointer so not set again
			lastCmd = v->CmtnPointer;
		}

		if (d != lastDuty)
		{
			__HAL_TIM_SET_COMPARE(v->htim, TIM_CHANNEL_1, d);
			__HAL_TIM_SET_COMPARE(v->htim, TIM_CHANNEL_2, d);
			__HAL_TIM_SET_COMPARE(v->htim, TIM_CHANNEL_4, d); //channel 4 as C, pin for channel 3 is busted

			__HAL_TIM_SET_COMPARE(v->htim_comp, TIM_CHANNEL_1, d);
			__HAL_TIM_SET_COMPARE(v->htim_comp, TIM_CHANNEL_2, d);
			__HAL_TIM_SET_COMPARE(v->htim_comp, TIM_CHANNEL_3, d);

			lastDuty = d;
		}
	}
}
