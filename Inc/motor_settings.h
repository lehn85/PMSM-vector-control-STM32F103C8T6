/*
 * motor_settings.h
 *
 *  Created on: 10 Aug 2017
 *      Author: PhuongLe
 */

#ifndef MOTOR_SETTINGS_H_
#define MOTOR_SETTINGS_H_

// Define the PMSM motor parameters
#define RS 		0.8               		// Stator resistance (ohm)
#define RR   	0               		// Rotor resistance (ohm)
#define LS   	0.0012     				// Stator inductance (H)
//#define Ld		0.96e-3
//#define Lq		2.19e-3
#define LR   	0						// Rotor inductance (H)
#define LM   	0						// Magnetizing inductance (H)
#define POLES   8						// Number of poles

// Define the base quantites
#define BASE_VOLTAGE    13.86		    // Base peak phase voltage (volt), maximum measurable DC Bus/sqrt(3)
#define BASE_CURRENT    7            	// Base peak phase current (amp) , maximum measurable peak current
#define BASE_FREQ      	BASE_SPEED/60*POLES/2           	// Base electrical frequency (Hz)
#define BASE_SPEED 		4500		// Base speed 3000 rpm
#define BASE_OMEGA		BASE_SPEED/60*2*PI		// = 3000/60*2*3.14

#endif /* MOTOR_SETTINGS_H_ */
