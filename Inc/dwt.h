/*
 * dwt.h
 *
 *  Created on: 23 Jun 2017
 *      Author: PhuongLe
 *  Copied from: http://embeddedb.blogspot.com.by/2013/10/how-to-count-cycles-on-arm-cortex-m.html
 */

#ifndef DWT_H_
#define DWT_H_

// addresses of registers
volatile uint32_t *DWT_CONTROL = (uint32_t *)0xE0001000;
volatile uint32_t *DWT_CYCCNT = (uint32_t *)0xE0001004;
volatile uint32_t *DEMCR = (uint32_t *)0xE000EDFC;

// enable the use DWT
#define DWT_ENABLE() *DEMCR = *DEMCR | 0x01000000

// Reset cycle counter
#define DWT_RESET_CYCLE_COUNTER() *DWT_CYCCNT = 0

// enable cycle counter
#define DWT_ENABLE_CYCLE_COUNTER() *DWT_CONTROL = *DWT_CONTROL | 1
//#define DISABLE_CYCLE_COUNTER() *DWT_CONTROL = *DWT_CONTROL & (~1);

// number of cycles stored in count variable
#define DWT_CYCLE_COUNTER() *DWT_CYCCNT

#endif /* DWT_H_ */
