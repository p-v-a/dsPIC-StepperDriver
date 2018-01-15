/* 
 * File:   sine_table.h
 * Author: vladimir
 *
 * Created on 16 January 2018, 12:07 AM
 */
#include <libq.h>

#ifndef SINE_TABLE_H
#define	SINE_TABLE_H

#ifdef	__cplusplus
extern "C" {
#endif

// Sinewave look-up table size and helper constants to rotate table across all IV quadrants.
#define TABLE_SIZE              256
#define TABLE_SIZE_MUL2         (256 * 2)
#define TABLE_SIZE_MUL3         (256 * 3)
#define TABLE_SIZE_MUL4         (256 * 4)    

//Micro-Step table  = 0 to 90 degrees of a cosine in Q16
extern const _Q16 sine_table[TABLE_SIZE];

#ifdef	__cplusplus
}
#endif

#endif	/* SINE_TABLE_H */

