/*
 * EVENTS MANAGEMENT SOURCE CODE
 *
 * This file contains the global bitmasks used in the application
 * and the definition of utility functions for events management.
 *
 *  Created on: 22/dic/2015
 *      Author: Paolo Sassi
 */

#include "events.h"
#include "globals.h"

/* Global events bitmasks */

uint8_t	t_mask;	/* target events bitmask */
uint8_t	p_mask;	/* patriot events bitmask */
uint8_t evts;	/* general events bitmask */

/* Events utility functions */

/*----------------------------------------------------------------------------------+
 *	clearEvents()																	|
 *																					|
 *	Clears all the event bitmasks													|
 *----------------------------------------------------------------------------------+
 */

void clearEvents() {
	t_mask = p_mask = evts = 0;
}

/*----------------------------------------------------------------------------------+
 *	clearTargetEvents(index)														|
 *																					|
 *	Clears the event bits associated to the target 'index'							|
 *----------------------------------------------------------------------------------+
 */

void clearTargetEvents(int32_t index) {
uint8_t mask;
int32_t i;

	mask = 1;	/* 0000 0001 */
	for (i = 0; i < index; i++) mask <<= 1;
	clearEvent(t_mask, mask);
	clearEvent(p_mask, mask);
	mask <<= 4;
	clearEvent(t_mask, mask);
	clearEvent(p_mask, mask);
	clearEvent(evts, mask);
}
