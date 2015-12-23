/*
 * events.c
 *
 *  Created on: 22/dic/2015
 *      Author: paolo
 */

#include "events.h"
#include "globals.h"

uint8_t	t_mask;
uint8_t	p_mask;
uint8_t evts;

void clearEvents() {
	t_mask = p_mask = evts = 0;
}

void clearTargetEvents(int32_t index) {
uint8_t mask;
int32_t i;

	mask = 1;
	for (i = 0; i < index; i++) mask <<= 1;

	clearEvent(t_mask, mask);
	clearEvent(p_mask, mask);
	mask <<= 4;
	clearEvent(t_mask, mask);
	clearEvent(p_mask, mask);
	clearEvent(evts, mask);
}
