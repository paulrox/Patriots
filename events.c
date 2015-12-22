/*
 * events.c
 *
 *  Created on: 22/dic/2015
 *      Author: paolo
 */

uint8_t	t_mask;
uint8_t	p_mask;
uint8_t evts;

void clearEvents() {
	t_mask = p_mask = evts = 0;
}
