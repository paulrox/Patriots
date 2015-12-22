/*
 * events.h
 *
 *  Created on: 22/dic/2015
 *      Author: paolo
 */

#ifndef EVENTS_H_
#define EVENTS_H_

#define END			0x01
#define T_CENTROID	0x02
#define P_CENTROID	0x04
#define PRED_READY	0x08

#define T1START		0x01
#define T2START		0x02
#define T3START		0x04
#define T4START		0x08
#define START		0x0F

#define T1MISS		0x10
#define T2MISS		0x20
#define T3MISS		0x40
#define T4MISS		0x80
#define MISSED		0xF0

#define P1FIRE		0x01
#define P2FIRE		0x02
#define P3FIRE		0x04
#define P4FIRE		0x08

#define setEvent(mask, event)	(mask |= event)
#define clearEvent(mask, event)	(mask &= !event)
#define isEvent(mask, event)	(mask & event)

extern uint8_t	t_mask;
extern uint8_t	p_mask;
extern uint8_t	evts;

extern void clearEvents();

#endif /* EVENTS_H_ */
