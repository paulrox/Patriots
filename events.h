/*
 * EVENTS MANAGEMENT HEADER FILE
 *
 * This file contains the macros used to identify and manipulate
 * the events bitmasks
 *
 *  Created on: 22/dic/2015
 *      Author: paolo
 */

#ifndef EVENTS_H_
#define EVENTS_H_

#include "globals.h"

/* Global Events */
#define END			0x01
#define T_CENTROID	0x02
#define P_CENTROID	0x04
#define PRED_READY	0x08
#define PRED1_READY	0x10
#define PRED2_READY	0x20
#define PRED3_READY	0x40
#define PRED4_READY	0x80

/* Target Events */
#define T1START		0x01
#define T2START		0x02
#define T3START		0x04
#define T4START		0x08
#define STARTED		0x0F
#define T1MISS		0x10
#define T2MISS		0x20
#define T3MISS		0x40
#define T4MISS		0x80
#define MISSED		0xF0

/* Patriot Events */
#define P1FIRE		0x01
#define P2FIRE		0x02
#define P3FIRE		0x04
#define P4FIRE		0x08
#define	FIRED		0x0F
#define P1HIT		0x10
#define P2HIT		0x20
#define P3HIT		0x40
#define P4HIT		0x80
#define HIT			0xF0

/* Bitmasks macros */
#define setEvent(mask, event)	(mask |= event)
#define clearEvent(mask, event)	(mask &= ~event)
#define toggleEvent(mask, event)(mask ^= event)
#define isEvent(mask, event)	((mask & event) == event)
#define isStarted				((t_mask & STARTED) > 0)
#define isFired					((p_mask & FIRED) > 0)
#define freeTargets				((t_mask & STARTED) < STARTED)

extern uint8_t	t_mask;
extern uint8_t	p_mask;
extern uint8_t	evts;

extern void clearEvents();
extern void clearTargetEvents();

#endif /* EVENTS_H_ */
