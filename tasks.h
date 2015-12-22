/*
 * TASKS HEADER FILE
 *
 *  Created on: 14/nov/2015
 *      Author: paolo
 */

#ifndef TASKS_H_
#define TASKS_H_

#include <stdlib.h>

#include "globals.h"

/* GLOBALS VARIABLES */

extern uint8_t end;	/* used to notify the end of the application */

/* TASK DECLARATIONS */

void *display_task(void *arg);
void *target_task(void *arg);
void *radar_task(void *arg);
void *parse_keyboard(void *arg);
void *patriot_task(void *arg);
void *ecs_task(void *arg);

/* GLOBAL FUNCTIONS */

void init();
void endProgram();


#endif /* TASKS_H_ */
