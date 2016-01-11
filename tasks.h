/*
 * TASKS HEADER FILE
 *
 *  Created on: 14/nov/2015
 *      Author: Paolo Sassi
 */

#ifndef TASKS_H_
#define TASKS_H_

#include <stdlib.h>

#include "globals.h"

/* TASK DECLARATIONS */

void *display_task(void *arg);
void *target_task(void *arg);
void *radar_task(void *arg);
void *keyboard_task(void *arg);
void *patriot_task(void *arg);
void *ecs_task(void *arg);

/* GLOBAL FUNCTIONS */

void init();
void waitEnd();
void endProgram();


#endif /* TASKS_H_ */
