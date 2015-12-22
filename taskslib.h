/*
 * patriots_lib.h
 *
 *  Created on: 30/ott/2015
 *      Author: paolo
 */

#ifndef TASKSLIB_H_
#define TASKSLIB_H_

/*	+---------------------------------------------------------------------------------+
 *	|																				  |
 *	|							LIST OF INCLUDED FILES								  |
 *	|																				  |
 *	+---------------------------------------------------------------------------------+
 */


#include <stdlib.h>
#include <time.h>

#include "globals.h"

/* Task management data structure */
typedef struct _task_des {
	pthread_t tid;			/* Task identifier */
	uint32_t prio;			/* Task priority */
	uint32_t period;		/* Task period */
	uint32_t deadline;		/* Task deadline */
	struct timespec at;		/* Task last activation time */
	struct timespec dl;		/* Task absolute deadline */
	uint32_t dmiss;			/* Task deadline misses */
} task_des;

extern task_des task_set[TASK_NUM];

/* Task management functions */
uint8_t create_task(void *func, uint32_t period, uint32_t deadline, uint32_t prio, uint32_t index);
void set_period(task_des *td);
void wfp(task_des *td);
void check_deadline(task_des *td);

/* Time related funtions */
void time_copy(struct timespec *td, struct timespec ts);
void time_add_ms(struct timespec *t, int ms);
int8_t time_cmp(struct timespec t1, struct timespec t2);
void get_time();

/* Utility functions */
void cartToPolar(float32_t x, float32_t y, float32_t *mod, float32_t *theta);
float32_t frand(float32_t min, float32_t max);

#endif /* TASKSLIB_H_ */
