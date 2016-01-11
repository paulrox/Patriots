/*
 * 	TASK LIBRARY for management of periodic tasks
 *
 *  Created on: 30/ott/2015
 *      Author: Paolo Sassi
 */


#include "taskslib.h"

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <sched.h>
#include <math.h>

#include "globals.h"


task_des task_set[TASK_NUM];

/*----------------------------------------------------------------------------------+
 *	create_task(func, period, deadline, prio, index)								|
 *																					|
 *	Creates a new task using SCHED_FIFO and the parameters passed as arguments.		|
 *	Returns 0 if no error occurred													|
 *----------------------------------------------------------------------------------+
 */

void create_task(void *func, uint32_t period, uint32_t deadline, uint32_t prio, uint32_t index)
{
uint32_t err;
pthread_attr_t attr;
struct sched_param par;

	task_set[index].prio = prio;
	task_set[index].period = period;
	task_set[index].deadline = deadline;
	task_set[index].dmiss = 0;
	task_set[index].index = index;

	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	par.sched_priority = prio;
	pthread_attr_setschedparam(&attr, &par);
	err = pthread_create(&task_set[index].tid, &attr, func, &task_set[index]);
	pthread_attr_destroy(&attr);
	if (err < 0) {
		printf("Unable to create task %d\n", index);
		exit(1);
	}
}

/*----------------------------------------------------------------------------------+
 *	set_period(td)																	|
 *																					|
 *	Set the new activation time for the task										|
 *----------------------------------------------------------------------------------+
 */

void set_period(task_des *td)
{
struct timespec t;

	clock_gettime(CLOCK_MONOTONIC, &t);
	time_copy(&(td->at), t);
	time_copy(&(td->dl), t);
	time_add_ms(&(td->at), td->period);
	time_add_ms(&(td->dl), td->deadline);
}

/*----------------------------------------------------------------------------------+
 *	wfp(td)																			|
 *																					|
 *	Called by a task after completing its computations to wait for the next period	|
 *----------------------------------------------------------------------------------+
 */

void wfp(task_des *td)
{
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(td->at), NULL);
	time_add_ms(&(td->at), td->period);
	time_add_ms(&(td->dl), td->deadline);
}

/*----------------------------------------------------------------------------------+
 *	check_deadline(td)																|
 *																					|
 *	Checks if a deadline miss has occurred											|
 *----------------------------------------------------------------------------------+
 */

void check_deadline(task_des *td)
{
struct timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);

	if (time_cmp(now, td->dl) > 0) {
		td->dmiss++;
	}
}

/*----------------------------------------------------------------------------------+
 *	time_copy(td, ts)																|
 *																					|
 *	Copies the timespec components from a source to a destination structure			|
 *----------------------------------------------------------------------------------+
 */

void time_copy(struct timespec *td, struct timespec ts)
{
	td->tv_sec = ts.tv_sec;
	td->tv_nsec = ts.tv_nsec;
}

/*----------------------------------------------------------------------------------+
 *	time_add_ms(t, ms)																|
 *																					|
 *	Adds a specified amount of milliseconds to a timespec data structure			|
 *----------------------------------------------------------------------------------+
 */

void time_add_ms(struct timespec *t, int ms)
{
	t->tv_sec += ms/1000;
	t->tv_nsec += (ms%1000)*1000000;

	if(t->tv_nsec > 1000000000) {
		t->tv_nsec -= 1000000000;
		t->tv_sec += 1;
	}
}

/*----------------------------------------------------------------------------------+
 *	time_cmp(t1, t2)																|
 *																					|															|
 *	Compares two timespec data structure, returning the information of which 		|
 *	contains the higher time value. Returns 1 if t1 contains the higher time value,	|
 *	-1 otherwise 																	|
 *----------------------------------------------------------------------------------+
 */

int8_t time_cmp(struct timespec t1, struct timespec t2)
{
	if (t1.tv_sec > t2.tv_sec) return 1;
	if (t1.tv_sec < t2.tv_sec) return -1;
	if (t1.tv_nsec > t2.tv_nsec) return 1;
	if (t1.tv_nsec < t2.tv_nsec) return -1;

	return 0;
}

/*----------------------------------------------------------------------------------+
 *	cartToPolar(x, y, mod, theta)													|
 *																					|
 *	Converts a point from cartesian coordinates to polar coordinates				|
 *----------------------------------------------------------------------------------+
 */

void cartToPolar(float32_t x, float32_t y, float32_t *mod, float32_t *theta)
{
	if (theta != NULL) *theta = atan2(y, x);
	if (mod != NULL) *mod = sqrt((x * x) + (y * y));
}

/*----------------------------------------------------------------------------------+
 *	frand(min, max)																	|
 *																					|
 *	Generate a pseudo-random floating number in the [min, max] interval				|
 *----------------------------------------------------------------------------------+
 */

float32_t frand(float32_t min, float32_t max)
{
float32_t r;
struct timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);

	/* takes the actual nsec as seed for rand() */
	srand((int32_t)now.tv_nsec);
	r = rand() / (float32_t)RAND_MAX;
	return min + (max - min) * r;

}
