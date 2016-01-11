/*
 * TASKS SOURCE CODE
 *
 * This file contains tasks code and some auxiliary functions used
 * by them.
 *
 *  Created on: 14/nov/2015
 *      Author: Paolo Sassi
 */

#include "tasks.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <allegro.h>

#include "globals.h"
#include "graphics.h"
#include "taskslib.h"
#include "events.h"


/* MUTEXES DECLARATIONS */
pthread_mutex_t	sim_mutex;	/* mutex for accessing simulation status */
pthread_mutex_t	t_mutex;	/* mutex for accessing target status */
pthread_mutex_t	p_mutex;	/* mutex for accessing patriot status */
pthread_mutex_t	r_mutex;	/* mutex for accessing radar scan data
 	 	 	 	 	 	 	 * and prediction data */

/* number of times the target is found inside the radar area */
uint8_t	engaged_cycle[MAX_TARGETS];
float32_t pos_f, v_f, a_f;			/* filter parameters */
float32_t coll_theta[MAX_TARGETS];	/* intercept angle */
coords r_coords[MAX_TARGETS];
stat p_stat[MAX_TARGETS], t_stat[MAX_TARGETS];
filter_stat f_stat[MAX_TARGETS];
sim_stats ss;

/* LOCAL FUNCTIONS DECLARATION */
static int32_t findFreeTarget();
static void filterStats(int32_t x, int32_t y, float32_t per, filter_stat *fs);
static uint8_t computeIntercept(filter_stat t, stat p, int32_t index);
static void cleanTargetStats(int32_t index);
static void cleanPatriotStats(int32_t index);
static void cleanSimStats();
static void getCartStats();

/*	+-------------------------------------------------------------------------+
 *	|																		  |
 *	|							 GLOBAL FUNCTIONS    			    		  |
 *	|																		  |
 *	+-------------------------------------------------------------------------+
 */

/*----------------------------------------------------------------------------+
 *	init()																	  |
 *																			  |
 *	Initializes the graphics related stuff, the mutexes, 					  |
 *	the physical status of the system and the events variables.				  |
 *----------------------------------------------------------------------------+
 */

void init()
{
int32_t i;
	/* initialize the graphic module */
	initGraphics();

	/* initialize all the mutexes before using them */
	pthread_mutex_init(&t_mutex, NULL);
	pthread_mutex_init(&sim_mutex, NULL);
	pthread_mutex_init(&r_mutex, NULL);
	pthread_mutex_init(&p_mutex, NULL);

	/* clean the application data structures */
	clearEvents();
	cleanSimStats();
	for (i = 0; i < MAX_TARGETS; i++) {
		cleanTargetStats(i);
		cleanPatriotStats(i);
	}
}

/*----------------------------------------------------------------------------------+
 *	waitEnd()																		|
 *																					|
 *	Waits the end of the initial task set											|
 *----------------------------------------------------------------------------------+
 */

void waitEnd()
{
int32_t i, ret;

	for (i = 0; i <= ECS_INDEX; i++) {
		ret = pthread_join(task_set[i].tid, NULL);
		if (ret != 0) {
			printf("Error during join on thread %d", i);
			exit(1);
		}
	}
}

/*----------------------------------------------------------------------------+
 *	endProgram()															  |
 *																			  |
 *	Terminates the allegro stuff and destroys the mutexes					  |
 *----------------------------------------------------------------------------+
 */

void endProgram()
{
	endGraphics();

	pthread_mutex_destroy(&t_mutex);
	pthread_mutex_destroy(&sim_mutex);
	pthread_mutex_destroy(&r_mutex);
	pthread_mutex_destroy(&p_mutex);
}

/*	+-------------------------------------------------------------------------+
 *	|																		  |
 *	|							    TASKS CODE     						      |
 *	|																		  |
 *	+-------------------------------------------------------------------------+
 */

/* DISPLAY TASK
 *
 * This is the only task which is able to draw on the screen.
 * It gets the data to draw the objects from shared data structures
 */

void * display_task(void *arg)
{
task_des *td;
sim_stats sim;
stat *t, *p;
filter_stat *pred;
coords *r;
uint8_t dmiss[TASK_NUM], i, tm_tmp, pm_tmp;

	td = (task_des*)arg;

	set_period(td);
	/* loop until application end */
	while (!isEvent(evts, END)) {

		/* Copy all the shared data structures */
		pthread_mutex_lock(&t_mutex);
		t = t_stat;
		pthread_mutex_unlock(&t_mutex);

		pthread_mutex_lock(&p_mutex);
		p = p_stat;
		pthread_mutex_unlock(&p_mutex);

		pthread_mutex_lock(&r_mutex);
		r = r_coords;
		pred = f_stat;
		pthread_mutex_unlock(&r_mutex);

		pthread_mutex_lock(&sim_mutex);
		sim = ss;
		pthread_mutex_unlock(&sim_mutex);

		for (i = 0; i < TASK_NUM; i++) {
			dmiss[i] = task_set[i].dmiss;
		}
		drawTaskStats(dmiss);
		drawSimStats(sim, pos_f, v_f, a_f);
		drawPatriotStats(p_mask, evts);
		if (isStarted) {	/* at least one target spawned on the screen */
			tm_tmp = 1;		/* set the bitmask to 0000 0001 */
			for (i = 0; i < MAX_TARGETS; i++) {
				if (isEvent(t_mask, tm_tmp)) {	/* check which target */
					drawTarget(realToAllegX(t[i].x), realToAllegY(t[i].y),
							t[i].v_theta, i);
					if (isEvent(evts, T_CENTROID)) {
						drawCentroid(r[i].x, r[i].y, RADAR_CENTROID);
					}
				}
				tm_tmp <<= 1;	/* left shift the bitmask by one position */
			}
		}
		if (isFired){	/* at least one patriot has been fired */
			pm_tmp = 1;	/* set the bitmask to 0000 0001 */
			for (i = 0; i < MAX_PATRIOTS; i++) {
				if (isEvent(p_mask, pm_tmp)) {	/* check which patriot */
					drawPatriot(realToAllegX(p[i].x), realToAllegY(p[i].y),
							p[i].v_theta);
					if (isEvent(evts, P_CENTROID)) {
						drawCentroid(realToAllegX(pred[i].xf),
								realToAllegY(pred[i].yf), PRED_CENTROID);
					}
				}
				pm_tmp <<= 1;	/* left shift the bitmask by one position */
			}
			tm_tmp = pm_tmp = 1;	/* reset the bitmasks */
			for (i = 0; i < MAX_PATRIOTS; i++) {
				if (isEvent(t_mask, tm_tmp) && isEvent(p_mask, pm_tmp) &&
						checkCollision(t[i], p[i])) {
					/* a collision has occurred */
					pm_tmp <<= 4;
					setEvent(p_mask, pm_tmp);	/* set target hit event */
					pm_tmp >>= 4;
					clearEvent(t_mask, tm_tmp);	/* clear target spawn event */
					clearEvent(p_mask, pm_tmp);	/* clear patriot fired event */
					/* update simulation statistics */
					pthread_mutex_lock(&sim_mutex);
					ss.t_hit++;
					ss.t_hitratio = (float32_t)ss.t_hit / ss.t_fired;
					pthread_mutex_unlock(&sim_mutex);
				}
				tm_tmp <<= 1;
				pm_tmp <<= 1;
			}
		}
		updateScreen();	/* update the simulation area of the screen */
		updateStats();	/* update the statistics area of the screen */

		check_deadline(td);

		wfp(td);
	}
	pthread_exit(NULL);
}

/* TARGET TASK
 *
 * This task computes the equations of motion for the enemy target and
 * updates the associated data structure.
 */

void * target_task(void *arg)
{
task_des *td;
float32_t	dt, dx, dy, vx, vy, ax, ay, da;
int32_t index;
uint8_t i, evt_mask, tmp_mask;

	/* update simulation statistics */
	pthread_mutex_lock(&sim_mutex);
	ss.t_fired++;
	pthread_mutex_unlock(&sim_mutex);
	td = (task_des*)arg;
	dt = TSCALE*(float32_t)td->period/1000;
	index = td->index - TARGET_INDEX;	/* get the target index */
	evt_mask = 1;
	/* compute the target event bitmask */
	for (i = 0; i < index; i++) evt_mask <<= 1;
	/* compute the initial speed vector angle in order
	 * to hit the middle of the city */
	pthread_mutex_lock(&t_mutex);
	/* horizontal spacing between the target and the middle of the city */
	dx = ((BOX_WIDTH / 2) * SCALE - t_stat[index].x);
	/* vertical spacing, always equal to y coordinate */
	dy = -t_stat[index].y;
	t_stat[index].v_theta = atan2f(dy, dx);
	da = frand((float32_t)-T_DA, (float32_t)T_DA);
	/* sum a random angle to the exact one */
	t_stat[index].v_theta += degToRad(da);
	pthread_mutex_unlock(&t_mutex);

	set_period(td);
	/* loop until application end and target is fired */
	while (!isEvent(evts, END) && isEvent(t_mask, evt_mask)) {

		pthread_mutex_lock(&t_mutex);
		/* Compute the target acceleration */
		ax = t_stat[index].a * cos(t_stat[index].a_theta);
		ay = t_stat[index].a * sin(t_stat[index].a_theta);
		/* Compute the target speed */
		vx = ax * dt + t_stat[index].v * cos(t_stat[index].v_theta);
		vy = ay * dt + t_stat[index].v * sin(t_stat[index].v_theta);
		/* Update the speed modulus */
		t_stat[index].v = sqrt((vx*vx) + (vy*vy));
		/* Update the target speed angle according to the new speed */
		t_stat[index].v_theta = atan2f(vy, vx);
		/* Update the target position */
		t_stat[index].x += vx * dt + ax * dt * dt / 2;
		t_stat[index].y += vy * dt + ay * dt * dt / 2;
		/* Check if the target reached the ground */
		if (t_stat[index].y <= CITY_COLLISION_Y) {
			/* Restart the simulation */
			clearEvent(t_mask, evt_mask);	/* clear target spawned event */
			tmp_mask = evt_mask << 4;
			setEvent(t_mask, tmp_mask);		/* set target miss event */
			/* update simulation statistics */
			pthread_mutex_lock(&sim_mutex);
			ss.t_missed++;
			ss.t_hitratio = (float32_t)ss.t_hit / ss.t_fired;
			pthread_mutex_unlock(&sim_mutex);
		}
		pthread_mutex_unlock(&t_mutex);

		check_deadline(td);

		wfp(td);
	}
	pthread_exit(NULL);
}

/* RADAR TASK
 *
 * Scans the area near the radar and stores the information into a buffer
 */

void * radar_task(void *arg)
{
task_des *td;
int32_t xc, yc, i;
uint8_t ret, mask;

	td = (task_des*)arg;

	set_period(td);
	/* loop until application end */
	while (!isEvent(evts, END)) {
		mask = 1;	/* set the bitmask to 0000 0001 */
		/* Scan the area looking for the targets */
		for (i = 0; i < MAX_TARGETS; i++) {
			if (isEvent(t_mask, mask)) {	/* target spawned */
				ret = scanArea(&xc, &yc, i);
				if (ret) {	/* target found */
					engaged_cycle[i]++;
					/* Update the target centroid coordinates */
					pthread_mutex_lock(&r_mutex);
					r_coords[i].x = xc;
					r_coords[i].y = yc;
					pthread_mutex_unlock(&r_mutex);
				}
			}
			mask <<= 1;
		}
		check_deadline(td);

		wfp(td);
	}
	pthread_exit(NULL);
}

/* KEYBOARD TASK
 *
 * Checks if any keyboard button has been pressed.
 */

void * keyboard_task(void *arg)
{
task_des *td;
int32_t free;

	td = (task_des*)arg;

	set_period(td);
	/* loop until application end */
	while (!isEvent(evts, END)) {
		/* key 'SPACEBAR' spawns a new target on the screen*/
		if (key[KEY_SPACE]) {
			if (freeTargets){
				/* search the index of a free target structure */
				free = findFreeTarget();
				/* create a new target task */
				create_task(target_task, 50, 50, 32, TARGET_INDEX + free);
			}
		}
		/* key 'C' shows the targets centroid computed from
		 * radar coordinates */
		if (key[KEY_C] && isStarted) {
			toggleEvent(evts, T_CENTROID);
		}
		/* key 'P' shows the targets centroid computed from
		 * predicted coordinates */
		if (key[KEY_P] && isStarted) {
			toggleEvent(evts, P_CENTROID);
		}
		/* key 'R' reset the simulation */
		if (key[KEY_R] && isStarted) {
			cleanSimStats();
			t_mask = 0;
			p_mask = 0;
		}
		/* keys '2' and '1' modify the position filter */
		if (key[KEY_2]) {
			if (pos_f < FILTER_MAX) pos_f += FILTER_STEP;
		}
		if (key[KEY_1]) {
			if (pos_f > FILTER_MIN) pos_f -= FILTER_STEP;
		}
		/* keys '4' and '3' modify the speed filter */
		if (key[KEY_4]) {
			if (v_f < FILTER_MAX) v_f += FILTER_STEP;
		}
		if (key[KEY_3]) {
			if (v_f > FILTER_MIN) v_f -= FILTER_STEP;
		}
		/* keys '6' and '5' modify the acceleration filter */
		if (key[KEY_6]) {
			if (a_f < FILTER_MAX) a_f += FILTER_STEP;
		}
		if (key[KEY_5]) {
			if (a_f > FILTER_MIN) a_f -= FILTER_STEP;
		}
		/* key 'ESC' quits the program */
		if (key[KEY_ESC]) setEvent(evts, END);
		check_deadline(td);

		wfp(td);
	}
	pthread_exit(NULL);
}

/* ECS TASK
 *
 * Analyzes the radar scanned image, performs computations to predict
 * its trajectory and fires the patriot missile.
 */

void * ecs_task(void *arg)
{
task_des *td;
coords *r;
stat tmp;
filter_stat *s;
int32_t i;
uint8_t mask, mask_l;

	td = (task_des*)arg;

	set_period(td);
	/* loop until application end */
	while (!isEvent(evts, END)) {
		/* copy the actual radar and prediction arrays
		 * to a temporary buffer */
		pthread_mutex_lock(&r_mutex);
		r = r_coords;
		s = f_stat;
		pthread_mutex_unlock(&r_mutex);

		mask = 1;	/* 0000 0001 */
		for (i = 0; i < MAX_TARGETS; i++) {
			mask_l = mask << 4;
			/* if a target is inside the radar area for at least MIN_RAD_CYCLE
			 *  periods and the patriot is not fired yet, fire the patriot */
			if (!isEvent(p_mask, mask) && (engaged_cycle[i] > MIN_RAD_CYCLE)) {
				/* update the filtered target statistics */
				filterStats(r[i].x * SCALE, BOX_HEIGHT * SCALE - r[i].y *
						SCALE, (float32_t)(td->period) / 1000, &s[i]);
				/* create the patriot task */
				create_task(patriot_task, PATRIOT_PER, PATRIOT_DL, PATRIOT_PRIO,
					PATRIOT_INDEX + i);
				setEvent(p_mask, mask); /* patriot 'i' fired */
			} else if (isEvent(p_mask, mask)){ /* patriot already fired */
				/* update the filtered target statistics */
				filterStats(r[i].x * SCALE, BOX_HEIGHT * SCALE - r[i].y *
						SCALE, (float32_t)(td->period) / 1000, &s[i]);
				pthread_mutex_lock(&p_mutex);
				tmp = p_stat[i];
				pthread_mutex_unlock(&p_mutex);
				/* if the patriot is moving at constant speed and
				 * and the prediction isn't already been computed */
				if (tmp.v >= PATRIOT_V_MAX && !isEvent(evts, mask_l)) {
					if (computeIntercept(s[i], tmp, i)) {
						setEvent(evts, mask_l); /* prediction ready */
					}
				}
			}
			pthread_mutex_lock(&r_mutex);
			f_stat[i] = s[i];
			pthread_mutex_unlock(&r_mutex);
			mask <<= 1;
		}
		check_deadline(td);

		wfp(td);
	}
	pthread_exit(NULL);
}

/* PATRIOT TASK
 *
 * Performs the equations of motion for the patriot missile in order
 * to hit the target.
 */

void * patriot_task(void *arg)
{
task_des *td;
float32_t p_vx, p_vy, p_ax, p_ay, theta, dx, dy, dt;
stat patr_tmp;
filter_stat pred_tmp;
uint8_t mask, mask_l;
int32_t i, index;

	td = (task_des*)arg;
	set_period(td);
	dt = TSCALE*(float32_t)td->period/1000;
	index = td->index - PATRIOT_INDEX;	/* get the index */
	mask = 1;	/* 0000 0001 */
	for (i = 0; i < index; i++) mask <<= 1; /* get the right bitmask */
	mask_l = mask << 4;
	/* loop until application end and target fired and target not missed */
	while (!isEvent(evts, END) && !isEvent(t_mask, mask_l) &&
			isEvent(t_mask, mask) > 0) {
		/* Get the filtered target data */
		pthread_mutex_lock(&r_mutex);
		pred_tmp = f_stat[index];
		pthread_mutex_unlock(&r_mutex);
		/* Get the patriot data */
		pthread_mutex_lock(&p_mutex);
		patr_tmp = p_stat[index];
		pthread_mutex_unlock(&p_mutex);

		getCartStats(patr_tmp, &p_vx, &p_vy, &p_ax, &p_ay);

		/* Compute the new patriot direction */
		dx = pred_tmp.xf - patr_tmp.x;
		dy = pred_tmp.yf - patr_tmp.y;
		if (!isEvent(evts, mask_l)) {	/* prediction not ready yet */
			theta = atan2f(dy, dx);	/* move in target direction */
		} else {	/* prediction ready */
			theta = coll_theta[index];	/* move to the intercept */
		}
		pthread_mutex_lock(&p_mutex);
		/* Update the patriot acceleration */
		p_ax = p_stat[index].a * cos(theta);
		p_ay = p_stat[index].a * sin(theta);
		p_stat[index].a_theta = theta;
		/* Update the patriot speed */
		if (p_stat[index].v <= PATRIOT_V_MAX) {	/* max speed not reached */
			p_vx += p_ax * dt;	/* increase the speed according to acc. */
			p_vy += (p_ay - G0) * dt;
			p_stat[index].v_theta = atan2f(p_vy, p_vx);
		} else {	/* max speed reached */
			/* the speed vector magnitude is maintained constant, but the angle
			 * is updated as if the patriot is moving at constant acc. */
			p_stat[index].v_theta = atan2f(p_vy + p_ay * dt, p_vx + p_ax * dt);
			p_vx = p_stat[index].v * cos(p_stat[index].v_theta);
			p_vy = p_stat[index].v * sin(p_stat[index].v_theta);
			p_ax = 0;
			p_ay = G0;
		}
		p_stat[index].v = sqrt((p_vx*p_vx) + (p_vy*p_vy));
		/* Update the patriot position */
		p_stat[index].x += p_vx * dt + p_ax * dt * dt / 2;
		p_stat[index].y += p_vy * dt + (p_ay - G0) * dt * dt / 2;

		pthread_mutex_unlock(&p_mutex);

		check_deadline(td);

		wfp(td);
	}
	cleanTargetStats(index);	/* reset the target physical status */
	cleanPatriotStats(index);	/* reset the patriot physical status */
	pthread_exit(NULL);
}

/*	+-------------------------------------------------------------------------+
 *	|																		  |
 *	|						 TASK UTILITY FUNCTIONS   					      |
 *	|																		  |
 *	+-------------------------------------------------------------------------+
 */

/*----------------------------------------------------------------------------+
 *	findFreeTarget()														  |
 *																			  |
 *	Searches a free target data structure and if found returns its index 	  |
 *----------------------------------------------------------------------------+
 */

static int32_t findFreeTarget()
{
int32_t index;
uint8_t mask;

	mask = 1;
	for (index = 0; index < MAX_TARGETS; index++) {
		if (!isEvent(t_mask, mask)) { /* target non started yet */
			cleanTargetStats(index);
			setEvent(t_mask, mask);
			return index;
		}
		mask <<= 1;
	}
	index = -1;
	return index;
}


/*----------------------------------------------------------------------------+
 *	filterStats(x_new, y_new, period, pos_f, v_f, a_f)						  |
 *																			  |
 *	Computes and filters the target position, speed and acceleration		  |
 *----------------------------------------------------------------------------+
 */

static void filterStats(int32_t x, int32_t y, float32_t per, filter_stat *fs)
{
float32_t xf, yf, vx, vy, ax, ay, vxf, vyf, axf, ayf;
	/* if it's the first filtering, we take the actual
	 * coordinates as the filtered ones. In this way,
	 * the initial filtered speed is equal to 0.
	 */
	if (fs->xf == 0 && fs->yf == 0) {
		fs->xf = x;
		fs->yf = y;
	}
	/* position filtering */
	xf = pos_f * fs->xf + (1 - pos_f) * x;
	yf = pos_f * fs->yf + (1 - pos_f) * y;
	/* speed computation */
	vx = (xf - fs->xf) / per;
	vy = (yf - fs->yf) / per;
	/* speed filtering */
	vxf = v_f * fs->vxf + (1 - v_f) * vx;
	vyf = v_f * fs->vyf + (1 - v_f) * vy;
	/* acceleration computation */
	ax = (vxf - fs->vxf) / per;
	ay = (vyf - fs->vyf) / per;
	/* acceleration filtering */
	axf = a_f * fs->axf + (1 - a_f) * ax;
	ayf = a_f * fs->ayf + (1 - a_f) * ay;
	/* updating filter data structure */
	fs->xf = xf;
	fs->yf = yf;
	fs->vxf = vxf;
	fs->vyf = vyf;
	fs->axf = axf;
	fs->ayf = ayf;
}

/*----------------------------------------------------------------------------+
 *	computeIntercept(t, p, index)							 				  |
 *																			  |
 *	Computes the intercept angle between a target and a Patriot missiles.	  |
 *----------------------------------------------------------------------------+
 */

static uint8_t computeIntercept(filter_stat t, stat p, int32_t index)
{
float32_t xt, yt, xp, yp, dist, dt, theta_i, i;

	theta_i = p.v_theta;	/* initial angle */

	for (i = theta_i - degToRad(THETA_DEV); i <= theta_i + degToRad(THETA_DEV);
			i += degToRad(THETA_STEP)) {
		for (dt = 0; dt <= T_MAX; dt += T_STEP) {
			/* target position */
			xt = t.xf + t.vxf * dt + t.axf / 2 * dt * dt;
			yt = t.yf + t.vyf * dt + t.ayf / 2 * dt * dt;
			/* patriot position */
			xp = p.x + p.v * cos(i) * dt;
			yp = p.y + p.v * sin(i) * dt;
			/* distance between target and patriot */
			dist = sqrt((xt - xp) * (xt - xp) + (yt - yp) * (yt - yp));
			if (dist <= COLL_DIST / 2) {
				/* if the distance is small enough, we found a
				 * possible intercept angle */
				coll_theta[index] = i;
				return 1;
			}
		}
	}
	return 0;
}


/*----------------------------------------------------------------------------+
 *	cleanTargetStats()														  |
 *																			  |
 *	Reset the target status and the events to the initial state.			  |
 *----------------------------------------------------------------------------+
 */

static void cleanTargetStats(int32_t index)
{
	t_stat[index].v = frand((float32_t)MIN_T_V, (float32_t)MAX_T_V);
	t_stat[index].v_theta = 0;
	t_stat[index].x = frand((float32_t)MIN_T_X0, (float32_t)MAX_T_X0);
	t_stat[index].y = TARGET_Y0;
	t_stat[index].a = G0;	/* the target is affected only by gravity */
	t_stat[index].a_theta = TARGET_ACC_DEG;	/* 270 degrees */

	f_stat[index].xf = f_stat[index].yf = 0;
	f_stat[index].vxf = f_stat[index].vyf = 0;
	f_stat[index].axf = f_stat[index].ayf = 0;

	r_coords[index].x = r_coords[index].y = 0;

	clearTargetEvents(index);

	engaged_cycle[index] = 0;
}

/*----------------------------------------------------------------------------+
 *	cleanPatriotStats()														  |
 *																			  |
 *	Reset the patriot status 												  |
 *----------------------------------------------------------------------------+
 */

static void cleanPatriotStats(int32_t index)
{
	p_stat[index].v = 100;
	p_stat[index].v_theta = PI / 2;
	if (index == 0) p_stat[index].x = PATRIOT_0_X0;
	if (index == 1) p_stat[index].x = PATRIOT_1_X0;
	if (index == 2) p_stat[index].x = PATRIOT_2_X0;
	if (index == 3) p_stat[index].x = PATRIOT_3_X0;
	p_stat[index].y = PATRIOT_Y0;
	p_stat[index].a = PATRIOT_ACC;
	p_stat[index].a_theta = PATRIOT_A_THETA;
}

/*----------------------------------------------------------------------------+
 *	cleanSimStats()															  |
 *																			  |
 *	Reset the simulation statistics											  |
 *----------------------------------------------------------------------------+
 */

static void cleanSimStats()
{
	ss.t_fired = ss.t_missed = ss.t_hit = ss.t_hitratio = 0;
	/* reset filters to default values */
	pos_f = P1;
	v_f = P2;
	a_f = P3;
	/* clear the centroids events */
	clearEvent(evts, T_CENTROID);
	clearEvent(evts, P_CENTROID);
}

/*----------------------------------------------------------------------------+
 *	getCartStats(s, vx, vy, ax, ay)											  |
 *																			  |
 *	Converts the polar representation of speed and acceleration to the		  |
 *	Cartesian one											  				  |
 *----------------------------------------------------------------------------+
 */

static void getCartStats(stat s, float32_t *vx, float32_t *vy,
		float32_t *ax, float32_t *ay)
{
	if (vx != NULL) *vx = s.v * cos(s.v_theta);
	if (vy != NULL)	*vy = s.v * sin(s.v_theta);
	if (ax != NULL) *ax = s.a * cos(s.a_theta);
	if (ay != NULL)	*ay = s.a * sin(s.a_theta);
}
