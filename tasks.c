/*
 * TASKS SOURCE CODE
 *
 * This file contains tasks code and some auxiliary functions used
 * by them.
 *
 *  Created on: 14/nov/2015
 *      Author: paolo
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
pthread_mutex_t	r_mutex;	/* mutex for accessing radar scan data */
pthread_mutex_t	p_mutex;	/* mutex for accessing patriot status */

/* EVENT DECLARATIONS */
/* number of times the target is found inside the radar area */
uint8_t	engaged_cycle[MAX_TARGETS];
float32_t pos_f, v_f, a_f;
float32_t coll_theta[MAX_TARGETS];
coords r_coords[MAX_TARGETS];
stat p_stat[MAX_TARGETS];
stat t_stat[MAX_TARGETS];
pred_stat ps[MAX_TARGETS];
sim_stats ss;

/* LOCAL FUNCTIONS DECLARATION */
static int32_t findFreeTarget();
static void filterStats(int32_t x, int32_t y, float32_t period, pred_stat *ps);
static void computeIntercept(pred_stat t, stat p, int32_t index);
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
 *	the physical status of the system and the events variables.				  |						|
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
stat pred_s;
stat *t, *p;
pred_stat *pred;
coords *r;
uint8_t dmiss[TASK_NUM];
uint8_t i, tm_tmp, pm_tmp;

	td = (task_des*)arg;

	set_period(td);

	while (!isEvent(evts, END)) {

		tm_tmp = 1;
		pm_tmp = 1;
		/* Copy all the shared data structures */
		pthread_mutex_lock(&t_mutex);
		t = t_stat;
		pthread_mutex_unlock(&t_mutex);

		pthread_mutex_lock(&p_mutex);
		p = p_stat;
		pthread_mutex_unlock(&p_mutex);

		pthread_mutex_lock(&r_mutex);
		r = r_coords;
		pred = ps;
		pthread_mutex_unlock(&r_mutex);

		pred_s.x = pred[0].xf;
		pred_s.y = pred[0].yf;
		cartToPolar(pred[0].vxf, pred[0].vyf, &pred_s.v, &pred_s.v_theta);
		cartToPolar(pred[0].axf, pred[0].ayf, &pred_s.a, &pred_s.a_theta);

		pthread_mutex_lock(&sim_mutex);
		sim = ss;
		pthread_mutex_unlock(&sim_mutex);

		for (i = 0; i < TASK_NUM; i++) {
			dmiss[i] = task_set[i].dmiss;
		}
		drawTaskStats(dmiss);
		drawSimStats(sim, pos_f, v_f, a_f);
		/* I have to convert the y coordinate of the physical
		 * system to the allegro one */
		if (isStarted) {
			tm_tmp = 1;
			for (i = 0; i < MAX_TARGETS; i++) {
				if (isEvent(t_mask, tm_tmp)) drawTarget((int32_t)(t[i].x / SCALE),
						BOX_HEIGHT - (int32_t)(t[i].y / SCALE), t[i].v_theta, i);
				tm_tmp <<= 1;
			}
			drawStats(t[0], TARGET_STATS, TARGET_STATS_Y0);
			if (isEvent(evts, T_CENTROID)) {
				drawCentroid(r[0].x, r[0].y, RADAR_CENTROID);
			}
		}
		if (isFired){
			pm_tmp = 1;
			for (i = 0; i < MAX_TARGETS; i++) {
				if (isEvent(p_mask, pm_tmp))
					drawPatriot((int32_t)(p[i].x / SCALE), BOX_HEIGHT -
						(int32_t)(p[i].y / SCALE), p[i].v_theta);
				pm_tmp <<= 1;
			}
			if (isEvent(evts, P_CENTROID)) {
				drawCentroid((int32_t)(pred[0].xf / SCALE), BOX_HEIGHT -
						(int32_t)(pred[0].yf / SCALE), PRED_CENTROID);
			}
			drawStats(p[0], PATRIOT_STATS, PATRIOT_STATS_Y0);
			drawStats(pred_s, PRED_STATS, PRED_STATS_Y0);
			tm_tmp = pm_tmp = 1;
			for (i = 0; i < MAX_TARGETS; i++) {
				if (isEvent(t_mask, tm_tmp) && isEvent(p_mask, pm_tmp) &&
						checkCollision(t[i], p[i])) {
					setEvent(p_mask, pm_tmp << 4);
					clearEvent(t_mask, tm_tmp);
					clearEvent(p_mask, pm_tmp);
					pthread_mutex_lock(&sim_mutex);
					ss.t_hit++;
					ss.t_hitratio = (float32_t)ss.t_hit / ss.t_fired;
					pthread_mutex_unlock(&sim_mutex);
					drawCollision();
				}
				tm_tmp <<= 1;
				pm_tmp <<= 1;
			}
		}
		updateScreen();	/* updates the simulation area of the screen */
		updateStats();

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

	pthread_mutex_lock(&sim_mutex);
	ss.t_fired++;
	pthread_mutex_unlock(&sim_mutex);

	td = (task_des*)arg;
	dt = TSCALE*(float32_t)td->period/1000;
	index = td->index - TARGET_INDEX;
	evt_mask = 1;
	for (i = 0; i < index; i++) evt_mask <<= 1;
	printf("index: %d\n", index);

	/* compute the initial speed vector angle in order
	 * to hit the middle of the city */
	pthread_mutex_lock(&t_mutex);
	/* horizontal spacing between the target and the middle of the city */
	dx = ((BOX_WIDTH / 2) * SCALE - t_stat[index].x);
	/* vertical spacing, always equal to y coordinate */
	dy = -t_stat[index].y;
	t_stat[index].v_theta = atan2f(dy, dx);
	da = frand((float32_t)-T_DA, (float32_t)T_DA);
	t_stat[index].v_theta += da * (PI / 180);
	pthread_mutex_unlock(&t_mutex);

	set_period(td);

	while (!isEvent(evts, END) && isEvent(t_mask, evt_mask)) {

		pthread_mutex_lock(&t_mutex);
		/* Compute the target acceleration */
		ax = t_stat[index].a * (float32_t)(cos(t_stat[index].a_theta));
		ay = t_stat[index].a * (float32_t)(sin(t_stat[index].a_theta)),
		/* Compute the target speed */
		vx = ax * dt + t_stat[index].v * (float32_t)(cos(t_stat[index].v_theta));
		vy = ay * dt + t_stat[index].v * (float32_t)(sin(t_stat[index].v_theta));
		/* Update the target position */
		t_stat[index].x += vx * dt + ax * dt * dt / 2;
		t_stat[index].y += vy * dt + ay * dt * dt / 2;

		/* Check if the target reached the ground */
		if (t_stat[index].y <= CITY_COLLISION_Y) {
			/* Restart the simulation */
			clearEvent(t_mask, evt_mask);
			tmp_mask = evt_mask << 4;
			setEvent(t_mask, tmp_mask);
			pthread_mutex_lock(&sim_mutex);
			ss.t_missed++;
			ss.t_hitratio = (float32_t)ss.t_hit / ss.t_fired;
			pthread_mutex_unlock(&sim_mutex);
		}
		/* Update the speed modulus */
		t_stat[index].v = sqrt((vx*vx) + (vy*vy));
		/* Update the target speed angle according to the new speed */
		t_stat[index].v_theta = atan2f(vy, vx);

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

	while (!isEvent(evts, END)) {
		mask = 1;
		/* Scan the area looking for the targets */
		for (i = 0; i < MAX_TARGETS; i++) {
			if (isEvent(t_mask, mask)) {
				ret = scanArea(&xc, &yc, i);
				if (ret) {
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

void * parse_keyboard(void *arg)
{
task_des *td;
int32_t free;

	td = (task_des*)arg;

	set_period(td);

	while (!end) {
		/* key 'A' spawns a target only if it's not on the screen */
		if (key[KEY_A]) {
			if (freeTargets){
				/* search a free target structure */
				free = findFreeTarget();
				/* create a new target task */
				create_task(target_task, 50, 50, 32, TARGET_INDEX + free);
			}
		}
		/* key 'C' shows the target centroid computed from
		 * radar coordinates */
		if (key[KEY_C] && isStarted) {
			toggleEvent(evts, T_CENTROID);
		}
		/* key 'P' shows the target centroid computed from
		 * predicted coordinates */
		if (key[KEY_P] && isStarted) {
			toggleEvent(evts, P_CENTROID);
		}
		/* key 'R' reset the simulation */
		if (key[KEY_R] && isStarted) {
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
pred_stat *s;
float32_t dx, dy, theta;
int32_t i;
uint8_t mask;

	td = (task_des*)arg;

	set_period(td);

	while (!isEvent(evts, END)) {
		pthread_mutex_lock(&r_mutex);
		r = r_coords;
		s = ps;
		pthread_mutex_unlock(&r_mutex);

		mask = 1;
		/* if a target is inside the radar area for at least MIN_RAD_CYCLE
		 *  periods, fire the patriot */
		for (i = 0; i < MAX_TARGETS; i++) {
			if (!isEvent(p_mask, mask) && engaged_cycle[i] > MIN_RAD_CYCLE) {
				/* Compute the predicted position according to the actual
				 *  centroid coordinates */
				filterStats(r[i].x*SCALE, BOX_HEIGHT * SCALE - r[i].y*SCALE,
						(float32_t)(td->period) / 1000, &s[i]);
				/* create the patriot task */
				create_task(patriot_task, PATRIOT_PER, PATRIOT_DL, PATRIOT_PRIO,
					PATRIOT_INDEX + i);
				setEvent(p_mask, mask);
			} else if (isEvent(p_mask, mask)){
				filterStats(r[i].x*SCALE, BOX_HEIGHT * SCALE - r[i].y*SCALE,
						(float32_t)(td->period) / 1000, &s[i]);
				pthread_mutex_lock(&p_mutex);
				tmp = p_stat[i];
				pthread_mutex_unlock(&p_mutex);
				if (tmp.v >= PATRIOT_V_MAX && !isEvent(evts, mask << 4)) {
					computeIntercept(s[i], tmp, i);
					setEvent(evts, mask << 4); /* prediction ready */
				}
			}
			pthread_mutex_lock(&r_mutex);
			ps[i] = s[i];
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
float32_t x, y, p_x, p_y, p_vx, p_vy, p_ax, p_ay, theta, dx, dy, dt;
stat patr_tmp;
pred_stat pred_tmp;
uint8_t mask;
int32_t i, index;

	td = (task_des*)arg;

	set_period(td);

	dt = TSCALE*(float32_t)td->period/1000;
	index = td->index - PATRIOT_INDEX;
	mask = 1;
	for (i = 0; i < index; i++) mask <<= 1;

	while (!isEvent(evts, END) && !isEvent(t_mask, mask << 4) &&
			isEvent(t_mask, mask) > 0) {

		/* Get the prediction data */
		pthread_mutex_lock(&r_mutex);
		pred_tmp = ps[index];
		pthread_mutex_unlock(&r_mutex);
		/* Get the patriot data */
		pthread_mutex_lock(&p_mutex);
		patr_tmp = p_stat[index];
		pthread_mutex_unlock(&p_mutex);

		x = pred_tmp.xf;
		y = pred_tmp.yf;
		p_x = patr_tmp.x;
		p_y = patr_tmp.y;

		getCartStats(patr_tmp, &p_vx, &p_vy, &p_ax, &p_ay);

		/* Compute the new patriot direction */
		dx = x - p_x;
		dy = y - p_y;

		if (!isEvent(evts, mask << 4)) {
			theta = atan2f(dy, dx);
		} else {
			theta = coll_theta[index];
		}

		pthread_mutex_lock(&p_mutex);
		/* Update the patriot acceleration */
		p_ax = p_stat[index].a * cos(theta);
		p_ay = p_stat[index].a * sin(theta);
		p_stat[index].a_theta = theta;
		/* Update the patriot speed */
		if (p_stat[index].v <= PATRIOT_V_MAX) {
			p_vx += p_ax * dt;
			p_vy += (p_ay - G0) * dt;
			p_stat[index].v_theta = atan2f(p_vy, p_vx);
		} else {
			p_stat[index].v_theta = atan2f(p_vy + p_ay * dt, p_vx + p_ax * dt);
			p_vx = p_stat[index].v * cos(p_stat[index].v_theta);
			p_vy = p_stat[index].v * sin(p_stat[index].v_theta);
			p_ax = 0;
			p_ay = G0;
		}
		/* Update the patriot position */
		p_x += p_vx * dt + p_ax * dt * dt / 2;
		p_y += p_vy * dt + (p_ay - G0) * dt * dt / 2;

		p_stat[index].x = p_x;
		p_stat[index].y = p_y;
		p_stat[index].v = sqrt((p_vx*p_vx) + (p_vy*p_vy));

		pthread_mutex_unlock(&p_mutex);

		check_deadline(td);

		wfp(td);
	}
	cleanTargetStats(index);	/* reset the target physical status */
	cleanPatriotStats(index);/* reset the patriot physical status */
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

static void filterStats(int32_t x, int32_t y, float32_t period, pred_stat *ps)
{
float32_t xf, yf, vx, vy, ax, ay, vxf, vyf, axf, ayf;

	if (ps->xf == 0 && ps->yf == 0) {
		ps->xf = x;
		ps->yf = y;
	}

	xf = pos_f * ps->xf + (1 - pos_f) * x;
	yf = pos_f * ps->yf + (1 - pos_f) * y;

	vx = (xf - ps->xf) / period;
	vy = (yf - ps->yf) / period;

	vxf = v_f * ps->vxf + (1 - v_f) * vx;
	vyf = v_f * ps->vyf + (1 - v_f) * vy;

	ax = (vxf - ps->vxf) / period;
	ay = (vyf - ps->vyf) / period;

	axf = a_f * ps->axf + (1 - a_f) * ax;
	ayf = a_f * ps->ayf + (1 - a_f) * ay;

	ps->xf = xf;
	ps->yf = yf;
	ps->vxf = vxf;
	ps->vyf = vyf;
	ps->axf = axf;
	ps->ayf = ayf;
}

/*----------------------------------------------------------------------------+
 *	computeIntercept(s, theta)							 					  |
 *																			  |
 *																			  |
 *----------------------------------------------------------------------------+
 */

static void computeIntercept(pred_stat t, stat p, int32_t index)
{
float32_t xt, yt, xp, yp, dist, dt, theta, theta_i, i;

	theta_i = p.v_theta;

	for (i = theta_i - degToRad(35); i <= theta_i + degToRad(35); i += degToRad(1)) {
		for (dt = 0; dt <= 25; dt += 0.1) {
			xt = t.xf + t.vxf * dt + t.axf / 2 * dt * dt;
			yt = t.yf + t.vyf * dt + t.ayf / 2 * dt * dt;
			xp = p.x + p.v * cos(i) * dt;
			yp = p.y + p.v * sin(i) * dt;
			dist = sqrt((xt - xp) * (xt - xp) + (yt - yp) * (yt - yp));
			if (dist <= COLL_DIST / 2) {
				coll_theta[index] = i;
				printf("t_intercept = %.2f\n", dt);
				return;
			}
		}
	}
	//printf("x_intercept = %.2f\n", x_coll);
	//printf("tan_a = %.2f\n", radToDeg(atan(tan_a)));
	//printf("theta_intercept = %.2f\n", radToDeg(coll_theta));
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

	ps[index].xf = ps[index].yf = ps[index].vxf = ps[index].vyf = 0;
	ps[index].axf = ps[index].ayf = 0;

	r_coords[index].x = r_coords[index].y = 0;

	clearTargetEvents(index);

	engaged_cycle[index] = 0;
}

/*----------------------------------------------------------------------------+
 *	cleanPatriotStats()														  |
 *																			  |
 *	Reset the patriot status												  |
 *----------------------------------------------------------------------------+
 */

static void cleanPatriotStats(int32_t index)
{

	p_stat[index].v = 100;
	p_stat[index].v_theta = 0;
	p_stat[index].x = PATRIOT_X0;
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

	pos_f = P1;
	v_f = P2;
	a_f = P3;

	clearEvent(evts, T_CENTROID);
	clearEvent(evts, P_CENTROID);
}

/*----------------------------------------------------------------------------+
 *	getCartStats(s, vx, vy, ax, ay)											  |
 *																			  |
 *	Reset the simulation statistics											  |
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
