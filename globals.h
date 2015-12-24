/*
 *	GLOBALS DEFINITION
 *
 *	This file contains the list of all the macros used in the application and
 *	also contains the definitions of all the data structures
 *
 *  Created on: 25/nov/2015
 *      Author: Paolo Sassi
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#include <stdlib.h>
#include <stdio.h>


/*	+-------------------------------------------------------------------------+
 *	|																		  |
 *	|							GLOBAL MACROS								  |
 *	|																		  |
 *	+-------------------------------------------------------------------------+
 */

/* TASK MACROS */

#define TASK_NUM			12
#define MAX_TARGETS			4
#define	DISPLAY_INDEX		0
#define DISPLAY_PER			35
#define DISPLAY_DL			DISPLAY_PER
#define	DISPLAY_PRIO		29
#define	RADAR_INDEX			1
#define	RADAR_PER			70
#define	RADAR_DL			RADAR_PER
#define	RADAR_PRIO			32
#define	KEYBOARD_INDEX		2
#define	KEYBOARD_PER		100
#define	KEYBOARD_DL			KEYBOARD_PER
#define	KEYBOARD_PRIO		33
#define ECS_INDEX			3
#define ECS_PER				35
#define ECS_DL				ECS_PER
#define ECS_PRIO			28
#define	TARGET_INDEX		4
#define	TARGET_PER			50
#define	TARGET_DL			TARGET_PER
#define	TARGET_PRIO			30
#define	PATRIOT_INDEX		8
#define	PATRIOT_PER			50
#define	PATRIOT_DL			PATRIOT_PER
#define	PATRIOT_PRIO		31

/* GRAPHICS MACROS */

#define SCALE 				150
#define SCREEN_WIDTH		1000
#define SCREEN_HEIGHT		600
#define TS_WIDTH			800
#define TS_HEIGHT			60
#define TS_POS_X			0
#define TS_POS_Y			20
#define BOX_WIDTH 			800
#define BOX_HEIGHT 			540
#define BOX_POS_X			0
#define BOX_POS_Y			TS_POS_Y + TS_HEIGHT
#define SS_WIDTH			198
#define SS_HEIGHT			300
#define TARGET_WIDTH		40
#define TARGET_HEIGHT		12
#define PATRIOT_WIDTH		40
#define PATRIOT_HEIGHT		12
#define CITY_WIDTH			800
#define CITY_HEIGHT			167
#define CITY_POS_X 			0
#define CITY_POS_Y 			(520 - CITY_HEIGHT)
#define RADAR_WIDTH			40
#define RADAR_HEIGHT		36
#define RADAR_POS_X 		380
#define RADAR_POS_Y 		484
#define RADAR_CENTROID		0
#define PRED_CENTROID		1
#define CITY_PATH 			"img/skyline.bmp"
#define RADAR_PATH 			"img/radar.bmp"
#define TARGET_PATH			"img/target_"
#define PATRIOT_PATH		"img/patriot.bmp"
#define VECTOR_LEN			20
#define CITY_COLLISION_Y	70 * SCALE
#define radtofix(x)			ftofix(((-x) * 128) / PI)

/* PHYSICS MACROS */

#define TSCALE				1
#define G0					9.8
#define MIN_T_X0			0
#define	MAX_T_X0			120000
#define MIN_T_V				680
#define MAX_T_V				1500
#define T_DA				5	/* max target speed angle deviation */
/* target initial y (real) coordinate */
#define TARGET_Y0			BOX_HEIGHT * SCALE
/* target acceleration vector angle */
#define TARGET_ACC_DEG		(float32_t)-((0.5) * PI)
/* patriot initial x (real) coordinate */
#define PATRIOT_X0			400 * SCALE
/* patriot initial y (real) coordinate */
#define PATRIOT_Y0			100 * SCALE
#define PATRIOT_V_MAX		1700
#define PATRIOT_ACC			300
#define PATRIOT_A_THETA		PI / 2
#define PI					3.14159265
#define P1					0.9	/* constant used for filtering the position */
#define P2					0.95/* constant used for filtering the speed */
#define P3					0.98/* constant used for acc. filtering */
#define FILTER_STEP			0.01
#define FILTER_MAX			0.99
#define FILTER_MIN			0.00
#define COLL_DIST			20 * SCALE

#define radToDeg(x)			x * (180 / PI)
#define degToRad(x)			x * (PI / 180)

/* RADAR MACROS */

#define RAD_RANGE_X			BOX_WIDTH
#define RAD_RANGE_Y			BOX_HEIGHT
#define RAD_RANGE_MIN		100
#define	RAD_DEG				180
#define RAD_STEP			1
#define RAD_AREA_X			(BOX_WIDTH - RAD_RANGE_X) / 2
#define RAD_AREA_Y			(BOX_HEIGHT - RAD_RANGE_Y) + BOX_POS_Y
#define MIN_RAD_CYCLE		50

/* ERROR CODES */

#define CREATE_ERR			0
#define DMISS				1



/*	+-------------------------------------------------------------------------+
 *	|																		  |
 *	|						GLOBAL DATA STRUCTURES							  |
 *	|																	      |
 *	+-------------------------------------------------------------------------+
 */

/* MISRA-C Compliant data types */

typedef				char	char_t;
typedef signed		char	int8_t;
typedef signed		short	int16_t;
typedef signed 		int		int32_t;
typedef signed 		long	int64_t;
typedef unsigned 	char	uint8_t;
typedef unsigned 	short	uint16_t;
typedef unsigned 	int		uint32_t;
typedef unsigned 	long	uint64_t;
typedef 			float	float32_t;
typedef 			double 	float64_t;
typedef long		double 	float128_t;


/* Missile status data structure, used for both enemy target
 * and Patriot missile. Data are stored using MKS units.
 */
typedef struct _stat {
	float32_t x;
	float32_t y;
	float32_t v;		/* speed vector modulus */
	float32_t v_theta;	/* speed vector angle in radians */
	float32_t a;		/* acceleration vector modulus */
	float32_t a_theta;	/* acceleration vector angle in radians */
} stat;

/* Data structure used to store the information
 * to compute predictions
 */
typedef struct _pred_stat {
	float32_t xf;
	float32_t yf;
	float32_t vxf;
	float32_t vyf;
	float32_t axf;
	float32_t ayf;
} pred_stat;

/* Simulation statistics */
typedef struct _sim_stats {
	uint8_t 	t_fired;
	uint8_t 	t_missed;
	uint8_t		t_hit;
	float32_t	t_hitratio;
} sim_stats;

/* Screen coordinates data structure */
typedef struct _coords {
	int32_t x;
	int32_t y;
} coords;


#endif /* GLOBALS_H_ */
