/*
 * 	PATRIOT SYSTEM SIMULATION
 *
 * 	Simple demo application simulating a Patriot which
 * 	tries to catch an incoming enemy missile.
 *
 *  Created on: 14/ott/2015
 *      Author: Paolo Sassi
 */

#include "globals.h"		// application global constants
#include "taskslib.h"		// task management functions
#include "graphics.h"		// graphics functions
#include "tasks.h"			// task code and task-related functions



int main()
{

	init();

	drawGUI();

	/* create the initial task set */
	create_task(display_task, DISPLAY_PER, DISPLAY_DL, DISPLAY_PRIO, DISPLAY_INDEX);
	create_task(radar_task, RADAR_PER, RADAR_DL, RADAR_PRIO, RADAR_INDEX);
	create_task(parse_keyboard, KEYBOARD_PER, KEYBOARD_DL, KEYBOARD_PRIO, KEYBOARD_INDEX);
	create_task(ecs_task, ECS_PER, ECS_DL, ECS_PRIO, ECS_INDEX);

	/* wait until application end */
	while(!end);

	endProgram();

	return 0;

}
