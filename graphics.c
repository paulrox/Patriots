/*
 * GRAPHICS MODULE SOURCE CODE
 *
 * This file contains all the definitions of the functions used in
 * the application for drawing on the screen and for graphics related
 * computations.
 *
 *  Created on: 03/nov/2015
 *      Author: Paolo Sassi
 */

#include "graphics.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <allegro.h>

#include "events.h"
#include "globals.h"

int32_t line_color, txt_color, head_color, p_color, tc_color, tp_color;
int32_t t_colors[MAX_TARGETS];

BITMAP *city, *radar, *bkg, *box_buffer, *ss_buffer, *ts_buffer,
	*radar_buffer, *patriot;
BITMAP	*targets[MAX_TARGETS];

static void drawInstructions();

/*----------------------------------------------------------------------------+
 *	initGraphics()															  |
 *																			  |
 *	Initializes the allegro library and the globals used for graphics		  |						|
 *----------------------------------------------------------------------------+
 */

void initGraphics()
{
int32_t i;
char t_path[13];

	/* initialize allegro related stuff */
	allegro_init();
	install_keyboard();
	set_color_depth(16);
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, SCREEN_WIDTH, SCREEN_HEIGHT, 0, 0);
	clear_to_color(screen, 0);

	/* initialize used colors*/
	txt_color = makecol(255, 255, 0);	/* yellow color for text */
	line_color = makecol(255, 0, 0);	/* red color for lines */
	head_color = makecol(160, 160, 160);/* gray color for heading text */
	/* target colors */
	t_colors[0] = makecol(255, 0, 0);
	t_colors[1] = makecol(0, 0, 255);
	t_colors[2] = makecol(255, 214, 0);
	t_colors[3] = makecol(197, 197, 197);
	p_color = makecol(0, 255, 0);		/* green color for patriot */
	tc_color = makecol(255, 255, 0);	/* yellow color for target centroid */
	tp_color = makecol(0, 255, 255);	/* blue color for predicted centroid */

	/* initialize screen buffers */
	ss_buffer = create_bitmap(SS_WIDTH, SS_HEIGHT);
	ts_buffer = create_bitmap(TS_WIDTH, TS_HEIGHT);
	bkg = create_bitmap(BOX_WIDTH, BOX_HEIGHT);
	box_buffer = create_bitmap(BOX_WIDTH, BOX_HEIGHT);
	radar_buffer = create_bitmap(RAD_RANGE_X, RAD_RANGE_Y);

	/* load images for the target and the patriot */
	for (i = 0; i < 4; i++) {
		sprintf(t_path, "img/target_%d.bmp", i);
		targets[i] = load_bitmap(t_path, NULL);
	}

	patriot = load_bitmap(PATRIOT_PATH, NULL);

	/* clear screen buffer to black */
	clear_to_color(bkg, 0);
	clear_to_color(box_buffer, 0);
}

/*----------------------------------------------------------------------------+
 *	endGraphics()															  |
 *																			  |
 *	Destroys all the used bitmaps and quits the allegro library		  		  |
 *----------------------------------------------------------------------------+
 */

void endGraphics()
{
int32_t i;

	destroy_bitmap(bkg);
	destroy_bitmap(ss_buffer);
	destroy_bitmap(ts_buffer);
	destroy_bitmap(box_buffer);
	destroy_bitmap(radar_buffer);
	destroy_bitmap(patriot);
	/* destroy all the targets bitmaps */
	for (i = 0; i < 4; i++) destroy_bitmap(targets[i]);

	allegro_exit();
}

/*----------------------------------------------------------------------------+
 *	drawGUI()															  	  |
 *																			  |
 *	Draws the static part of the GUI and draws the background image		  	  |
 *----------------------------------------------------------------------------+
 */

void drawGUI() {

	/* TOP GUI */
	textout_centre_ex(screen, font, "Patriot System Simulation",
			400, 5, txt_color, 0);
	line(screen, 0, 19, 800, 19, line_color);
	line(screen, 290, 0, 290, 19, line_color);
	line(screen, 508, 0, 508, 19, line_color);
	line(screen, 0, 79, 800, 79, line_color);

	/* RIGHT SIDE GUI */
	line(screen, BOX_WIDTH + 1, 0, BOX_WIDTH + 1, SCREEN_HEIGHT, line_color);
	drawInstructions();

	/* load background images */
	city = load_bitmap(CITY_PATH, NULL);
	if (city == NULL) {
			printf("%s not found!\n", CITY_PATH);
			exit(1);
	}
	radar = load_bitmap(RADAR_PATH, NULL);
	if (radar == NULL) {
		printf("%s not found!\n", RADAR_PATH);
		exit(1);
	}
	blit(city, bkg, 0, 0, CITY_POS_X, CITY_POS_Y, city->w, city->h);
	draw_sprite(bkg, radar, RADAR_POS_X, RADAR_POS_Y);
	blit(bkg, screen, 0, 0, 0, 80, BOX_WIDTH, BOX_HEIGHT);

	/* destroy the temporary bitmaps */
	destroy_bitmap(city);
	destroy_bitmap(radar);
}

/*----------------------------------------------------------------------------+
 *	updateScreen()														  	  |
 *																			  |
 *	Updates the simulation box on the screen	  	 						  |
 *----------------------------------------------------------------------------+
 */

void updateScreen()
{
	blit(box_buffer, screen, 0, 0, BOX_POS_X, BOX_POS_Y, BOX_WIDTH,
			BOX_HEIGHT);
	blit(bkg, box_buffer, 0, 0, 0, 0, BOX_WIDTH, BOX_HEIGHT);
}

/*----------------------------------------------------------------------------+
 *	updateStats()														  	  |
 *																			  |
 *	Updates the simulation statistics on the screen	  	 					  |
 *----------------------------------------------------------------------------+
 */

void updateStats()
{
	blit(ss_buffer, screen, 0, 0, BOX_WIDTH + 2, 0, SS_WIDTH, SS_HEIGHT);
	clear_to_color(ss_buffer, 0);
}

/*----------------------------------------------------------------------------+
 *	scanArea(xc, yc)							 			        		  |
 *																			  |
 *	Copy a portion of the screen to a buffer and if the target is detected 	  |
 *	computes its centroid.				    					  			  |
 *----------------------------------------------------------------------------+
 */

uint8_t scanArea(int32_t *xc, int32_t *yc, int32_t index)
{
uint8_t found;
int32_t x, y, pixel_num, target_x, target_y;

	target_x = target_y = found = pixel_num = 0;

	/* copy a rectangular region near the radar to the radar buffer */
	blit(screen, radar_buffer, RAD_AREA_X, RAD_AREA_Y, 0, 0,
			RAD_RANGE_X, RAD_RANGE_Y);
	/* scan the radar buffer searching the target */
	for (x = 0; x <= RAD_RANGE_X; x++) {
		for (y = 0; y <= RAD_RANGE_Y - RAD_RANGE_MIN; y++) {
			if (getpixel(radar_buffer, x, y) == t_colors[index]) {
				found = 1;
				target_x += x;
				target_y += y;
				pixel_num++;
			}
		}
	}
	/* if at least one target pixel is found, computes the centroid
	 * for the detected pixels */
	if (found) {
		*xc = (target_x / pixel_num);
		*yc = (target_y / pixel_num);
		return 1;
	}
	return 0;
}

/*----------------------------------------------------------------------------+
 *	checkCollision(t, p)								 			          |
 *																			  |
 *	Checks whether a collision between target and patriot has occurred	  	  |
 *----------------------------------------------------------------------------+
 */

uint8_t checkCollision(stat t, stat p)
{
float32_t dist;

	dist = sqrt((t.x - p.x) * (t.x - p.x) + (t.y - p.y) * (t.y - p.y));

	if (dist <= COLL_DIST) {
		return 1;
	}
	return 0;
}

/*----------------------------------------------------------------------------+
 *	drawTaskStats(dmiss)									 				  |
 *																			  |
 *	Draw statistics about the tasks into a buffer 							  |
 *	and then copy them on the screen	  	 								  |
 *----------------------------------------------------------------------------+
 */

void drawTaskStats(uint8_t *dmiss)
{
char_t s[20];
int32_t i, sum;

	sum = 0;
	clear_to_color(ts_buffer, 0);
	textout_centre_ex(ts_buffer, font, "Display", 100, 10, head_color, 0);
	sprintf(s, "DMiss: %d", dmiss[0]);
	textout_centre_ex(ts_buffer, font, s, 100, 32, head_color, 0);
	textout_centre_ex(ts_buffer, font, "Radar", 200, 10, head_color, 0);
	sprintf(s, "DMiss: %d", dmiss[1]);
	textout_centre_ex(ts_buffer, font, s, 200, 32, head_color, 0);
	textout_centre_ex(ts_buffer, font, "Keyboard", 300, 10, head_color, 0);
	sprintf(s, "DMiss: %d", dmiss[2]);
	textout_centre_ex(ts_buffer, font, s, 300, 32, head_color, 0);
	textout_centre_ex(ts_buffer, font, "ECS", 400, 10, head_color, 0);
	sprintf(s, "DMiss: %d", dmiss[3]);
	textout_centre_ex(ts_buffer, font, s, 400, 32, head_color, 0);
	for (i = TARGET_INDEX; i < TARGET_INDEX + MAX_TARGETS; i++) {
		sum += dmiss[i];
	}
	textout_centre_ex(ts_buffer, font, "Target", 500, 10, head_color, 0);
	sprintf(s, "DMiss: %d", sum);
	textout_centre_ex(ts_buffer, font, s, 500, 32, head_color, 0);
	sum = 0;
	for (i = PATRIOT_INDEX; i < PATRIOT_INDEX + MAX_TARGETS; i++) {
		sum += dmiss[i];
	}
	textout_centre_ex(ts_buffer, font, "Patriot", 600, 10, head_color, 0);
	sprintf(s, "DMiss: %d", sum);
	textout_centre_ex(ts_buffer, font, s, 600, 32, head_color, 0);

	blit(ts_buffer, screen, 0, 0, 0, 20, TS_WIDTH, TS_HEIGHT-2);
}

/*----------------------------------------------------------------------------+
 *	drawSimStats(ss, p_f, v_f, a_f)									 		  |
 *																			  |
 *	Draw statistics about the simulation into a buffer	  	 		  		  |
 *----------------------------------------------------------------------------+
 */

void drawSimStats(sim_stats ss, float32_t p_f, float32_t v_f, float32_t a_f)
{
char_t s[20];

	textout_centre_ex(ss_buffer, font, "Simulation", SS_WIDTH / 2, 20,
			head_color, 0);
	textout_centre_ex(ss_buffer, font, "Statistics", SS_WIDTH / 2, 40,
			head_color, 0);
	sprintf(s, "Target Fired: %d", ss.t_fired);
	textout_centre_ex(ss_buffer, font, s, SS_WIDTH / 2, 60, txt_color, 0);
	sprintf(s, "Target Hit: %d", ss.t_hit);
	textout_centre_ex(ss_buffer, font, s, SS_WIDTH / 2, 80, txt_color, 0);
	sprintf(s, "Target Missed: %d", ss.t_missed);
	textout_centre_ex(ss_buffer, font, s, SS_WIDTH / 2, 100, txt_color, 0);
	if (ss.t_fired == 0) {
		sprintf(s, "Hit Ratio: %.2f%%", 0.00);
	} else {
		sprintf(s, "Hit Ratio: %.2f%%", ss.t_hitratio * 100);
	}
	textout_centre_ex(ss_buffer, font, s, SS_WIDTH / 2, 120, txt_color, 0);
	sprintf(s, "Position Filter: %.2f", p_f);
	textout_centre_ex(ss_buffer, font, s, SS_WIDTH / 2, 140, txt_color, 0);
	sprintf(s, "Speed Filter: %.2f", v_f);
	textout_centre_ex(ss_buffer, font, s, SS_WIDTH / 2, 160, txt_color, 0);
	sprintf(s, "Acc. Filter: %.2f", a_f);
	textout_centre_ex(ss_buffer, font, s, SS_WIDTH / 2, 180, txt_color, 0);

}

/*----------------------------------------------------------------------------+
 *	drawPatriotStats(pm, evts)								 			      |
 *																			  |
 *	Draws status information about the patriots					 	 		  |
 *----------------------------------------------------------------------------+
 */

void drawPatriotStats(uint8_t pm, uint8_t evts_tmp)
{
char_t s[20], t[20];
uint8_t mask1, mask2;
int32_t i, y;

	mask1 = 1;		/* 0000 0001 */
	mask2 = 16;		/* 0001 0000 */
	textout_centre_ex(ss_buffer, font, "Patriot Stats", SS_WIDTH / 2,
			200, head_color, 0);
	y = 220;
	for (i = 0; i < MAX_TARGETS; i++) {
		if (isEvent(evts_tmp, mask2)) {	/* prediction ready */
			sprintf(t, "Target lock");
		} else if(isEvent(pm, mask1)) {	/* patriot fired */
			sprintf(t, "Fired");
		} else {						/* patriot not fired yet */
			sprintf(t, "Ready");
		}
		sprintf(s, "P%d: %s", i, t);
		textout_centre_ex(ss_buffer, font, s, SS_WIDTH / 2, y, txt_color, 0);
		y += 20;
		mask1 <<= 1;
		mask2 <<= 1;
	}
}

void drawInstructions()
{
	textout_centre_ex(screen, font, "Instructions", BOX_WIDTH + SS_WIDTH / 2,
			320, head_color, 0);
	textout_centre_ex(screen, font, "Spacebar:", BOX_WIDTH + SS_WIDTH / 2, 340, txt_color, 0);
	textout_ex(screen, font, "Spawn a new target", BOX_WIDTH + 20, 355, txt_color, 0);
	textout_centre_ex(screen, font, "R:", BOX_WIDTH + SS_WIDTH / 2, 375, txt_color, 0);
	textout_ex(screen, font, "Reset the simulation", BOX_WIDTH + 20, 390, txt_color, 0);
	textout_centre_ex(screen, font, "C:", BOX_WIDTH + SS_WIDTH / 2, 410, txt_color, 0);
	textout_ex(screen, font, "Show real centroid", BOX_WIDTH + 20, 425, txt_color, 0);
	textout_centre_ex(screen, font, "P:", BOX_WIDTH + SS_WIDTH / 2, 445, txt_color, 0);
	textout_ex(screen, font, "Show sampled centroid", BOX_WIDTH + 20, 460, txt_color, 0);
	textout_centre_ex(screen, font, "1, 2:", BOX_WIDTH + SS_WIDTH / 2, 480, txt_color, 0);
	textout_ex(screen, font, "Pos Filter - / +", BOX_WIDTH + 20, 495, txt_color, 0);
	textout_centre_ex(screen, font, "3, 4:", BOX_WIDTH + SS_WIDTH / 2, 515, txt_color, 0);
	textout_ex(screen, font, "Speed Filter - / +", BOX_WIDTH + 20, 530, txt_color, 0);
	textout_centre_ex(screen, font, "5, 6:", BOX_WIDTH + SS_WIDTH / 2, 550, txt_color, 0);
	textout_ex(screen, font, "Acc Filter - / +", BOX_WIDTH + 20, 565, txt_color, 0);
}

/*----------------------------------------------------------------------------+
 *	drawCollision()								 			        		  |
 *																			  |
 *	Draws a visual output about the objects collision	  	 			  	  |
 *----------------------------------------------------------------------------+
 */

void drawCollision()
{
	textout_centre_ex(ss_buffer, font, "Target destroyed!",
			SS_WIDTH / 2, 510, txt_color, 0);
}

/*----------------------------------------------------------------------------+
 *	drawTarget()								 			        		  |
 *																			  |
 *	Draws the rotated sprite of the enemy target into a buffer  	 		  |
 *----------------------------------------------------------------------------+
 */

void drawTarget(int32_t x, int32_t y, float32_t angle, int32_t index)
{
	rotate_sprite(box_buffer, targets[index], x - TARGET_WIDTH / 2,
			y - TARGET_HEIGHT / 2, radtofix(angle));
}

/*----------------------------------------------------------------------------+
 *	drawPatriot()								 			        		  |
 *																			  |
 *	Draws the rotated sprite of the patriot target into a buffer  	 		  |
 *----------------------------------------------------------------------------+
 */

void drawPatriot(int32_t x, int32_t y, float32_t angle)
{
	rotate_sprite(box_buffer, patriot, x - PATRIOT_WIDTH / 2, y - PATRIOT_HEIGHT / 2, radtofix(angle));
}

/*----------------------------------------------------------------------------+
 *	drawCentroid()								 			        		  |
 *																			  |
 *	Draws a small circle in the specified coordinates	 				      |
 *----------------------------------------------------------------------------+
 */

void drawCentroid(int32_t x, int32_t y, uint8_t type)
{
	if (type == RADAR_CENTROID) circlefill(box_buffer, x, y , 3, tc_color);
	if (type == PRED_CENTROID) circlefill(box_buffer, x, y, 3, tp_color);
}
