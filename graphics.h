/*
 * GRAPHICS MODULE HEADER FILE
 *
 *  Created on: 03/nov/2015
 *      Author: paolo
 */

#ifndef GRAPHICS_H_
#define GRAPHICS_H_

#include "globals.h"

/* GLOBAL FUNCTIONS */

void initGraphics();
void endGraphics();
void drawGUI();
void updateScreen();
void updateStats();
uint8_t scanArea(int32_t *xc, int32_t *yc, int32_t index);
uint8_t checkCollision(stat t, stat p);
void drawTarget(int32_t x, int32_t y, float32_t angle, int32_t index);
void drawPatriot(int32_t x, int32_t y, float32_t angle);
void drawTaskStats(uint8_t *dmiss);
void drawSimStats(sim_stats ss, float32_t p_f, float32_t v_f, float32_t a_f);
void drawStats(stat s, uint8_t type, int32_t y0);
void drawCentroid(int32_t x, int32_t y, uint8_t type);
void drawCollision();

#endif /* GRAPHICS_H_ */
