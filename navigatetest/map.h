#pragma once
#include "ThisRobot.h"
// We need to know, for various purposes, how big our map is allowed to be
#define MAP_WIDTH  120
#define MAP_HEIGHT 120

#define ALLMAP_WIDTH  600
#define ALLMAP_HEIGHT 600

#define SWPARATOR_BIG ','
#define SWPARATOR_SMALL ';'

extern unsigned char rawmap[ALLMAP_WIDTH][ALLMAP_HEIGHT];
extern int allmap[ALLMAP_WIDTH][ALLMAP_HEIGHT];
extern unsigned char showmap[ALLMAP_WIDTH][ALLMAP_HEIGHT];
extern int nowmap[ALLMAP_WIDTH][ALLMAP_HEIGHT];
extern bool imgflash;
void LowInitializeWorldMap();
void buildlowrawmap();