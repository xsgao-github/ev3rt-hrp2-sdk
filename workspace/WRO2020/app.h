#include "target_test.h"

#ifndef _TEST_FILE_H_
#define _TEST_FILE_H_

#define BLUE_STREET 0
#define GREEN_STREET 1
#define YELLOW_STREET 2
#define RED_STREET 3

#define REMOVESNOW 0
#define BLUEMATERIAL 1
#define BLACKMATERIAL 2
#define TASKDONE 3

#define COLOR_4 0
#define A_MOTOR 1
#define D_MOTOR 2
#define CAR_MOTOR 3

/*
* street:
* RED, YELLOW, GREEN, or BLUE
* 
* section:
* RED and YELLOW: Section 1 is the area containing the instructions
* GREEN and BLUE: Section 1 is the area where the road goes diagonaly into their respective streets
* Section 1 is the "beginning" of the road, and 3 being the "end"
* 
* distance:
* Distance in centimeters from the start of the road
* 
* dash:
* (Only applicable for streets RED and YELLOW) Dashes from the start of the section, with a dash being a black line
* 
* facing:
* direction in degrees of where the robot is facing, 0 being facing the wall with the dispensers and snow depot, and ranges from -180 to 179.
*/
typedef struct {
	int street;
	int section;
	float distance;
	int dash;
	int facing;
} position;

static int tasks[4] = {0, 0, 0, 0};

extern void	main_task(intptr_t exinf);
#endif /* TOPPERS_MACRO_ONLY */
