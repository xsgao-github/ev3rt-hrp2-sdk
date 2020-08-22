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

int tasks[4] = {-1, -1, -1, -1};
int current_Street = -1;

extern void	main_task(intptr_t exinf);
#endif /* TOPPERS_MACRO_ONLY */
