#include "target_test.h"

#ifndef _TEST_FILE_H_
#define _TEST_FILE_H_

#define BLUE_STREET 0
#define GREEN_STREET 1
#define YELLOW_STREET 2
#define RED_STREET 3

#define COLLECTSNOW 0
#define BLUEMATERIAL 1
#define BLACKMATERIAL 2
#define ROADDONE -1

#define COLOR_4 0
#define A_MOTOR 1
#define D_MOTOR 2

typedef struct {
int street;
int dash;
} position;

extern void	main_task(intptr_t exinf);
#endif /* TOPPERS_MACRO_ONLY */
