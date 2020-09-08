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

#define COLOR_4 0
#define A_MOTOR 1
#define D_MOTOR 2

/**
 * \brief Struct for runStreets
 * \param Index_1: doSnow
 * \param Index_2: doCar
 * \param Index_3: doAbrasive
 * \param Index_4: detectCar
 * \param Index_5: snowDepot (Only for red street)
 * \param Index_6: carDepot (Only for red street)
 * \param Index_7: collectAbrasive (Only for red street)
 * \param Index_8: uTurn (Only for red street)
**/
typedef struct {
    int doSnow;
    int doCar;
    int doAbrasive;
    int detectCar;
    int snowDepot;
    int carDepot;
    int collectAbrasive;
    int uTurn;
} directions;

/**
 * 
**/
typedef struct {
    int street;
    int dash;
} position;


extern void	main_task(intptr_t exinf);
#endif /* TOPPERS_MACRO_ONLY */
