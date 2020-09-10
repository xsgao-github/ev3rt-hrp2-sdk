#include <stdlib.h>
#include <stdio.h>
#include "ev3api.h"
#include "app.h"
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

// define motors and sensors
const int color_1 = EV3_PORT_2, color_2 = EV3_PORT_2, color_3 = EV3_PORT_3, color_4 = EV3_PORT_4, left_motor = EV3_PORT_B, right_motor = EV3_PORT_C, a_motor = EV3_PORT_A, d_motor = EV3_PORT_D;

// declare methods
void run2020();
void runBlueStreet(int backToBase);
void runGreenStreet(int backToBase);
void runYellowStreet(int backToBase);
void runRedStreet(int backToBase);
void goBackToBase(int fromBlueStreet);
void readCode();
void linePID_with_tasks(int distance, int speed);
void color3PID(int distance,int tasksNumA,int tasksNumD);
void wall_follow_with_tasks(int distance,int steer,int detectCar,int tasksNumA,int tasksNumD,int speed);
void execute_tasks(float distance);
void waitforButton();
void writeInstructions(int doSnow, int doCar, int doAbrasive, int detectCar, int snowDepot, int carDepot, int collectAbrasive, int uTurn);
void init();
void display_sensors();
void displayValues(float text,int row,int collumn,int clearScreen);
void displayText(char text[100],int row,int collumn,int clearScreen);
void writeInstructions(int doSnow,int doCar,int doAbrasive,int detectCar,int snowDepot,int carDepot,int collectAbrasive,int uTurn);
static void button_clicked_handler(intptr_t button);

//declare global variables
rgb_raw_t rgb1;
rgb_raw_t rgb4;
position pos = {-1, -1};
directions instructions = {0,0,0,0,0,0,0,0};
/**
 * Instructions for robot
 * 
 * Index_2 - Street [BLUE_STREET, GREEN_STREET, YELLOW_STREET, RED_STREET]
 * Index_2 - Task or Taskdone
**/
int tasks[4][2] = {{-1, 0}, {-1, 0}, {-1, 0}, {-1, 0}};
/**
 * All task directions written to here
 * 
 * Index 1 - Street [BLUE_STREET, GREEN_STREET, YELLOW_STREET, RED_STREET]
 * Index 2 - Sensor/Motor [COLOR_4, A_MOTOR, D_MOTOR]
 * Index 3 - index of task (0-5), with 7th spacer 6
 * Index 4 - data:
 * ----------------COLOR_4-[distance at read (cm), null, null]
 * ----------------A_MOTOR-[distance at execute (cm), distance at return (cm), degrees to rotate]
 * ----------------D_MOTOR-[distance at execute (cm), number of abrasive to dispense (int), null]
**/
int allTasks[4][3][7][3] = {
    //blue
    {
        //color_4
        {
            //index 0
            {
                1000,0,0
            },
            //index 1
            {
                1000,0,0
            },
            //index 2
            {
                1000,0,0
            },
            //index 3
            {
                1000,0,0
            },
            //index 4
            {
                1000,0,0
            },
            //index 5
            {
                1000,0,0
            },
            //index 6
            {
                1000,0,0
            },
        },
        //a_motor
        {
            //index 0
            {
                5,18,280
            },
            //index 1
            {
                48, 55, 100
            },
            //index 2
            {
                1000,0,0 // spacer
            },
            //index 3
            {
                15, 1000, 300 // sort of spacer
            },
            //index 4
            {
                1000,0,0
            },
            //index 5
            {
                1000,0,0
            },
            //index 6
            {
                1000,0,0
            },
        },
        //d_motor
        {
            //index 0
            {
                40,1,0
            },
            //index 1
            {
                80,1,0
            },
            //index 2
            {
                1000,0,0 // spacer
            },
            //index 3
            {
                25,2,0
            },
            //index 4
            {
                1000,0,0
            },
            //index 5
            {
                1000,0,0
            },
            //index 6
            {
                1000,0,0
            },
        },
    },
    //green
    {
        //color_4
        {
            //index 0
            {
                1000,0,0
            },
            //index 1
            {
                1000,0,0
            },
            //index 2
            {
                1000,0,0
            },
            //index 3
            {
                1000,0,0
            },
            //index 4
            {
                1000,0,0
            },
            //index 5
            {
                1000,0,0
            },
            //index 6
            {
                1000,0,0
            },
        },
        //a_motor
        {
            //index 0
            {
                0,13,400
            },
            //index 1
            {
                1000,0,0 // spacer
            },
            //index 2
            {
                8,15,400
            },
            //index 3
            {
                1000,10,300 // arbitrary bad idea
            },
            //index 4
            {
                15,25,0
            },
            //index 5
            {
                1000,0,0
            },
            //index 6
            {
                1000,0,0
            },
        },
        //d_motor
        {
            //index 0
            {
                40,1,0
            },
            //index 1
            {
                80,1,0
            },
            //index 2
            {
                1000,0,0 // spacer
            },
            //index 3
            {
                25,2,0
            },
            //index 4
            {
                1000,0,0
            },
            //index 5
            {
                1000,0,0
            },
            //index 6
            {
                1000,0,0
            },
        },
    },
    //yellow
    {
        //color_4
        {
            //index 0
            {
                41,53,0
            },
            //index 1
            {
                14,26,0
            },
            //index 2
            {
                1000,0,0
            },
            //index 3
            {
                1000,0,0
            },
            //index 4
            {
                1000,0,0
            },
            //index 5
            {
                1000,0,0
            },
            //index 6
            {
                1000,0,0
            },
        },
        //a_motor
        {
            //index 0
            {
                43,60,400
            },
            //index 1
            {
                3,16,200
            },
            //index 2
            {
                30,37,450
            },
            //index 3
            {
                1000,0,0
            },
            //index 4
            {
                1000,0,0
            },
            //index 5
            {
                1000,0,0
            },
            //index 6
            {
                1000,0,0
            },
        },
        //d_motor
        {
            //index 0
            {
                10,1,0
            },
            //index 1
            {
                20,1,0
            },
            //index 2
            {
                5,2,0
            },
            //index 3
            {
                1000,0,0
            },
            //index 4
            {
                1000,0,0
            },
            //index 5
            {
                1000,0,0
            },
            //index 6
            {
                1000,0,0
            },
        },
    },
    //red
    {
        //color_4
        {
            //index 0
            {
                13,25,0
            },
            //index 1
            {
                77,89,0
            },
            //index 2
            {
                1000,0,0
            },
            //index 3
            {
                1000,0,0
            },
            //index 4
            {
                1000,0,0
            },
            //index 5
            {
                1000,0,0
            },
            //index 6
            {
                1000,0,0
            },
        },
        //a_motor
        {
            //index 0
            {
                28,44,350
            },
            //index 1
            {
                60,85,200
            },
            //index 2
            {
                93,112,350
            },
            //index 3
            {
                0,7,400
            },
            //index 4
            {
                1000,0,0
            },
            //index 5
            {
                1000,0,0
            },
            //index 6
            {
                1000,0,0
            },
        },
        //d_motor
        {
            //index 0
            {
                5,1,0
            },
            //index 1
            {
                80,1,0
            },
            //index 2
            {
                0,1,0
            },
            //index 3
            {
                15,1,0
            },
            //index 4
            {
                1000,0,0
            },
            //index 5
            {
                1000,0,0
            },
            //index 6
            {
                1000,0,0
            },
        },
    },
};
/**
 * Position at where the car was detected
 * 
 * Index 1 - Car detected [0,1,2]
**/
int carDetected[4] = {
    //blue
    -1,
    //green
    -1,
    //yellow
    -1,
    //red
    -1,
};

int mute_task_running = false;
int color_4_index = 0;
int next_color_4_task[3] = {0,0,0};
int a_motor_index = 0;
int a_task_running = false;
int next_a_motor_task[3] = {0,0,0};
int d_motor_index = 0;
int d_task_running = false;
int next_d_motor_task[3] = {0,0,0};
int back_loaded = false; // false, BLUEMATERIAL, BLACKMATERIAL
int car_motor_index = 0;

void main_task(intptr_t unused) {
    init();
    readCode();
    writeInstructions(0,0,1,0,0,0,0,0);
    runBlueStreet(0);
    //run2020();
    goBackToBase(false);
}

void run2020() {
    if (pos.street == RED_STREET){
        if(tasks[RED_STREET][0] == 0 && tasks[YELLOW_STREET][0] == 0){
            if(tasks[GREEN_STREET][0] == 1){
                writeInstructions(1,0,0,1,1,0,2,0);
                runRedStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(1,0,0,1,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(0,0,0,0,1,0,0,0);
                runRedStreet(1);
                goBackToBase(RED_STREET);
            }
            else if(tasks[BLUE_STREET][0] == 1){
                writeInstructions(1,0,0,1,1,0,1,0);
                runRedStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(1,0,0,1,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(0,0,0,0,1,0,0,0);
                runRedStreet(1);
                goBackToBase(RED_STREET);
            }
        }
        else if(tasks[RED_STREET][0] == 0 && tasks[GREEN_STREET][0] == 0){
            if(tasks[YELLOW_STREET][0] == 1){
                writeInstructions(1,0,0,1,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(1,0,0,1,1,0,2,1);
                runRedStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runYellowStreet(1);
                goBackToBase(RED_STREET);
            }
            else if(tasks[BLUE_STREET][0] == 1){
                writeInstructions(1,0,0,1,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(1,0,0,1,1,0,1,1);
                runRedStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runYellowStreet(1);
                goBackToBase(RED_STREET);
            }
        }
        else if(tasks[RED_STREET][0] == 0 && tasks[BLUE_STREET][0] == 0){
            if(tasks[YELLOW_STREET][0] == 1){
                writeInstructions(1,0,0,1,1,0,1,0);
                runRedStreet(0);
                writeInstructions(1,0,0,1,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(0,0,0,0,1,0,0,0);
                runRedStreet(1);
                goBackToBase(RED_STREET);
            }
            else if(tasks[GREEN_STREET][0] == 1){
                writeInstructions(1,0,0,1,1,0,2,0);
                runRedStreet(0);
                writeInstructions(1,0,0,1,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(0,0,0,0,1,0,0,0);
                runRedStreet(1);
                goBackToBase(RED_STREET);
            }
        }
        else if(tasks[YELLOW_STREET][0] == 0 && tasks[GREEN_STREET][0] == 0){
            if(tasks[RED_STREET][0] == 1){
                writeInstructions(1,0,0,1,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(0,0,0,0,1,0,2,0);
                runRedStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(1,0,0,1,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,1,0,1,0,0,0);
                runRedStreet(1);
                goBackToBase(RED_STREET);
            }
            else if(tasks[BLUE_STREET][0] == 1){
                writeInstructions(1,0,0,1,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(0,0,0,0,1,0,1,0);
                runRedStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(1,0,0,1,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,1,0,1,0,0,0);
                runRedStreet(1);
                goBackToBase(RED_STREET);
            }
        }
        else if(tasks[YELLOW_STREET][0] == 0 && tasks[BLUE_STREET][0] == 0){
            if(tasks[RED_STREET][0] == 1){
                writeInstructions(0,0,0,0,0,0,2,0);
                runRedStreet(0);
                writeInstructions(1,0,0,1,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(1,0,0,1,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(0,0,1,0,1,0,0,0);
                runRedStreet(1);
                goBackToBase(RED_STREET);
            }
            else if(tasks[GREEN_STREET][0] == 1){
                writeInstructions(0,0,0,0,0,0,1,0);
                runRedStreet(0);
                writeInstructions(1,0,0,1,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(1,0,0,1,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(0,0,1,0,1,0,0,0);
                runRedStreet(1);
                goBackToBase(RED_STREET);
            }
        }
        else if(tasks[GREEN_STREET][0] == 0 && tasks[BLUE_STREET][0] == 0){
            if(tasks[RED_STREET][0] == 1){
                writeInstructions(1,0,0,1,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(0,0,0,0,1,0,2,0);
                runRedStreet(0);
                writeInstructions(1,0,0,1,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,1,0,1,0,0,0);
                runRedStreet(1);
                goBackToBase(RED_STREET);
            }
            else if(tasks[YELLOW_STREET][0] == 1){
                writeInstructions(1,0,0,1,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(0,0,0,0,1,0,1,0);
                runRedStreet(0);
                writeInstructions(1,0,0,1,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,1,0,1,0,0,0);
                runRedStreet(1);
                goBackToBase(RED_STREET);
            }
        }
    }
    else if (pos.street == YELLOW_STREET){
        if(tasks[RED_STREET][0] == 0 && tasks[YELLOW_STREET][0] == 0){
            if(tasks[GREEN_STREET][0] == 1){
                writeInstructions(1,0,0,1,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(1,0,0,1,1,0,2,1);
                runRedStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(0,0,0,0,0,1,0,0);
                runRedStreet(1);
                goBackToBase(RED_STREET);
            }
            else if(tasks[BLUE_STREET][0] == 1){
                writeInstructions(1,0,0,1,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(1,0,0,1,1,0,1,1);
                runRedStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(0,0,0,0,0,1,0,0);
                runRedStreet(1);
                goBackToBase(RED_STREET);
            }
        }
        else if(tasks[RED_STREET][0] == 0 && tasks[GREEN_STREET][0] == 0){
            if(tasks[YELLOW_STREET][0] == 1){
                writeInstructions(0,0,0,0,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(1,0,0,1,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(1,0,0,1,1,0,2,0);
                runRedStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,0,0,0,1,0,0);
                runRedStreet(1);
                goBackToBase(RED_STREET);
            }
            else if(tasks[BLUE_STREET][0] == 1){
                writeInstructions(0,0,0,0,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(1,0,0,1,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(1,0,0,1,1,0,1,0);
                runRedStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,0,0,0,1,0,0);
                runRedStreet(1);
                goBackToBase(RED_STREET);
            }
        }
        else if(tasks[RED_STREET][0] == 0 && tasks[BLUE_STREET][0] == 0){
            if(tasks[YELLOW_STREET][0] == 1){
                writeInstructions(1,0,0,1,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(0,0,0,0,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(1,0,0,1,1,0,1,1);
                runRedStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runGreenStreet(1);
                goBackToBase(RED_STREET);
            }
            else if(tasks[GREEN_STREET][0] == 1){
                writeInstructions(1,0,0,1,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(0,0,0,0,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(1,0,0,1,1,0,2,1);
                runRedStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runGreenStreet(1);
                goBackToBase(RED_STREET);
            }
        }
        else if(tasks[YELLOW_STREET][0] == 0 && tasks[GREEN_STREET][0] == 0){
            if(tasks[RED_STREET][0] == 1){
                writeInstructions(1,0,0,1,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(1,0,0,1,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(0,0,0,0,1,0,2,0);
                runRedStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(0,0,0,0,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runRedStreet(1);
                goBackToBase(RED_STREET);
            }
            else if(tasks[BLUE_STREET][0] == 1){
                writeInstructions(1,0,0,1,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(1,0,0,1,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(0,0,0,0,1,0,1,0);
                runRedStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(0,0,0,0,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runRedStreet(1);
                goBackToBase(RED_STREET);
            }
        }
        else if(tasks[YELLOW_STREET][0] == 0 && tasks[BLUE_STREET][0] == 0){
            if(tasks[RED_STREET][0] == 1){
                writeInstructions(1,0,0,1,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(1,0,0,1,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,0,0,1,0,2,0);
                runRedStreet(0);
                writeInstructions(0,0,0,0,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(0,0,1,0,0,1,0,0);
                runRedStreet(1);
                goBackToBase(RED_STREET);
            }
            else if(tasks[GREEN_STREET][0] == 1){
                writeInstructions(1,0,0,1,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(1,0,0,1,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,0,0,1,0,1,0);
                runRedStreet(0);
                writeInstructions(0,0,0,0,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(0,0,1,0,0,1,0,0);
                runRedStreet(1);
                goBackToBase(RED_STREET);
            }
        }
        else if(tasks[GREEN_STREET][0] == 0 && tasks[BLUE_STREET][0] == 0){
            if(tasks[RED_STREET][0] == 1){
                writeInstructions(1,0,0,1,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(0,0,0,0,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(1,0,0,1,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(0,0,0,0,1,0,2,0);
                runRedStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,1,0,0,1,0,0);
                runRedStreet(1);
                goBackToBase(RED_STREET);
            }
            else if(tasks[YELLOW_STREET][0] == 1){
                writeInstructions(1,0,0,1,0,0,0,0);
                runBlueStreet(0);
                writeInstructions(0,0,0,0,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(1,0,0,1,0,0,0,0);
                runGreenStreet(0);
                writeInstructions(0,0,0,0,1,0,1,0);
                runRedStreet(0);
                writeInstructions(0,0,1,0,0,0,0,0);
                runYellowStreet(0);
                writeInstructions(0,0,1,0,0,1,0,0);
                runRedStreet(1);
                goBackToBase(RED_STREET);
            }
        }
    }
}
void runBlueStreet(int backToBase) {
    color_4_index = 0;
    a_motor_index = 0;
    d_motor_index = 0;
    pos.street = YELLOW_STREET;

    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    ev3_motor_steer(left_motor, right_motor, 40, 1);
    //wall_follow_with_tasks(77, 3, 0, 1, 0, 30);
    //wall_follow_with_tasks(79, 3, 0, 1, 0, 30);
    wall_follow_with_tasks(78, 3, 0, 1, 0, 30);
    ev3_motor_set_power(a_motor, -50);
    tslp_tsk(100);
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    ev3_motor_steer(left_motor, right_motor, -20, 0);
    while (((ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor)) / 2) > -150) {
        display_sensors();
    }
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    tslp_tsk(100);
    //ev3_motor_rotate(right_motor, 340, 30, true);
    ev3_motor_rotate(right_motor, 350, 30, true);
    tslp_tsk(100);
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    ev3_motor_steer(left_motor, right_motor, 20, 0);
    while (((ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor)) / 2) < 40) {
        display_sensors();
    }
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    ev3_motor_stop(a_motor, false);
    a_motor_index = 0;
    d_motor_index = 0;
    pos.street = BLUE_STREET;
    tslp_tsk(100);
    if (instructions.doSnow == 1) {
        linePID_with_tasks(60, 25);
        tslp_tsk(100);
        ev3_motor_rotate(right_motor, 75, 20, true);
        a_motor_index++;
        ev3_motor_reset_counts(left_motor);
        ev3_motor_reset_counts(right_motor);
        ev3_motor_steer(left_motor, right_motor, 25, 0);
        while ((((abs(ev3_motor_get_counts(left_motor)) + abs(ev3_motor_get_counts(right_motor))) / 2) * ((3.1415926535 * 8.1) / 360)) < 20) {
            execute_tasks((((abs(ev3_motor_get_counts(left_motor)) + abs(ev3_motor_get_counts(right_motor))) / 2) * ((3.1415926535 * 8.1) / 360)));
        }
        ev3_motor_steer(left_motor, right_motor, 0, 0);
        tslp_tsk(100);
        ev3_motor_steer(left_motor, right_motor, 10, -1);
        while (ev3_color_sensor_get_reflect(color_2) > 15) {
            display_sensors();
        }
        tslp_tsk(150);
        ev3_motor_steer(left_motor, right_motor, -10, 0);
        ev3_motor_set_power(a_motor, -80);
        tslp_tsk(250);
        ev3_motor_rotate(right_motor, 235, 20, true);
        ev3_motor_rotate(a_motor, 500, 80, true);
        tslp_tsk(100);
        linePID_with_tasks(31, 25);
        ev3_motor_set_power(a_motor, -50);
    }
    else if (instructions.doCar == 1) {
        ev3_motor_set_power(a_motor, 50);
        linePID_with_tasks(84, 30);
        ev3_motor_set_power(a_motor, -50);
        tslp_tsk(100);
        ev3_motor_steer(left_motor, right_motor, 10, -1);
        while (ev3_color_sensor_get_reflect(color_2) > 15) {
            display_sensors();
        }
        tslp_tsk(150);
        ev3_motor_steer(left_motor, right_motor, -10, 0);
        tslp_tsk(200);
        ev3_motor_rotate(right_motor, 235, 20, true);
        ev3_motor_stop(a_motor, false);
        tslp_tsk(100);
        ev3_motor_rotate(right_motor, 90, 20, true);
        ev3_motor_rotate(left_motor, 90, 20, true);
        ev3_motor_reset_counts(left_motor);
        ev3_motor_reset_counts(right_motor);
        ev3_motor_steer(left_motor, right_motor, 20, 5);
        while (((ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor)) / 2) < 50) {
            display_sensors();
        }
        ev3_motor_steer(left_motor, right_motor, 0, 0);
        ev3_motor_rotate(left_motor, 90, 20, true);
        ev3_motor_rotate(right_motor, 90, 20, true);
        tslp_tsk(100);
        linePID_with_tasks(18, 20);
        tslp_tsk(100);
    }
    else if (instructions.doAbrasive == 1) {
        linePID_with_tasks(84, 30);
        ev3_motor_steer(left_motor, right_motor, 10, -1);
        while (ev3_color_sensor_get_reflect(color_2) > 15) {
            display_sensors();
        }
        tslp_tsk(150);
        ev3_motor_steer(left_motor, right_motor, -10, 0);
        tslp_tsk(200);
        ev3_motor_rotate(right_motor, 235, 20, true);
        tslp_tsk(100);
        d_motor_index++;
        linePID_with_tasks(32, 30);
    }
    else {
        ev3_speaker_play_tone(NOTE_G6, -1);
        while (true);
    }
    ev3_motor_rotate(left_motor, 130, 20, false);
    ev3_motor_rotate(right_motor, 130, 20, true);
    ev3_motor_stop(a_motor, false);
    tslp_tsk(100);
    ev3_motor_steer(left_motor, right_motor, 30, -40);
    tslp_tsk(850);
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    tslp_tsk(100);
    ev3_motor_steer(left_motor, right_motor, 20, 10);
    tslp_tsk(1000);
    ev3_motor_steer(left_motor, right_motor, -20, 12);
    tslp_tsk(1000);
    ev3_motor_steer(left_motor, right_motor, -20, -10);
    tslp_tsk(700);
    ev3_motor_steer(left_motor, right_motor, -20, 0);
    tslp_tsk(100);
    ev3_motor_steer(left_motor, right_motor, 20, 8);
    while (ev3_color_sensor_get_reflect(color_3) > 30) {
        display_sensors();
    }
    tslp_tsk(100);
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    if(carDetected[pos.street] == -1){
        carDetected[pos.street] = 3;
    }
    pos.street = YELLOW_STREET;
}
void runGreenStreet(int backToBase) {
    color_4_index = 0;
    a_motor_index = 0;
    d_motor_index = 0;
    pos.street = RED_STREET;

    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    wall_follow_with_tasks(53, 3, 0, 1, 0, 30);
    tslp_tsk(100);
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    ev3_motor_steer(left_motor, right_motor, -20, 0);
    while (((ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor)) / 2) > -150) {
        display_sensors();
    }
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    tslp_tsk(100);
    ev3_motor_rotate(right_motor, 340, 30, true);
    tslp_tsk(100);
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    ev3_motor_steer(left_motor, right_motor, 20, 0);
    while (((ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor)) / 2) < 40) {
        display_sensors();
    }
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    a_motor_index = 0;
    d_motor_index = 0;
    pos.street = GREEN_STREET;
    tslp_tsk(100);
    if (instructions.doSnow == 1) {
        linePID_with_tasks(25, 25);
        tslp_tsk(100);
        ev3_motor_rotate(right_motor, 80, 20, true);
        ev3_motor_rotate(left_motor, 160, 20, true);
        ev3_motor_rotate(right_motor, 80, 20, true);
        tslp_tsk(100);
        a_motor_index++;
        linePID_with_tasks(22, 25);
        tslp_tsk(100);
        ev3_motor_rotate(right_motor, 70, 10, true);
        ev3_motor_rotate(left_motor, 100, 20, false);
        ev3_motor_rotate(right_motor, 100, 20, true);
        ev3_motor_rotate(right_motor, 35, 10, true);
        ev3_motor_rotate(left_motor, 175, 20, false);
        ev3_motor_rotate(right_motor, 175, 20, true);
        tslp_tsk(100);
        //linePID_with_tasks(20, 25);
        tslp_tsk(100);
        ev3_motor_rotate(a_motor, 300, 80, false);
        ev3_motor_steer(left_motor, right_motor, 10, -1);
        while (ev3_color_sensor_get_reflect(color_2) > 15) {
            display_sensors();
        }
        tslp_tsk(150);
        ev3_motor_steer(left_motor, right_motor, -10, 0);
        tslp_tsk(150);
        ev3_motor_rotate(right_motor, 220, 20, true);
        tslp_tsk(100);
        a_task_running = 1;
        linePID_with_tasks(25, 25);
    }
    else if (instructions.doCar == 1) {
        ev3_motor_set_power(a_motor, 50);
        linePID_with_tasks(84, 30);
        ev3_motor_set_power(a_motor, -50);
        tslp_tsk(100);
        ev3_motor_steer(left_motor, right_motor, 10, -1);
        while (ev3_color_sensor_get_reflect(color_2) > 15) {
            display_sensors();
        }
        tslp_tsk(150);
        ev3_motor_steer(left_motor, right_motor, -10, 0);
        tslp_tsk(200);
        ev3_motor_rotate(right_motor, 235, 20, true);
        ev3_motor_stop(a_motor, false);
        tslp_tsk(100);
        ev3_motor_rotate(left_motor, 80, 10, false);
        ev3_motor_rotate(right_motor, 80, 10, true);
        ev3_motor_rotate(right_motor, 85, 20, true);
        ev3_motor_rotate(left_motor, 90, 20, true);
        ev3_motor_reset_counts(left_motor);
        ev3_motor_reset_counts(right_motor);
        ev3_motor_steer(left_motor, right_motor, 20, 5);
        while (((ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor)) / 2) < 10) {
            display_sensors();
        }
        ev3_motor_steer(left_motor, right_motor, 0, 0);
        ev3_motor_rotate(left_motor, 90, 20, true);
        ev3_motor_rotate(right_motor, 85, 20, true);
        tslp_tsk(100);
        linePID_with_tasks(14, 20);
        tslp_tsk(100);
    }
    else if (instructions.doAbrasive == 1) {
        linePID_with_tasks(84, 30);
        tslp_tsk(100);
        ev3_motor_steer(left_motor, right_motor, 10, -1);
        while (ev3_color_sensor_get_reflect(color_2) > 15) {
            display_sensors();
        }
        tslp_tsk(150);
        ev3_motor_steer(left_motor, right_motor, -10, 0);
        tslp_tsk(150);
        ev3_motor_rotate(right_motor, 235, 20, true);
        tslp_tsk(100);
        d_motor_index++;
        linePID_with_tasks(30, 30);
    }
    else {
        ev3_speaker_play_tone(NOTE_G6, -1);
        while (true);
    }
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    ev3_motor_rotate(left_motor, 180, 20, false);
    ev3_motor_rotate(right_motor, 180, 20, true);
    tslp_tsk(100);
    ev3_motor_rotate(right_motor, 210, 20, true);
    tslp_tsk(100);
    ev3_motor_steer(left_motor, right_motor, 20, 3);
    tslp_tsk(1000);
    ev3_motor_steer(left_motor, right_motor, 20, 1);
    while (ev3_color_sensor_get_reflect(color_3) > 30) {
        display_sensors();
    }
    tslp_tsk(100);
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    if(carDetected[pos.street] == -1){
        carDetected[pos.street] = 3;
    }
    pos.street = RED_STREET;
}
void runYellowStreet(int backToBase) {
    //doCar
    if(instructions.doCar == 0){
        color_4_index = 0;
        a_motor_index = 0;
        d_motor_index = 0;
        //Main length
        int abrasive = 0;
        int car = 0;
        if(instructions.doAbrasive){
            abrasive = 1;
        }
        if(instructions.detectCar){
            car = 1;
        }
        wall_follow_with_tasks(90,3,car,1,abrasive,25);
        wall_follow_with_tasks(42,3,car,2,abrasive,10);
    }
    else if(instructions.doCar == 1){
        ev3_motor_reset_counts(left_motor);
        ev3_motor_reset_counts(right_motor);
        ev3_motor_reset_counts(a_motor);
        ev3_motor_reset_counts(d_motor);
        float wheelDistance = -100;
        int taskIndex = 0;
        //switch
        switch(carDetected[pos.street]){
            case 0:
                ev3_motor_steer(left_motor,right_motor,35,3);
                while(wheelDistance < 132){
                    wheelDistance = (ev3_motor_get_counts(left_motor) / 2 + ev3_motor_get_counts(right_motor) / 2) * ((3.1415926535 * 8.1) / 360);
                    tslp_tsk(1);
                }
                ev3_motor_steer(left_motor,right_motor,0,0);
                break;
            case 1:
                ev3_motor_set_power(a_motor,80);
                ev3_motor_steer(left_motor,right_motor,25,3);
                while(wheelDistance < 132){
                    if(wheelDistance > 100 && taskIndex == 0){
                        ev3_motor_set_power(a_motor,-80);
                        taskIndex = 1;
                    }
                    wheelDistance = (ev3_motor_get_counts(left_motor) / 2 + ev3_motor_get_counts(right_motor) / 2) * ((3.1415926535 * 8.1) / 360);
                    tslp_tsk(1);
                }
                ev3_motor_steer(left_motor,right_motor,0,0);
                carDetected[pos.street] = 0;
                break;
            case 2:
                ev3_motor_steer(left_motor,right_motor,25,3);
                while(wheelDistance < 132){
                    if(wheelDistance > 85 && taskIndex == 0){
                        ev3_motor_set_power(a_motor,80);
                        taskIndex = 1;
                    }
                    if(wheelDistance > 125 && taskIndex == 1){
                        ev3_motor_set_power(a_motor,-80);
                        taskIndex = 2;
                    }
                    wheelDistance = (ev3_motor_get_counts(left_motor) / 2 + ev3_motor_get_counts(right_motor) / 2) * ((3.1415926535 * 8.1) / 360);
                    tslp_tsk(1);
                }
                carDetected[pos.street] = 0;
                ev3_motor_steer(left_motor,right_motor,0,0);
                break;
            case 3:
                ev3_motor_steer(left_motor,right_motor,25,3);
                while(wheelDistance < 94){
                    wheelDistance = (ev3_motor_get_counts(left_motor) / 2 + ev3_motor_get_counts(right_motor) / 2) * ((3.1415926535 * 8.1) / 360);
                    tslp_tsk(1);
                }
                ev3_motor_steer(left_motor,right_motor,0,0);
                ev3_motor_reset_counts(left_motor);
                ev3_motor_reset_counts(right_motor);
                ev3_motor_steer(left_motor,right_motor,20,-45);
                tslp_tsk(500);
                ev3_motor_steer(left_motor,right_motor,0,0);
                ev3_motor_steer(left_motor,right_motor,20,45);
                tslp_tsk(800);
                ev3_motor_steer(left_motor,right_motor,0,0);
                ev3_motor_steer(left_motor,right_motor,20,-45);
                tslp_tsk(500);
                ev3_motor_steer(left_motor,right_motor,0,0);
                ev3_motor_reset_counts(left_motor);
                ev3_motor_reset_counts(right_motor);
                wheelDistance = 0;
                ev3_motor_steer(left_motor,right_motor,25,5);
                while(wheelDistance < 32){
                    if(wheelDistance > 20){
                        ev3_motor_set_power(a_motor,-80);
                    }
                    wheelDistance = (ev3_motor_get_counts(left_motor) / 2 + ev3_motor_get_counts(right_motor) / 2) * ((3.1415926535 * 8.1) / 360);
                    tslp_tsk(1);
                }
                carDetected[pos.street] = 0;
                ev3_motor_steer(left_motor,right_motor,0,0);
                break;
        }
    }
    //move forward
    //ev3_motor_steer(left_motor,right_motor,30,0);
    //tslp_tsk(100);
    //ev3_motor_steer(left_motor,right_motor,0,0);
    //turn 1
    ev3_motor_steer(left_motor,right_motor,30,-45);
    tslp_tsk(600);
    ev3_motor_steer(left_motor,right_motor,0,0);
    //move forward
    ev3_motor_steer(left_motor,right_motor,30,0);
    tslp_tsk(800);
    ev3_motor_steer(left_motor,right_motor,0,0);
    //turn 2
    ev3_motor_steer(left_motor,right_motor,30,-45);
    tslp_tsk(250);
    ev3_motor_steer(left_motor,right_motor,0,0);
    //Side Length
    if(instructions.doAbrasive){
        wall_follow_with_tasks(35,3,0,0,1,35);
    }
    else{
        wall_follow_with_tasks(35,3,0,0,0,35);
    }
    //detect line
    ev3_motor_steer(left_motor,right_motor,30,0);
    while (ev3_color_sensor_get_reflect(color_3) > 30) {
    }
    ev3_motor_steer(left_motor,right_motor,0,0);
    if(backToBase){
        //move forward
        ev3_motor_steer(left_motor,right_motor,30,0);
        tslp_tsk(1000);
        ev3_motor_steer(left_motor,right_motor,0,0);
    }
    else{
        //move forward
        ev3_motor_steer(left_motor,right_motor,30,0);
        tslp_tsk(250);
        ev3_motor_steer(left_motor,right_motor,0,0);
        //turn
        ev3_motor_steer(left_motor,right_motor,30,-45);
        tslp_tsk(800);
        ev3_motor_steer(left_motor,right_motor,0,0);
        //move forward
        ev3_motor_reset_counts(left_motor);
        ev3_motor_reset_counts(right_motor);
        float wheelDistance = 0;
        ev3_motor_steer(left_motor,right_motor,40,3);
        while(wheelDistance < 42){
            wheelDistance = (ev3_motor_get_counts(left_motor) / 2 + ev3_motor_get_counts(right_motor) / 2) * ((3.1415926535 * 8.1) / 360);
        }
        ev3_motor_steer(left_motor,right_motor,30,3);
        //detect line
        while (ev3_color_sensor_get_reflect(color_3) > 30) {
        }
        ev3_motor_steer(left_motor,right_motor,0,0);
        if(carDetected[pos.street] == -1 && instructions.detectCar){
            carDetected[pos.street] = 3;
        }
        pos.street = RED_STREET;
    }
}
void runRedStreet(int backToBase) {
    //doCar
    if(instructions.doCar == 0){
        color_4_index = 0;
        a_motor_index = 0;
        d_motor_index = 0;
        //Main Length
        int abrasive = 0;
        int car = 0;
        if(instructions.doAbrasive){
            abrasive = 2;
        }
        if(instructions.detectCar){
            car = 2;
        }
        if(instructions.doSnow){
            wall_follow_with_tasks(130,3,car,3,abrasive,25);
        }
        else{
            a_motor_index = 1;
            wall_follow_with_tasks(130,3,car,1,abrasive,50);
        }
    }
    else if(instructions.doCar == 1){
        ev3_motor_reset_counts(left_motor);
        ev3_motor_reset_counts(right_motor);
        ev3_motor_reset_counts(a_motor);
        ev3_motor_reset_counts(d_motor);
        float wheelDistance = -100;
        int taskIndex = 0;
        //switch
        displayValues(carDetected[pos.street],1,1,1);
        switch(carDetected[pos.street]){
            case 0:
                ev3_motor_steer(left_motor,right_motor,35,3);
                while(wheelDistance < 128){
                    wheelDistance = (ev3_motor_get_counts(left_motor) / 2 + ev3_motor_get_counts(right_motor) / 2) * ((3.1415926535 * 8.1) / 360);
                    tslp_tsk(1);
                }
                ev3_motor_steer(left_motor,right_motor,0,0);
                break;
            case 1:
                ev3_motor_set_power(a_motor,80);
                ev3_motor_steer(left_motor,right_motor,25,3);
                while(wheelDistance < 128){
                    if(wheelDistance > 20 && taskIndex == 0){
                        ev3_motor_set_power(a_motor,-80);
                        taskIndex = 1;
                    }
                    wheelDistance = (ev3_motor_get_counts(left_motor) / 2 + ev3_motor_get_counts(right_motor) / 2) * ((3.1415926535 * 8.1) / 360);
                    tslp_tsk(1);
                }
                carDetected[pos.street] = 0;
                ev3_motor_steer(left_motor,right_motor,0,0);
                break;
            case 2:
                ev3_motor_set_power(a_motor,80);
                ev3_motor_steer(left_motor,right_motor,25,3);
                while(wheelDistance < 128){
                    if(wheelDistance > 100 && taskIndex == 0){
                        ev3_motor_set_power(a_motor,-80);
                        taskIndex = 1;
                    }
                    wheelDistance = (ev3_motor_get_counts(left_motor) / 2 + ev3_motor_get_counts(right_motor) / 2) * ((3.1415926535 * 8.1) / 360);
                    tslp_tsk(1);
                }
                carDetected[pos.street] = 0;
                ev3_motor_steer(left_motor,right_motor,0,0);
                break;
            case 3:
                ev3_motor_steer(left_motor,right_motor,25,3);
                while(wheelDistance < 68){
                    wheelDistance = (ev3_motor_get_counts(left_motor) / 2 + ev3_motor_get_counts(right_motor) / 2) * ((3.1415926535 * 8.1) / 360);
                    tslp_tsk(1);
                }
                ev3_motor_steer(left_motor,right_motor,0,0);
                ev3_motor_reset_counts(left_motor);
                ev3_motor_reset_counts(right_motor);
                ev3_motor_steer(left_motor,right_motor,20,-45);
                tslp_tsk(450);
                ev3_motor_steer(left_motor,right_motor,0,0);
                ev3_motor_steer(left_motor,right_motor,20,45);
                tslp_tsk(350);
                ev3_motor_steer(left_motor,right_motor,0,0);
                ev3_motor_reset_counts(left_motor);
                ev3_motor_reset_counts(right_motor);
                wheelDistance = 0;
                ev3_motor_steer(left_motor,right_motor,25,5);
                while(wheelDistance < 50){
                    if(wheelDistance > 20){
                        ev3_motor_set_power(a_motor,-80);
                    }
                    wheelDistance = (ev3_motor_get_counts(left_motor) / 2 + ev3_motor_get_counts(right_motor) / 2) * ((3.1415926535 * 8.1) / 360);
                    tslp_tsk(1);
                }
                carDetected[pos.street] = 0;
                ev3_motor_steer(left_motor,right_motor,0,0);
                break;
        }
    }
    if(instructions.snowDepot){
        //move amotor
        ev3_motor_set_power(a_motor,50);
        tslp_tsk(800);
        ev3_motor_set_power(a_motor,0);
        //detect line
        ev3_motor_steer(left_motor, right_motor, 10, 5);
        while(ev3_color_sensor_get_reflect(color_3) > 35){
            
        }
        //move backwards
        ev3_motor_steer(left_motor,right_motor,-30,0);
        tslp_tsk(500);
        ev3_motor_steer(left_motor,right_motor,0,0);
        //turn amotor back and turn
        ev3_motor_set_power(a_motor,-50);
        ev3_motor_steer(left_motor,right_motor,-15,75);
        tslp_tsk(600);
        ev3_motor_set_power(a_motor,50);
        tslp_tsk(500);
        ev3_motor_steer(left_motor,right_motor,0,0);
        //turn amotor back completely
        ev3_motor_set_power(a_motor,-50);
        tslp_tsk(700);
        ev3_motor_set_power(a_motor,0);
        //back up
        ev3_motor_steer(left_motor,right_motor,-10,0);
        tslp_tsk(1000);
        ev3_motor_steer(left_motor,right_motor,0,0);
    }
    else if(instructions.carDepot){
        //turn
        ev3_motor_steer(left_motor,right_motor,-20,90);
        tslp_tsk(250);
        ev3_motor_steer(left_motor,right_motor,0,0);
        //move and back
        ev3_motor_steer(left_motor,right_motor,30,0);
        tslp_tsk(1100);
        ev3_motor_steer(left_motor,right_motor,0,0);
        ev3_motor_steer(left_motor,right_motor,-30,0);
        tslp_tsk(1200);
        ev3_motor_steer(left_motor,right_motor,0,0);
        if(instructions.doSnow){
            //turn amotor
            ev3_motor_set_power(a_motor,80);
            tslp_tsk(500);
        }
        //turn again
        ev3_motor_steer(left_motor,right_motor,-30,90);
        tslp_tsk(350);
        ev3_motor_steer(left_motor,right_motor,0,0);
        if(instructions.doSnow){
            //turn amotor
            ev3_motor_set_power(a_motor,-80);
            tslp_tsk(500);
        }
        //back up
        ev3_motor_steer(left_motor,right_motor,-10,0);
        tslp_tsk(2000);
        ev3_motor_steer(left_motor,right_motor,0,0);
    }
    else if(!instructions.snowDepot && !instructions.carDepot){
        //move backwards
        ev3_motor_steer(left_motor,right_motor,-30,0);
        tslp_tsk(100);
        ev3_motor_steer(left_motor,right_motor,0,0);
        //turn amotor back and turn
        ev3_motor_rotate(a_motor,200,80,true);
        ev3_motor_steer(left_motor,right_motor,-15,80);
        ev3_motor_rotate(a_motor,100,80,false);
        tslp_tsk(1100);
        ev3_motor_steer(left_motor,right_motor,0,0);
        //turn amotor back completely
        ev3_motor_set_power(a_motor,-80);
        tslp_tsk(700);
        ev3_motor_set_power(a_motor,0);
        //back up
        ev3_motor_steer(left_motor,right_motor,-10,0);
        tslp_tsk(2000);
        ev3_motor_steer(left_motor,right_motor,0,0);
    }
    if(backToBase){
        //move forward
        ev3_motor_steer(left_motor,right_motor,30,1);
        tslp_tsk(3000);
        ev3_motor_steer(left_motor,right_motor,0,0);
        //turn
        ev3_motor_steer(left_motor,right_motor,30,45);
        tslp_tsk(700);
        ev3_motor_steer(left_motor,right_motor,0,0);
        //move forward
        ev3_motor_steer(left_motor,right_motor,30,1);
        tslp_tsk(2500);
        ev3_motor_steer(left_motor,right_motor,0,0);
    }
    else{
        ev3_motor_steer(left_motor,right_motor,10,0);
        tslp_tsk(1000);
        ev3_motor_steer(left_motor,right_motor,0,0);
        ev3_motor_steer(left_motor,right_motor,10,45);
        tslp_tsk(1700);
        ev3_motor_steer(left_motor,right_motor,0,0);
        ev3_motor_steer(left_motor, right_motor, 20, 0);
        while (ev3_color_sensor_get_reflect(color_3) > 35) {
            tslp_tsk(10);
        }
        ev3_motor_steer(left_motor,right_motor,0,0);
        ev3_motor_steer(left_motor, right_motor, -20, 0);
        tslp_tsk(250);
        ev3_motor_steer(left_motor,right_motor,0,0);
        ev3_motor_steer(left_motor,right_motor,10,-45);
        tslp_tsk(1800);
        ev3_motor_steer(left_motor,right_motor,0,0);
        int abrasive = 0;
        if(instructions.doAbrasive){
            abrasive = 2;
        }
        a_motor_index = 3;
        //Side Length
        color3PID(40,0,abrasive);
        ev3_speaker_play_tone(NOTE_A4,60);
        //detect line
        ev3_motor_steer(left_motor, right_motor, 15, 0);
        while (ev3_color_sensor_get_reflect(color_2) > 35) {
            tslp_tsk(10);
        }
        ev3_motor_steer(left_motor,right_motor,0,0);
        ev3_motor_steer(left_motor,right_motor,10,0);
        tslp_tsk(1000);
        ev3_motor_steer(left_motor,right_motor,0,0);
        ev3_motor_steer(left_motor,right_motor,-10,0);
        tslp_tsk(1000);
        ev3_motor_steer(left_motor,right_motor,0,0);
        if(instructions.uTurn && 0){
            ev3_motor_steer(left_motor,right_motor,15,90);
            tslp_tsk(1700);
            ev3_motor_steer(left_motor,right_motor,0,0);
            ev3_motor_steer(left_motor,right_motor,30,0);
            tslp_tsk(2500);
            ev3_motor_steer(left_motor,right_motor,0,0);
            ev3_motor_steer(left_motor,right_motor,15,-45);
            tslp_tsk(1300);
            ev3_motor_steer(left_motor,right_motor,0,0);
            //move amotor
            ev3_motor_set_power(a_motor,50);
            tslp_tsk(800);
            ev3_motor_set_power(a_motor,0);
            //detect line
            ev3_motor_steer(left_motor, right_motor, 10, 5);
            while(ev3_color_sensor_get_reflect(color_3) > 40){
                tslp_tsk(10);
            }
            //move backwards
            ev3_motor_steer(left_motor,right_motor,-30,0);
            tslp_tsk(500);
            ev3_motor_steer(left_motor,right_motor,0,0);
            //turn amotor back and turn
            ev3_motor_set_power(a_motor,-50);
            ev3_motor_steer(left_motor,right_motor,-15,75);
            tslp_tsk(600);
            ev3_motor_set_power(a_motor,50);
            tslp_tsk(500);
            ev3_motor_steer(left_motor,right_motor,0,0);
            //turn amotor back completely
            ev3_motor_set_power(a_motor,-50);
            tslp_tsk(700);
            ev3_motor_set_power(a_motor,0);
            //back up
            ev3_motor_steer(left_motor,right_motor,-10,0);
            tslp_tsk(1000);
            ev3_motor_steer(left_motor,right_motor,0,0);
            //back up
            ev3_motor_steer(left_motor,right_motor,10,0);
            tslp_tsk(1500);
            ev3_motor_steer(left_motor,right_motor,0,0);
            ev3_motor_rotate(a_motor,400,50,false);
            ev3_motor_steer(left_motor,right_motor,10,45);
            tslp_tsk(1700);
            ev3_motor_steer(left_motor,right_motor,0,0);
            ev3_motor_steer(left_motor,right_motor,10,-45);
            tslp_tsk(1400);
            ev3_motor_steer(left_motor,right_motor,0,0);
            //Side Length
            color3PID(37,0,0);
            //detect line
            ev3_motor_steer(left_motor, right_motor, 15, 0);
            while (ev3_color_sensor_get_reflect(color_3) > 35) {
                tslp_tsk(10);
            }
            ev3_motor_steer(left_motor,right_motor,0,0);
        }
        if(instructions.collectAbrasive == 1){
            ev3_motor_steer(left_motor,right_motor,-15,0);
            tslp_tsk(2700);
            ev3_motor_steer(left_motor,right_motor,0,0);
            ev3_motor_set_power(a_motor,80);
            //ev3_motor_steer(left_motor,right_motor,15,-45);
            //tslp_tsk(1690);
            //ev3_motor_steer(left_motor,right_motor,0,0);
            //while (abs(((ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor)) / 2)) < 280) {
                //display_sensors();
            //}
            //while (abs(ev3_motor_get_counts(right_motor)) < 220) {
            //}
            ev3_motor_rotate(right_motor,220,10,true);
            ev3_motor_steer(left_motor,right_motor,-10,0);
            tslp_tsk(1900);
            ev3_motor_steer(left_motor,right_motor,0,0);
            ev3_motor_steer(left_motor,right_motor,10,0);
            tslp_tsk(1900);
            ev3_motor_steer(left_motor,right_motor,0,0);
            //ev3_motor_steer(left_motor,right_motor,-15,-45);
            //tslp_tsk(1690);
            //ev3_motor_steer(left_motor,right_motor,0,0);
            //ev3_motor_rotate(right_motor,220,10,true);
            ev3_motor_rotate(right_motor,220,-10,true);
            ev3_motor_set_power(a_motor,-80);
            color3PID(25,0,0);
            ev3_motor_steer(left_motor,right_motor,-15,90);
            tslp_tsk(845);
            ev3_motor_steer(left_motor,right_motor,0,0);
            ev3_motor_steer(left_motor,right_motor,-10,0);
            tslp_tsk(1000);
            ev3_motor_steer(left_motor,right_motor,0,0);
            ev3_motor_steer(left_motor,right_motor,10,0);
            tslp_tsk(1000);
            ev3_motor_steer(left_motor,right_motor,0,0);
            ev3_motor_steer(left_motor,right_motor,15,90);
            tslp_tsk(845);
            ev3_motor_steer(left_motor,right_motor,0,0);
            ev3_motor_set_power(a_motor,-80);
            ev3_motor_steer(left_motor, right_motor, 15, 0);
            while (ev3_color_sensor_get_reflect(color_3) > 35) {
                tslp_tsk(10);
            }
            ev3_motor_steer(left_motor,right_motor,0,0);
        }
        
        if(instructions.collectAbrasive == 2){
            ev3_motor_steer(left_motor,right_motor,-15,0);
            tslp_tsk(1000);
            ev3_motor_steer(left_motor,right_motor,0,0);
            ev3_motor_set_power(a_motor,80);
            ev3_motor_steer(left_motor,right_motor,15,-45);
            tslp_tsk(1690);
            ev3_motor_steer(left_motor,right_motor,0,0);
            ev3_motor_steer(left_motor,right_motor,-10,0);
            tslp_tsk(1900);
            ev3_motor_steer(left_motor,right_motor,0,0);
            ev3_motor_steer(left_motor,right_motor,10,0);
            tslp_tsk(1900);
            ev3_motor_steer(left_motor,right_motor,0,0);
            ev3_motor_steer(left_motor,right_motor,-15,-45);
            tslp_tsk(1690);
            ev3_motor_steer(left_motor,right_motor,0,0);
            ev3_motor_set_power(a_motor,-80);
            ev3_motor_steer(left_motor,right_motor,-15,0);
            tslp_tsk(1000);
            ev3_motor_steer(left_motor,right_motor,0,0);
            ev3_motor_steer(left_motor,right_motor,-15,90);
            tslp_tsk(845);
            ev3_motor_steer(left_motor,right_motor,0,0);
            ev3_motor_steer(left_motor,right_motor,-10,0);
            tslp_tsk(1000);
            ev3_motor_steer(left_motor,right_motor,0,0);
            ev3_motor_steer(left_motor,right_motor,10,0);
            tslp_tsk(1000);
            ev3_motor_steer(left_motor,right_motor,0,0);
            ev3_motor_steer(left_motor,right_motor,15,90);
            tslp_tsk(845);
            ev3_motor_steer(left_motor,right_motor,0,0);
            ev3_motor_set_power(a_motor,-80);
            color3PID(25,0,0);
            ev3_motor_steer(left_motor, right_motor, 15, 0);
            while (ev3_color_sensor_get_reflect(color_3) > 35) {
                tslp_tsk(10);
            }
            ev3_motor_steer(left_motor,right_motor,0,0);
        }
        
        //turn amotor back completely
        ev3_motor_set_power(a_motor,50);
        tslp_tsk(300);
        ev3_motor_set_power(a_motor,0);
        //move forward
        ev3_motor_steer(left_motor,right_motor,30,0);
        tslp_tsk(500);
        ev3_motor_steer(left_motor,right_motor,0,0);
        //turn
        ev3_motor_steer(left_motor,right_motor,30,-45);
        tslp_tsk(650);
        ev3_motor_steer(left_motor,right_motor,0,0);
        //turn amotor back completely
        ev3_motor_set_power(a_motor,-50);
        //move forward
        ev3_motor_steer(left_motor,right_motor,20,5);
        tslp_tsk(1000);
        ev3_motor_steer(left_motor,right_motor,0,0);
        ev3_motor_set_power(a_motor,0);
        //detect line
        ev3_motor_steer(left_motor, right_motor, 30, 3);
        while (ev3_color_sensor_get_reflect(color_3) > 30) {
            tslp_tsk(10);
        }
        ev3_motor_steer(left_motor,right_motor,0,0);
        if(carDetected[pos.street] == -1 && instructions.detectCar){
            carDetected[pos.street] = 3;
        }
        pos.street = YELLOW_STREET;
    }
}
/**
 * \brief goes back to home
 * \param street RED_STREET or false
**/
void goBackToBase(int street) {
    if (street == RED_STREET) {
        ev3_motor_steer(left_motor,right_motor,-30, 5);
        ev3_motor_set_power(a_motor, 100);
        tslp_tsk(1200);
        ev3_motor_stop(a_motor, false);
        ev3_motor_steer(left_motor, right_motor, -30, -4);
        tslp_tsk(1200);
        ev3_motor_steer(left_motor, right_motor, 30, 5);
        tslp_tsk(1000);
        ev3_motor_steer(left_motor, right_motor, -30, 2);
        tslp_tsk(2000);
    }
    else if (street == YELLOW_STREET) {
        //empty
    }
    else {
        ev3_motor_steer(left_motor,right_motor,-30, 0);
        ev3_motor_set_power(a_motor, 100);
        tslp_tsk(1200);
        ev3_motor_stop(a_motor, false);
        tslp_tsk(4200);
    }

    ev3_lcd_fill_rect(0, 0, 178, 128, EV3_LCD_WHITE);
    ev3_motor_stop(left_motor, false);
    ev3_motor_stop(right_motor, false);
    ev3_motor_stop(a_motor, false);
    ev3_motor_stop(d_motor, false);
    ev3_led_set_color(LED_GREEN);
    ev3_speaker_set_volume(100);
    ev3_speaker_play_tone(NOTE_C5, 500);
    ev3_lcd_draw_string("Program Finished", 10, 50);
    ev3_lcd_draw_string("Flashing Lights", 15, 65);
    while (true) {
        ev3_led_set_color(LED_GREEN);
        tslp_tsk(200);
        ev3_led_set_color(LED_ORANGE);
        tslp_tsk(200);
        ev3_led_set_color(LED_RED);
        tslp_tsk(200);
    }
}

void readCode() {
    // define array & variable
    int values[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
    int i;
    rgb_raw_t rgb3;

    //leave start

    /*readColorCode start
    float wheelDistance = 0;
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    while(wheelDistance < 22){
        ev3_motor_steer(left_motor,right_motor,30,5);
        wheelDistance = (ev3_motor_get_counts(left_motor) / 2 + ev3_motor_get_counts(right_motor) / 2) * ((3.1415926535 * 8.1) / 360);
    }
    ev3_motor_steer(left_motor, right_motor, 5, 5);
    colorid_t color3color = 6;
    while(color3color == 6){
        color3color = ev3_color_sensor_get_color(color_3);
        tslp_tsk(10);
    }
    // stop d_motor
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    int x = 0;
    while(x < 50){
        color3color = ev3_color_sensor_get_color(color_3);
        x += 1;
        tslp_tsk(15);
        displayValues(color3color,1,1,1);
    }
    if(color3color == 5){
        pos.street = RED_STREET;
        ev3_speaker_play_tone(NOTE_G4, 40);
    }
    if(color3color == 4){
        pos.street = YELLOW_STREET;
        ev3_speaker_play_tone(NOTE_G5, 40);
    }
    */
    ///*readCode start
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    ev3_motor_steer(left_motor, right_motor, 30, 2);
    while (abs(((ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor)) / 2)) < 280) {
        display_sensors();
    }

    // detect line
    ev3_motor_steer(left_motor, right_motor, 7, 1);
    while (rgb3.b > 60) {
        ev3_color_sensor_get_rgb_raw(color_3, &rgb3);
        displayValues(rgb3.g, 1, 1, 1);
        tslp_tsk(15);
    }
    ev3_motor_steer(left_motor, right_motor, 0, 0);

    // stop d_motor
    ev3_motor_stop(d_motor, false);

    // write down road
    for (i = 0; i < 50; i++) {
        ev3_color_sensor_get_rgb_raw(color_3, &rgb3);
        displayValues(rgb3.g, 0, 0, 1);
        tslp_tsk(15);
    }
    if (rgb3.g < 110) {
        pos.street = RED_STREET;
        ev3_speaker_play_tone(NOTE_G4, 40);
    } else {
        pos.street = YELLOW_STREET;
        ev3_speaker_play_tone(NOTE_G5, 40);
    }
    //*/

    tslp_tsk(100);
    // record instructions
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    ev3_motor_steer(left_motor, right_motor, 10, 3);
    ht_nxt_color_sensor_measure_rgb(color_4, &rgb4);
    tslp_tsk(100);
    for (i = 0; i < 8; i++) {
        // read instructions
        while (abs(((ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor)) / 2)) < (i * 58)) {
            display_sensors();
        }
        ht_nxt_color_sensor_measure_rgb(color_4, &rgb4);
        if (((rgb4.r + rgb4.g + rgb4.b) / 3) > 40) {
            values[i] = 1;
            ev3_speaker_play_tone(NOTE_C5, 50);
        } else {
            values[i] = 0;
            ev3_speaker_play_tone(NOTE_C4, 50);
        }
    }
    i = 0;
    while (i < 8) {
    // decode instructions
        if (values[i] == 1) {
            if (values[i + 1] == 1) {
                ev3_speaker_play_tone(NOTE_G6, -1);
            } else {
                tasks[i / 2][0] = BLACKMATERIAL;
            }
        } else {
            if (values[i + 1] == 1) {
                tasks[i / 2][0] = BLUEMATERIAL;
            } else {
                tasks[i / 2][0] = COLLECTSNOW;
            }
        }
        i += 2;
    }

    // detect line
    ev3_motor_steer(left_motor, right_motor, 10, 1);
    while (ev3_color_sensor_get_reflect(color_3) > 30) {
        display_sensors();
        tslp_tsk(10);
    }
    tslp_tsk(100);
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    tslp_tsk(5);

    // display things in a very medium font
    char lcdstr[100];
    sprintf(lcdstr, "%d, %d", tasks[BLUE_STREET][0], tasks[GREEN_STREET][0]);
    ev3_lcd_draw_string(lcdstr, 0, 15);
    sprintf(lcdstr, "%d, %d", tasks[YELLOW_STREET][0], tasks[RED_STREET][0]);
    ev3_lcd_draw_string(lcdstr, 0, 30);
    if (pos.street == RED_STREET) {
        ev3_lcd_draw_string("RED_STREET", 0, 45);
    } else if (pos.street == YELLOW_STREET) {
        ev3_lcd_draw_string("YELLOW_STREET", 0, 45);
    }
}

/**
 * \brief follows a solid line using a PID and does tasks
 * \param distance Distance in cm
 * \param speed speed abs (recommended 25 with tasks, 30 without)
**/
void linePID_with_tasks(int distance, int speed){
    ev3_lcd_fill_rect(0, 0, 178, 128, EV3_LCD_WHITE);

    float PIDvalues[2][3] = {
        //25
        {
            0.03,0.2,3.8
        },
        //30
        {
            //0.03,0.2,2.8
            0.03,0.2,2.5
        }
    };

    int pidIndex = 0;
    if(speed == 25){
        pidIndex = 0;
    }
    else{
        pidIndex = 1;
    }

    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    ev3_motor_reset_counts(a_motor);
    ev3_motor_reset_counts(d_motor);
    float wheelDistance = (((abs(ev3_motor_get_counts(left_motor)) + abs(ev3_motor_get_counts(right_motor))) / 2) * ((3.1415926535 * 8.1) / 360));
    float lasterror = 0, integral = 0;
    while (wheelDistance < distance) {
        execute_tasks(wheelDistance);
        //if(ev3_motor_get_counts(a_motor) > 490){
        //    ev3_motor_reset_counts(a_motor);
        //    ev3_motor_rotate(a_motor,-500,13,false);
        //    
        //}
        //if(ev3_motor_get_counts(a_motor) < -490){
        //    ev3_motor_reset_counts(a_motor);
        //    ev3_motor_rotate(a_motor,500,13,false);
        //}
        wheelDistance = (((abs(ev3_motor_get_counts(left_motor)) + abs(ev3_motor_get_counts(right_motor))) / 2) * ((3.1415926535 * 8.1) / 360));
        float error = ev3_color_sensor_get_reflect(color_2) - ev3_color_sensor_get_reflect(color_3);
        integral = error + integral * 0.6;
        float steer = PIDvalues[pidIndex][0] * error + PIDvalues[pidIndex][1] * integral + PIDvalues[pidIndex][2] * (error - lasterror);
        ev3_motor_steer(left_motor, right_motor, speed, steer);
        lasterror = error;
    }
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    tslp_tsk(5);
    return;
}

/**
 * \brief follows a line using Color_4 and does tasks
 * \param distance Distance in cm
 * \param tasksNumA amount of tasks for A_Motor
 * \param tasksNumD amount of tasks for D_Motor
**/
void color3PID(int distance,int tasksNumA,int tasksNumD){
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    ev3_motor_reset_counts(a_motor);
    ev3_motor_reset_counts(d_motor);
    int isTurningA = 0;
    int a_motorStopped = 0;
    float wheelDistance = 0;
    int tasksLeftA = tasksNumA;
    int tasksLeftD = tasksNumD;
    for(int i = 0;i < 3;i++){
        next_a_motor_task[i] = allTasks[pos.street][1][a_motor_index][i];
    }
    for(int i = 0;i < 3;i++){
        next_d_motor_task[i] = allTasks[pos.street][2][d_motor_index][i];
    }
    float lasterror = 0, integral = 0;
    while (wheelDistance < distance) {
        if(ev3_motor_get_power(a_motor) == 0 && ev3_motor_get_counts(a_motor) < 10 && a_motorStopped == 0){
            ev3_motor_stop(a_motor,false);
            a_motorStopped = 1;
        }
        if(wheelDistance > next_a_motor_task[0] && tasksLeftA > 0 && isTurningA == 0){
            ev3_motor_reset_counts(a_motor);
            ev3_motor_rotate(a_motor,next_a_motor_task[2],80,false);
            isTurningA = 1;
        }
        if(wheelDistance > next_a_motor_task[1] && tasksLeftA > 0 && isTurningA == 1){
            ev3_motor_set_power(a_motor,-80);
            a_motor_index += 1;
            a_motorStopped = 0;
            for(int i = 0;i < 3;i++){
                next_a_motor_task[i] = allTasks[pos.street][1][a_motor_index][i];
            }
            isTurningA = 0;
            tasksLeftA -= 1;
        }
        if(wheelDistance > next_d_motor_task[0] && tasksLeftD > 0){
            ev3_speaker_play_tone(NOTE_A4,60);
            ev3_motor_rotate(d_motor,next_d_motor_task[1] * 360,20,false);
            d_motor_index += 1;
            for(int i = 0;i < 3;i++){
                next_d_motor_task[i] = allTasks[pos.street][2][d_motor_index][i];
            }
            tasksLeftD -= 1;
        }
        wheelDistance = (ev3_motor_get_counts(left_motor) / 2 + ev3_motor_get_counts(right_motor) / 2) * ((3.1415926535 * 8.1) / 360);
        float error = ev3_color_sensor_get_reflect(color_3) - 43;
        integral = error + integral * 0.5;
        float steer = 0.5 * error + 0.01 * integral + 0.1 * (error - lasterror);
        ev3_motor_steer(left_motor, right_motor, 15, steer);
        lasterror = error;
        tslp_tsk(10);
    }
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    return;
}

/**
 * \brief follows a wall with a wall follower and does tasks
 * \param distance Distance in cm
 * \param steer Steer amount, ranging from 0 to 100
 * \param detectCar do you detect car
 * \param tasksNumA amount of tasks for A_Motor
 * \param tasksNumD amount of tasks for D_Motor
 * \param speed speed abs (recommended 25 with tasks, 30 without, sometimes 15)
**/
void wall_follow_with_tasks(int distance,int steer,int detectCar,int tasksNumA,int tasksNumD, int speed){
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    ev3_motor_reset_counts(a_motor);
    ev3_motor_reset_counts(d_motor);
    int lastDash = 0;
    int isTurningA = 0;
    int a_motorStopped = 0;
    float wheelDistance = -100;
    int tasksLeft4 = detectCar;
    int tasksLeftA = tasksNumA;
    int tasksLeftD = tasksNumD;
    int i = 0;
    for(i = 0;i < 3;i++){
        next_color_4_task[i] = allTasks[pos.street][0][color_4_index][i];
    }
    for(i = 0;i < 3;i++){
        next_a_motor_task[i] = allTasks[pos.street][1][a_motor_index][i];
    }
    for(i = 0;i < 3;i++){
        next_d_motor_task[i] = allTasks[pos.street][2][d_motor_index][i];
    }
    ev3_motor_steer(left_motor,right_motor,speed,steer);
    while (wheelDistance < distance) {
        if(ev3_motor_get_power(a_motor) == 0 && ev3_motor_get_counts(a_motor) < 10 && a_motorStopped == 0){
            ev3_motor_stop(a_motor,false);
            a_motorStopped = 1;
        }
        if(wheelDistance > next_color_4_task[0] && tasksLeft4 > 0){
            bool_t val = ht_nxt_color_sensor_measure_rgb(color_4,  &rgb4);
            assert(val);
            if(rgb4.b > 75){
                ev3_speaker_play_tone(NOTE_C5,60);
                carDetected[pos.street] = color_4_index + 1;
            }
            displayValues(rgb4.r,1,1,1);
            displayValues(rgb4.g,2,1,0);
            displayValues(rgb4.b,3,1,0);
        }
        if(wheelDistance > next_color_4_task[1] && tasksLeft4 > 0){
            tasksLeft4 -= 1;
            color_4_index += 1;
            for(int i = 0;i < 3;i++){
                next_color_4_task[i] = allTasks[pos.street][0][color_4_index][i];
            }
        }
        if(wheelDistance > next_a_motor_task[0] && tasksLeftA > 0 && isTurningA == 0){
            ev3_motor_reset_counts(a_motor);
            ev3_motor_rotate(a_motor,next_a_motor_task[2],80,false);
            isTurningA = 1;
        }
        if(wheelDistance > next_a_motor_task[1] && tasksLeftA > 0 && isTurningA == 1){
            ev3_motor_set_power(a_motor,-50);
            a_motor_index += 1;
            a_motorStopped = 0;
            for(int i = 0;i < 3;i++){
                next_a_motor_task[i] = allTasks[pos.street][1][a_motor_index][i];
            }
            isTurningA = 0;
            tasksLeftA -= 1;
        }
        if(wheelDistance > next_d_motor_task[0] && tasksLeftD > 0){
            ev3_speaker_play_tone(NOTE_A4,60);
            ev3_motor_rotate(d_motor,360 * next_d_motor_task[1],20,false);
            d_motor_index += 1;
            for(int i = 0;i < 3;i++){
                next_d_motor_task[i] = allTasks[pos.street][2][d_motor_index][i];
            }
            tasksLeftD -= 1;
        }
        if(ev3_color_sensor_get_reflect(color_2) > 75 && pos.dash % 2 == 0 && wheelDistance > lastDash + 3){
            pos.dash += 1;
            lastDash = wheelDistance;
        }
        if(ev3_color_sensor_get_reflect(color_2) < 15 && pos.dash % 2 == 1 && wheelDistance > lastDash + 3){
            lastDash = wheelDistance;
            pos.dash += 1;
        }
        wheelDistance = (ev3_motor_get_counts(left_motor) / 2 + ev3_motor_get_counts(right_motor) / 2) * ((3.1415926535 * 8.1) / 360);
        tslp_tsk(1);
    }
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    return;
}

/**
 * \brief executes tasks
 * \param distance Distance of robot travelled (cm)
 * \param doCar Do we do car collection or not?
**/
void execute_tasks(float distance) {
    display_sensors();

    //check for a_motor task, execute task if task is to collect snow and it is time and collect cars is false
    //execute part 1 of task
    if (distance > allTasks[pos.street][A_MOTOR][a_motor_index][0] && a_task_running == 0 && tasks[pos.street][0] == COLLECTSNOW && instructions.doSnow == 1) {
        ev3_motor_stop(a_motor, false);
        ev3_motor_rotate(a_motor, allTasks[pos.street][A_MOTOR][a_motor_index][2], 80, false);
        a_task_running = 1;
        ev3_speaker_play_tone(NOTE_C4, 50);
    }
    //execute part 2 of task
    if (distance > allTasks[pos.street][A_MOTOR][a_motor_index][1] && a_task_running == 1 && tasks[pos.street][0] == COLLECTSNOW && instructions.doSnow == 1) {
        ev3_motor_stop(a_motor, false);
        ev3_motor_rotate(a_motor, -1*allTasks[pos.street][A_MOTOR][a_motor_index][2], 80, false);
        a_task_running = 0;
        a_motor_index++;
        ev3_speaker_play_tone(NOTE_C5, 50);
    }

    //check for d_motor task, execute task if task is to dispense material and back is loaded and it is time and it is the correct material
    //execute task
    if (distance > allTasks[pos.street][D_MOTOR][d_motor_index][0] && d_task_running == 0 && instructions.doAbrasive == 1) {
        ev3_motor_stop(d_motor, false);
        ev3_motor_rotate(d_motor, allTasks[pos.street][D_MOTOR][d_motor_index][1]*360, 30, false);
        //d_task_running = 1;
        d_motor_index++;
        ev3_speaker_play_tone(NOTE_G4, 50);

    }

    //check for color_4 task, execute if it is time
    if (distance > allTasks[pos.street][COLOR_4][color_4_index][0]) {
        if(rgb4.b > 70){
            ev3_speaker_play_tone(NOTE_E4, 50);
            carDetected[pos.street] = (color_4_index + 1);
            color_4_index++;
        }
    }
}

void waitforButton() {
    ev3_led_set_color(LED_OFF);
    while (1) {
        if (ev3_button_is_pressed(ENTER_BUTTON)) {
            while (ev3_button_is_pressed(ENTER_BUTTON));
            break;
        }
    }
    ev3_led_set_color(LED_GREEN);
}

/**
 * \brief literally writes parameters
 * \param doSnow robot collects snow on street if true
 * \param doCar robot collects cars on street if true
 * \param doAbrasive robot dispenses abrasive material on street if true
 * \param detectCar robot detects cars on street if true
 * \param snowDepot robot goes to snowDepot after running street | WARNING: only supported by runRedStreet
 * \param carDepot robot goes to carDepot (parking lot) after running street | WARNING: only supported by runRedStreet
 * \param collectAbrasive collects abrasive material after running street  | WARNING: only supported by runRedStreet
 * \param uTurn u-turns robot after street to go backwards  | WARNING: not supported at all by any street DO NOT USE
**/
void writeInstructions(int doSnow, int doCar, int doAbrasive, int detectCar, int snowDepot, int carDepot, int collectAbrasive, int uTurn) {
    instructions.doSnow = doSnow;
    instructions.doCar = doCar;
    instructions.doAbrasive = doAbrasive;
    instructions.detectCar = detectCar;
    instructions.snowDepot = snowDepot;
    instructions.carDepot = carDepot;
    instructions.collectAbrasive = collectAbrasive;
    instructions.uTurn = uTurn;
}

void init() {
    // Register button handlers
    ev3_button_set_on_clicked(BACK_BUTTON, button_clicked_handler, BACK_BUTTON);
    //ev3_button_set_on_clicked(DOWN_BUTTON, button_clicked_handler, DOWN_BUTTON);
    //ev3_button_set_on_clicked(UP_BUTTON, button_clicked_handler, UP_BUTTON);
    
    // Configure motors
    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);
    ev3_motor_config(a_motor, MEDIUM_MOTOR);
    ev3_motor_config(d_motor, MEDIUM_MOTOR);
    
    // Configure sensors
    ev3_sensor_config(color_2, COLOR_SENSOR);
    ev3_sensor_config(color_3, COLOR_SENSOR);
    ev3_sensor_config(color_4, HT_NXT_COLOR_SENSOR);
    
    // Set up sensors
    ev3_color_sensor_get_reflect(color_2);
    ev3_color_sensor_get_reflect(color_3);
    //bool_t val1 = ht_nxt_color_sensor_measure_rgb(color_1, &rgb1);
    //assert(val1);
    bool_t val4 = ht_nxt_color_sensor_measure_rgb(color_4, &rgb4);
    assert(val4);

    // Configure brick
    ev3_lcd_set_font(EV3_FONT_MEDIUM);
    ev3_speaker_set_volume(10);

    // reset snow/car collector and abrasive material spreader
    ev3_motor_set_power(a_motor, -100);
    //ev3_motor_set_power(d_motor, -100);
    tslp_tsk(1500);
    ev3_motor_stop(a_motor, false);
    //ev3_motor_stop(d_motor, false);

    // wait for button press
    ev3_lcd_draw_string("Press OK to run", 14, 45);
    ev3_lcd_fill_rect(77, 87, 24, 20, EV3_LCD_BLACK);
    //ev3_lcd_fill_rect(79, 89, 20, 1, EV3_LCD_WHITE);
    ev3_lcd_draw_string("OK", 79, 90);
    while (1) {
        if (ev3_button_is_pressed(ENTER_BUTTON)) {
            while (ev3_button_is_pressed(ENTER_BUTTON));
            break;
        }
    }
    ev3_lcd_fill_rect(0, 0, 178, 128, EV3_LCD_WHITE);
    //ev3_motor_set_power(d_motor, 100);
}

void display_sensors() {
    // declare variables
    char msg[100];
    int value;

    // wait for values to be refreshed
    tslp_tsk(10);

    // read motor counts
    value = ev3_motor_get_counts(left_motor);
    sprintf(msg, "L: %d   ", value);
    ev3_lcd_draw_string(msg, 10*0, 15*0);
    value = ev3_motor_get_counts(right_motor);
    sprintf(msg, "R: %d   ", value);
    ev3_lcd_draw_string(msg, 10*8, 15*0);

    // read sensor rgb1
    /*
    bool_t val1 = ht_nxt_color_sensor_measure_rgb(color_1, &rgb1);
    assert(val1);
    sprintf(msg, "RGB1:");
    ev3_lcd_draw_string(msg, 10*0, 15*1.5);
    sprintf(msg, "R: %d", rgb1.r);
    ev3_lcd_draw_string(msg, 10*0, /15*2.5);
    sprintf(msg, "G: %d", rgb1.g);
    ev3_lcd_draw_string(msg, 10*7, 15*2.5);
    sprintf(msg, "B: %d", rgb1.b);
    ev3_lcd_draw_string(msg, 10*14, 15*2.5);
    */

    // read sensor rgb4
    bool_t val4 = ht_nxt_color_sensor_measure_rgb(color_4, &rgb4);
    assert(val4);
    sprintf(msg, "RGB4:");
    ev3_lcd_draw_string(msg, 10*0, 15*4);
    sprintf(msg, "R: %d  ", rgb4.r);
    ev3_lcd_draw_string(msg, 10*0, 15*5);
    sprintf(msg, "G: %d  ", rgb4.g);
    ev3_lcd_draw_string(msg, 10*7, 15*5);
    sprintf(msg, "B: %d  ", rgb4.b);
    ev3_lcd_draw_string(msg, 10*14, 15*5);

    // read linefollow sensors
    sprintf(msg, "Light2 & Light3:");
    ev3_lcd_draw_string(msg, 10*0, 15*6.5);
    value = ev3_color_sensor_get_reflect(color_2);
    sprintf(msg, "L: %d  ", value);
    ev3_lcd_draw_string(msg, 10*0, 15*7.5);
    value = ev3_color_sensor_get_reflect(color_3);
    sprintf(msg, "L: %d  ", value);
    ev3_lcd_draw_string(msg, 10*7, 15*7.5);
}

void displayValues(float text,int row,int collumn,int clearScreen) {
    char lcdstr[100];
    
    if(clearScreen){
        ev3_lcd_fill_rect(0, 0, 178, 128, EV3_LCD_WHITE);
    }
    sprintf(lcdstr, "%f3", text);
    ev3_lcd_draw_string(lcdstr, collumn * 10, row * 15);
}
void displayText(char text[100],int row,int collumn,int clearScreen) {
    char lcdstr[100];
    
    if(clearScreen){
        ev3_lcd_fill_rect(0, 0, 178, 128, EV3_LCD_WHITE);
    }
    sprintf(lcdstr, "%s", text);
    ev3_lcd_draw_string(lcdstr, collumn * 10, row * 15);
}

static void button_clicked_handler(intptr_t button) {
    switch(button) {
    case BACK_BUTTON:
        ev3_lcd_fill_rect(0, 0, 178, 128, EV3_LCD_WHITE);
        ev3_lcd_draw_string("Stopping Program", 10, 60);
        ev3_motor_stop(left_motor, false);
        ev3_motor_stop(right_motor, false);
        ev3_motor_stop(a_motor, false);
        ev3_motor_stop(d_motor, false);
        ev3_led_set_color(LED_RED);
        ev3_speaker_set_volume(100);
        ev3_speaker_play_tone(250, 1000);
        ev3_lcd_draw_string("Program  Stopped", 10, 60);
        exit(0);
        break;
    case DOWN_BUTTON:
        ev3_motor_stop(left_motor, false);
        ev3_motor_stop(right_motor, false);
        ev3_motor_stop(a_motor, false);
        ev3_motor_stop(d_motor, false);
        ev3_led_set_color(LED_ORANGE);
        exit(0);
        break;
    case UP_BUTTON:
        if (mute_task_running) {
            ev3_speaker_set_volume(10);
            mute_task_running = 1;
        } else {
            ev3_speaker_set_volume(0);
            mute_task_running = 0;
        }
        ev3_button_set_on_clicked(UP_BUTTON, button_clicked_handler, UP_BUTTON);
    }
}
