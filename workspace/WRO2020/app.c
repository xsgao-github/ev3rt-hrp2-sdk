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
void runBlueStreet();
void runGreenStreet();
void runYellowStreet();
void runRedStreet();
void readCode();
void readColorCode();
void linePID_with_tasks(int distance, int doCar);
void color4PID(int distance,int tasksNumA,int tasksNumD);
void wall_follow_with_tasks(int distance,int steer,int tasksNum4,int tasksNumA,int tasksNumD,int doCar,int speed);
void execute_tasks(float distance, int doCar);
void init();
void display_sensors();
static void button_clicked_handler(intptr_t button);

//declare global variables
rgb_raw_t rgb1;
rgb_raw_t rgb4;
position pos = {-1, -1};
/*
 * instructions for robot
 * Index 1 - Street [BLUE_STREET, GREEN_STREET, YELLOW_STREET, RED_STREET]
 * Index 2 - Task or Taskdone
*/
int tasks[4][2] = {{-1, 0}, {-1, 0}, {-1, 0}, {-1, 0}};
/*
 * All task directions written to here
 * Index 1 - Street [BLUE_STREET, GREEN_STREET, YELLOW_STREET, RED_STREET]
 * Index 2 - Sensor/Motor [COLOR_4, A_MOTOR, D_MOTOR]
 * Index 3 - index of task (0-5), with 7th spacer 6
 * Index 4 - data:
 * ---------------COLOR_4-[distance at read (cm), null, null]
 * ---------------A_MOTOR-[distance at execute (cm), distance at return (cm), degrees to rotate]
 * ---------------D_MOTOR-[distance at execute (cm), degrees to rotate, abrasive type]
*/
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
                17,23,350
            },
            //index 1
            {
                23,30,0
            },
            //index 2
            {
                52,59,190
            },
            //index 3
            {
                1,7,0
            },
            //index 4
            {
                15,17,0
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
                30,0,-600
            },
            //index 1
            {
                80,0,-900
            },
            //index 2
            {
                30,0,-1200
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
        //d_motor
        {
            //index 0
            {
                30,0,-600
            },
            //index 1
            {
                80,0,-900
            },
            //index 2
            {
                30,0,-1200
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
    //yellow
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
                47,60,400
            },
            //index 1
            {
                98,109,275
            },
            //index 2
            {
                120,133,450
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
    },
    //red
    {
        //color_4
        {
            //index 0
            {
                20,0,0
            },
            //index 1
            {
                85,0,0
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
                32,44,350
            },
            //index 1
            {
                71,89,275
            },
            //index 2
            {
                100,112,350
            },
            //index 3
            {
                17,29,500
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
    },
};
/*
* Tasks for current randomization.
* Index 1 - Street [BLUE_STREET, GREEN_STREET, YELLOW_STREET, RED_STREET]
* Index 2 - data:
* ---------------[distance at execute (cm), distance at return (cm), degrees to rotate]
*/
int carTasks[4][3] = {
    //blue
    {
        //car
        0,0,0
    },
    //green
    {
        //car
        0,0,0
    },
    //yellow
    {
        //car
        0,0,0
    },
    //red
    {
        //car
        0,0,0
    },
};
/*
 * how to get car?
 * Index 1 - Street [BLUE_STREET, GREEN_STREET, YELLOW_STREET, RED_STREET]
 * Index 2 - Car [Car 1 is car on other side, Car 2 is first car, Car 3 is second car]
 * Index 3 - data:
 * ---------------[distance at execute (cm), distance at return (cm), degrees to rotate]
*/
int carArray[4][3][3] = {
    //blue
    {
        //car 1
        {
            0,0,0
        },
        //car 2
        {
            0,0,0
        },
        //car 3
        {
            0,0,0
        },
    },
    //green
    {
        //car 1
        {
            0,0,0
        },
        //car 2
        {
            0,0,0
        },
        //car 3
        {
            0,0,0
        },
    },
    //yellow
    {
        //car 1
        {
            0,0,0
        },
        //car 2
        {
            0,0,0
        },
        //car 3
        {
            0,0,0
        },
    },
    //red
    {
        //car 1
        {
            0,0,0
        },
        //car 2
        {
            50,60,300
        },
        //car 3
        {
            0,0,0
        },
    },
};
/*
 * Position at where the car was detected
 * Index 1 - Car detected [0,1,2]
*/
int carDetected[4] = {
    //blue
    0,
    //green
    0,
    //yellow
    0,
    //red
    0,
};

int color_4_index = 0;
int next_color_4_task[3] = {0,0,0};
int a_motor_index = 0;
int next_a_motor_task[3] = {0,0,0};
int a_turning = false;
int d_motor_index = 0;
int next_d_motor_task[3] = {0,0,0};
int d_turning = false;
int back_loaded = false; // false, BLUEMATERIAL, BLACKMATERIAL
int car_motor_index = 0;
int round_index = 0;

void main_task(intptr_t unused) {
    init();
    readCode();
    //readColorCode();
    //run2020();
    runBlueStreet();
}
void run2020(){
    int road = 0;
    int i = 0;
    int j = 0;
    //road1
    while(road < 2){
        if (pos.street == RED_STREET){
            if (tasks[GREEN_STREET][0] == COLLECTSNOW){
                road += 1;
                runGreenStreet();
                tasks[GREEN_STREET][1] = 1;
            }
            else if (tasks[RED_STREET][0] == COLLECTSNOW){
                road += 1;
                runRedStreet();
                tasks[RED_STREET][1] = 1;
            }
            else{
                runRedStreet();
            }
        }
        else if (pos.street == YELLOW_STREET){
            if (tasks[BLUE_STREET][0] == COLLECTSNOW){
                road += 1;
                runBlueStreet();
                tasks[BLUE_STREET][1] = 1;
            }
            else if (tasks[YELLOW_STREET][0] == COLLECTSNOW){
                road += 1;
                runYellowStreet();
                tasks[YELLOW_STREET][1] = 1;
            }
            else{
                runYellowStreet();
            }
        }
    }
    round_index += 1;
    for (i = 0;i < 3;i++){
        for (j = 0;j < 3;j++){
            carTasks[i][j] = carArray[i][carDetected[i]][j];
        }
    }
    if (pos.street = RED_STREET){
        pos.street = YELLOW_STREET;
    }
    else if (pos.street = YELLOW_STREET){
        
    }
    round_index += 1;

}
void runBlueStreet(){
    color_4_index = 0;
    a_motor_index = 0;
    d_motor_index = 0;
    pos.street = YELLOW_STREET;

    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    ev3_motor_steer(left_motor, right_motor, 40, 1);
    wall_follow_with_tasks(77, 1, 1, 2, 0, false,25);
    ev3_motor_steer(left_motor, right_motor, 0, 0);
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
    while (((ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor)) / 2) < 80) {
        display_sensors();
    }
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    pos.street = BLUE_STREET;
    tslp_tsk(100);
    if (round_index != 0 && carDetected[BLUE_STREET] == 1) {
        //linePID_with_tasks(86, false);
        ev3_speaker_play_tone(NOTE_G6, 1000000000);
    } else {
        linePID_with_tasks(60, (round_index != 0 && tasks[BLUE_STREET][0] == COLLECTSNOW));
        tslp_tsk(100);
        ev3_motor_rotate(right_motor, 60, 20, true);
        ev3_motor_reset_counts(left_motor);
        ev3_motor_reset_counts(right_motor);
        ev3_motor_steer(left_motor, right_motor, 20, 0);
        while (((ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor)) / 2) < 320) {
            execute_tasks((((abs(ev3_motor_get_counts(left_motor)) + abs(ev3_motor_get_counts(right_motor))) / 2) * ((3.1415926535 * 8.1) / 360)), false);
        }
        ev3_motor_steer(left_motor, right_motor, 0, 0);
    }
    tslp_tsk(100);
    ev3_motor_rotate(right_motor, 10, 20, true);
    ev3_motor_steer(left_motor, right_motor, 10, -1);
    while (ev3_color_sensor_get_reflect(color_2) > 20) {
        display_sensors();
    }
    tslp_tsk(250);
    ev3_motor_steer(left_motor, right_motor, -10, 0);
    tslp_tsk(250);
    ev3_motor_rotate(right_motor, 190, 20, true);
    tslp_tsk(100);
    linePID_with_tasks(32, (round_index != 0 && tasks[BLUE_STREET][0] == 0));
    //ev3_motor_steer(left_motor, right_motor, 10, 0);
    //while (((ev3_color_sensor_get_reflect(color_2) + ev3_color_sensor_get_reflect(color_3)) / 2) > 30) {
    //    display_sensors();
    //}
    //ev3_motor_steer(left_motor, right_motor, 0, 0);
    ev3_motor_rotate(left_motor, 150, 20, false);
    ev3_motor_rotate(right_motor, 150, 20, true);
    tslp_tsk(100);
    ev3_motor_rotate(right_motor, 220, 20, true);
    tslp_tsk(100);
    //ev3_motor_steer(left_motor, right_motor, 20, 5);
    //tslp_tsk(1000);
    ev3_motor_steer(left_motor, right_motor, 10, 5);
    while (ev3_color_sensor_get_reflect(color_3) > 20) {
        display_sensors();
    }
    tslp_tsk(100);
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    pos.street = YELLOW_STREET;
}
void runGreenStreet(){
    color_4_index = 0;
    a_motor_index = 0;
    d_motor_index = 0;
    pos.street = RED_STREET;

    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    //ev3_motor_steer(left_motor, right_motor, 40, 1);
    //while (((ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor)) / 2) < 615) {
    //    display_sensors();
    //}
    //ev3_motor_steer(left_motor, right_motor, 0, 0);
    wall_follow_with_tasks(45, 1, 1, 2, 0, false,25);
    tslp_tsk(100);
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    ev3_motor_steer(left_motor, right_motor, -20, 0);
    while (((ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor)) / 2) > -15) {
        display_sensors();
    }
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    tslp_tsk(100);
    ev3_motor_rotate(right_motor, 340, 30, true);
    tslp_tsk(100);
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    ev3_motor_steer(left_motor, right_motor, 20, 0);
    while (((ev3_motor_get_counts(left_motor) + ev3_motor_get_counts(right_motor)) / 2) < 80) {
        display_sensors();
    }
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    pos.street = GREEN_STREET;
    linePID_with_tasks(86, (round_index != 0 && tasks[BLUE_STREET][0] == 0));
    ev3_motor_rotate(right_motor, 10, 20, true);
    ev3_motor_steer(left_motor, right_motor, 10, -1);
    while (ev3_color_sensor_get_reflect(color_2) > 20) {
        display_sensors();
    }
    tslp_tsk(250);
    ev3_motor_steer(left_motor, right_motor, -10, 0);
    tslp_tsk(250);
    ev3_motor_rotate(right_motor, 210, 20, true);
    tslp_tsk(100);
    linePID_with_tasks(38, (round_index != 0 && tasks[BLUE_STREET][0] == 0));
    //ev3_motor_steer(left_motor, right_motor, 10, 0);
    //while (((ev3_color_sensor_get_reflect(color_2) + ev3_color_sensor_get_reflect(color_3)) / 2) > 30) {
    //    display_sensors();
    //}
    //ev3_motor_steer(left_motor, right_motor, 0, 0);
    ev3_motor_rotate(left_motor, 150, 20, false);
    ev3_motor_rotate(right_motor, 150, 20, true);
    tslp_tsk(250);
    ev3_motor_rotate(right_motor, 220, 20, true);
    tslp_tsk(250);
    ev3_motor_steer(left_motor, right_motor, 20, 5);
    tslp_tsk(1000);
    ev3_motor_steer(left_motor, right_motor, 10, 1);
    while (ev3_color_sensor_get_reflect(color_3) > 20) {
        display_sensors();
    }
    tslp_tsk(100);
    ev3_motor_steer(left_motor, right_motor, 0, 0);

    pos.street = RED_STREET;
}
void runYellowStreet(){
    color_4_index = 0;
    a_motor_index = 0;
    d_motor_index = 0;
    wall_follow_with_tasks(100,3,0,1,0,0,25);
    wall_follow_with_tasks(40,3,0,2,0,0,10);
    ev3_motor_steer(left_motor,right_motor,30,-45);
    tslp_tsk(700);
    ev3_motor_steer(left_motor,right_motor,0,0);
    wall_follow_with_tasks(76,3,0,0,0,0,25);
    ev3_motor_steer(left_motor,right_motor,30,-45);
    tslp_tsk(700);
    ev3_motor_steer(left_motor,right_motor,0,0);
    ev3_motor_steer(left_motor, right_motor, 15, 5);
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    float wheelDistance = 0;
    ev3_motor_steer(left_motor,right_motor,15,0);
    while(wheelDistance < 44){
        wheelDistance = (ev3_motor_get_counts(left_motor) / 2 + ev3_motor_get_counts(right_motor) / 2) * ((3.1415926535 * 8.1) / 360);
    }
    while (ev3_color_sensor_get_reflect(color_3) > 20) {
    }
    ev3_motor_steer(left_motor,right_motor,0,0);
    pos.street = RED_STREET;
}
void runRedStreet(){
    color_4_index = 0;
    a_motor_index = 0;
    d_motor_index = 0;
    wall_follow_with_tasks(128,3,2,3,0,0);
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    float wheelDistance = 0;
    ev3_motor_set_power(a_motor,50);
    tslp_tsk(800);
    ev3_motor_set_power(a_motor,0);
    ev3_motor_steer(left_motor, right_motor, 10, 5);
    while (ev3_color_sensor_get_reflect(color_3) > 20) {
    }
    ev3_motor_steer(left_motor,right_motor,-30,0);
    tslp_tsk(300);
    ev3_motor_steer(left_motor,right_motor,0,0);
    ev3_motor_rotate(a_motor,200,-50,true);
    ev3_motor_steer(left_motor,right_motor,-15,70);
    ev3_motor_rotate(a_motor,100,-50,false);
    tslp_tsk(1075);
    ev3_motor_steer(left_motor,right_motor,0,0);
    ev3_motor_set_power(a_motor,-50);
    tslp_tsk(700);
    ev3_motor_set_power(a_motor,0);
    ev3_motor_steer(left_motor,right_motor,-30,0);
    tslp_tsk(402);
    ev3_motor_steer(left_motor,right_motor,0,0);
    wall_follow_with_tasks(60,0,0,1,0,0);
    ev3_motor_steer(left_motor, right_motor, 10, 5);
    while (ev3_color_sensor_get_reflect(color_3) > 20) {
    }
    ev3_motor_steer(left_motor,right_motor,30,0);
    tslp_tsk(450);
    ev3_motor_steer(left_motor,right_motor,0,0);
    ev3_motor_steer(left_motor,right_motor,30,-45);
    tslp_tsk(700);
    ev3_motor_steer(left_motor,right_motor,0,0);
    ev3_motor_steer(left_motor, right_motor, 15, 5);
    while (ev3_color_sensor_get_reflect(color_3) > 20) {
    }
    ev3_motor_steer(left_motor,right_motor,0,0);
    pos.street = YELLOW_STREET;
}

void readCode() {
    // define variables
    int bit1 = -1;
    int bit2 = -1;

    // leave start
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    ev3_motor_steer(left_motor, right_motor, 30, 2);
    while (abs(((ev3_motor_get_counts(EV3_PORT_B) + ev3_motor_get_counts(EV3_PORT_C)) / 2)) < 250) {
        display_sensors();
    }

    // detect line
    ev3_motor_steer(left_motor, right_motor, 10, 1);
    while (rgb4.b > 40) {
        display_sensors();
    }
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    if (rgb4.g < 20) {
        pos.street = RED_STREET;
        ev3_speaker_play_tone(NOTE_G4, 40);
    } else {
        pos.street = YELLOW_STREET;
        ev3_speaker_play_tone(NOTE_G5, 40);
    }
    tslp_tsk(50);
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    ev3_motor_steer(left_motor, right_motor, -10, 0);
    while (((abs(ev3_motor_get_counts(EV3_PORT_B)) + abs(ev3_motor_get_counts(EV3_PORT_C))) / 2) < 15) {
        display_sensors();
    }
    ev3_motor_steer(left_motor, right_motor, 0, 0);

    tslp_tsk(50);
    // record instructions
    ev3_motor_reset_counts(EV3_PORT_B);
    ev3_motor_reset_counts(EV3_PORT_C);
    ev3_motor_steer(left_motor, right_motor, 10, 1);
    int i;
    for (i = 1; i < 5; i++) {
        // read instructions
        bit1 = 0;
        bit2 = 0;
        while (abs(((ev3_motor_get_counts(EV3_PORT_B) + ev3_motor_get_counts(EV3_PORT_C)) / 2)) < (i * 58)) {
            display_sensors();
        }
        if (((rgb4.r + rgb4.g + rgb4.b) / 3) > 40) {
            bit1 = 1;
            ev3_speaker_play_tone(NOTE_C5, 50);
        } else {
            bit1 = 0;
            ev3_speaker_play_tone(NOTE_C4, 50);
        }
        while (abs(((ev3_motor_get_counts(EV3_PORT_B) + ev3_motor_get_counts(EV3_PORT_C)) / 2)) < ((i + 1) * 58)) {
            display_sensors();
        }
        if (((rgb4.r + rgb4.g + rgb4.b) / 3) > 40) {
            bit2 = 1;
            ev3_speaker_play_tone(NOTE_C5, 50);
        } else {
            bit2 = 0;
            ev3_speaker_play_tone(NOTE_C4, 50);
        }

        // decode instructions
        if (bit1 == 1) {
            if (bit2 == 1) {
                ev3_speaker_play_tone(NOTE_G6, -1);
            } else {
                tasks[i][0] = BLACKMATERIAL;
            }
        } else {
            if (bit2 == 1) {
                tasks[i][0] = BLUEMATERIAL;
            } else {
                tasks[i][0] = COLLECTSNOW;
            }
        }
    }

    // detect line
    ev3_motor_steer(left_motor, right_motor, 10, 1);
    while (ev3_color_sensor_get_reflect(color_3) > 20) {
        display_sensors();
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

void readColorCode(){
    float wheelDistance = 0;
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    float values[8] = {0,0,0,0,0,0,0,0};
    int isReading = 0;
    int i = 0;
    char lcdstr[100];
    while(wheelDistance < 21){
        ev3_motor_steer(left_motor,right_motor,25,5);
        wheelDistance = (ev3_motor_get_counts(left_motor) / 2 + ev3_motor_get_counts(right_motor) / 2) * ((3.1415926535 * 8.1) / 360);
    }
    ev3_motor_steer(left_motor, right_motor, 7, 5);
    colorid_t color3color = 6;
    while(color3color == 6){
        color3color = ev3_color_sensor_get_color(color_3);
        sprintf(lcdstr, "Color: %d", color3color);
	    ev3_lcd_draw_string(lcdstr, 0, 45);
        tslp_tsk(10);
    }
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    int x = 0;
    while(x < 50){
        color3color = ev3_color_sensor_get_color(color_3);
        sprintf(lcdstr, "Color: %d", color3color);
        ev3_lcd_draw_string(lcdstr, 0, 45);
        x += 1;
        tslp_tsk(10);
    }
    if(color3color == 5){
        pos.street = RED_STREET;
    }
    if(color3color == 4){
        pos.street = YELLOW_STREET;
    }
    sprintf(lcdstr, "%f3", wheelDistance);
    ev3_lcd_draw_string(lcdstr, 0, 105);
    ev3_motor_steer(left_motor, right_motor, 10, 5);
    while(wheelDistance < 58){
        wheelDistance = (ev3_motor_get_counts(left_motor) / 2 + ev3_motor_get_counts(right_motor) / 2) * ((3.1415926535 * 8.1) / 360);
        bool_t val = ht_nxt_color_sensor_measure_rgb(color_4,  &rgb4);
        assert(val);
        if(rgb4.r > 55 && isReading < 2){
            isReading = 50;
            i = round((wheelDistance - 26) / 4);
            values[i] = 1;
            ev3_speaker_play_tone(NOTE_C4,50);
        }
        else if(rgb4.g > 55 && isReading < 2){
            isReading = 50;
            i = round((wheelDistance - 26) / 4);
            values[i] = 1;
            ev3_speaker_play_tone(NOTE_C4,50);
        }
        else if(rgb4.b > 55 && isReading < 2){
            isReading = 50;
            i = round((wheelDistance - 26) / 4);
            values[i] = 1;
            ev3_speaker_play_tone(NOTE_C4,50);
        }
        else if(isReading > 1){
            isReading = isReading - 1;
        }
        tslp_tsk(10);
    }
    for(int i = 0; i < 7; i += 2){
        if(values[i] == 0){
            if(values[i + 1] == 0){
                tasks[i/2][0] = 0;
            }
            else{
                tasks[i/2][0] = 1;
            }
        }
        else{
            if(values[i + 1] == 0){
                tasks[i/2][0] = 2;
            }
            else{
            }
        }
    }
    pos.dash = 0;
    ev3_motor_steer(left_motor, right_motor, 10, 5);
    while (ev3_color_sensor_get_reflect(color_3) > 20) {
    }
    sprintf(lcdstr, "         %d", tasks[0][0]);
    ev3_lcd_draw_string(lcdstr, 0, 15);
    sprintf(lcdstr, "         %d", tasks[1][0]);
    ev3_lcd_draw_string(lcdstr, 0, 15 * 2);
    sprintf(lcdstr, "         %d", tasks[2][0]);
    ev3_lcd_draw_string(lcdstr, 0, 15 * 3);
    sprintf(lcdstr, "         %d", tasks[3][0]);
    ev3_lcd_draw_string(lcdstr, 0, 15 * 4);
    ev3_motor_steer(left_motor, right_motor, 0, 0);
}

/**
 * \brief follows a solid line using a PID and does tasks
 * \param distance Distance in cm
*/
void linePID_with_tasks(int distance, int doCar){
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    ev3_motor_reset_counts(a_motor);
    ev3_motor_reset_counts(d_motor);
    float wheelDistance = ev3_motor_get_counts(left_motor) / 2 + ev3_motor_get_counts(right_motor) / 2;
    float lasterror = 0, integral = 0;
    while (wheelDistance < distance) {
        execute_tasks(wheelDistance, doCar);
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
        float steer = 0.035 * error + 0.25 * integral + 4 * (error - lasterror);
        ev3_motor_steer(left_motor, right_motor, 30, steer);
        lasterror = error;  
        display_sensors();
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
*/
void color4PID(int distance,int tasksNumA,int tasksNumD){
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    ev3_motor_reset_counts(a_motor);
    ev3_motor_reset_counts(d_motor);
    int isTurningA = 0;
    int isTurningD = 0;
    char lcdstr[100];
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
        if(wheelDistance > next_a_motor_task[0] && tasksLeftA > 0 && isTurningA == 0){
            ev3_motor_rotate(a_motor,next_a_motor_task[2],0,false);
            isTurningA = 1;
        }
        if(wheelDistance > next_a_motor_task[1] && tasksLeftA > 0 && isTurningA == 1){
            ev3_motor_set_power(a_motor,-80);
            a_motor_index += 1;
            for(int i = 0;i < 3;i++){
                next_a_motor_task[i] = allTasks[pos.street][1][a_motor_index][i];
            }
            isTurningA = 0;
            tasksLeftA -= 1;
        }
        if(wheelDistance > next_d_motor_task[0] && tasksLeftD > 0 && isTurningD == 0){
            ev3_motor_rotate(d_motor,next_d_motor_task[2],80,false);
            isTurningD = 1;
        }
        if(wheelDistance > next_d_motor_task[1] && tasksLeftD > 0 && isTurningD == 1){
            ev3_motor_set_power(d_motor,-80);
            d_motor_index += 1;
            for(int i = 0;i < 3;i++){
                next_d_motor_task[i] = allTasks[pos.street][2][d_motor_index][i];
            }
            isTurningD = 0;
            tasksLeftD -= 1;
        }
        wheelDistance = (ev3_motor_get_counts(left_motor) / 2 + ev3_motor_get_counts(right_motor) / 2) * ((3.1415926535 * 8.1) / 360);
        bool_t val = ht_nxt_color_sensor_measure_rgb(color_4,  &rgb4);
        assert(val);
        float error = (rgb4.r + rgb4.g + rgb4.b) / 3 - 30;
        integral = error + integral * 0.5;
        float steer = 0.7 * error + 0 * integral + 0 * (error - lasterror);
        if(steer > 30){
            steer = 30;
        }
        ev3_motor_steer(left_motor, right_motor, 20, steer);
        lasterror = error;
    }
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    return;
}

/**
 * \brief follows a wall with a wall follower and does tasks
 * \param distance Distance in cm
 * \param steer Steer amount, ranging from 0 to 100
 * \param tasksNum4 amount of tasks for Color_4
 * \param tasksNumA amount of tasks for A_Motor
 * \param tasksNumD amount of tasks for D_Motor
 * \param doCar
*/
void wall_follow_with_tasks(int distance,int steer,int tasksNum4,int tasksNumA,int tasksNumD, int doCar, int speed){
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    ev3_motor_reset_counts(a_motor);
    ev3_motor_reset_counts(d_motor);
    int lastDash = 0;
    int isTurningA = 0;
    int a_motorStopped = 0;
    int isTurningD = 0;
    char lcdstr[100];
    float wheelDistance = -100;
    int tasksLeft4 = tasksNum4;
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
            ev3_speaker_play_tone(NOTE_A5,60);
            a_motorStopped = 1;
        }
        if(wheelDistance > next_color_4_task[0] && tasksLeft4 > 0){
            ev3_speaker_play_tone(NOTE_C4,60);
            tasksNum4 -= 1;
            bool_t val = ht_nxt_color_sensor_measure_rgb(color_4,  &rgb4);
            assert(val);
            if(rgb4.g < 40 && rgb4.r < 40 && rgb4.b < 40){
                ev3_speaker_play_tone(NOTE_C5,60);
                carDetected[pos.street] = color_4_index + 1;
            }
            sprintf(lcdstr, "%d,  %d,  %d,  ", rgb4.r, rgb4.g, rgb4.b);
            ev3_lcd_draw_string(lcdstr, 0, 45);
            color_4_index += 1;
            for(int i = 0;i < 3;i++){
                next_color_4_task[i] = allTasks[pos.street][0][color_4_index][i];
            }
        }
        if(wheelDistance > next_a_motor_task[0] && tasksLeftA > 0 && isTurningA == 0){
            ev3_motor_reset_counts(a_motor);
            ev3_speaker_play_tone(NOTE_A4,60);
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
        if(wheelDistance > carTasks[pos.street][0] && doCar && isTurningA == 0){
            ev3_motor_rotate(a_motor,carTasks[pos.street][2],80,false);
            isTurningA = 1;
        }
        if(wheelDistance > carTasks[pos.street][1] && doCar && isTurningA == 1){
            ev3_motor_set_power(a_motor,-80);
            isTurningA = 0;
        }
        if(wheelDistance > next_d_motor_task[0] && tasksLeft4 > 0 && isTurningD == 0 && back_loaded){
            ev3_motor_rotate(d_motor,next_d_motor_task[2],80,false);
            isTurningD = 1;
        }
        if(wheelDistance > next_d_motor_task[1] && tasksLeft4 > 0 && isTurningD == 1 && back_loaded){
            ev3_motor_set_power(d_motor,-100);
            d_motor_index += 1;
            for(int i = 0;i < 3;i++){
                next_d_motor_task[i] = allTasks[pos.street][2][d_motor_index][i];
            }
            isTurningD = 0;
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
        //sprintf(lcdstr, "%d,   %d", carDetected[3],ev3_motor_get_counts(a_motor));
        //ev3_lcd_draw_string(lcdstr, 0, 45);
    }
    ev3_motor_steer(left_motor, right_motor, 0, 0);
    return;
}

/**
 * \brief executes tasks
 * \param distance Distance of robot travelled (cm)
 * \param doCar Do we do car collection or not?
*/
void execute_tasks(float distance, int doCar) {
    display_sensors();

    //declare/define variables
    int a_degrees;
    int d_degrees;

    //check if motors are moving
    if (abs(ev3_motor_get_power(a_motor)) == 0) {
        a_turning = 0;
    }
    if (abs(ev3_motor_get_power(d_motor)) == 0) {
        d_turning = 0;
    }

    //check for a_motor task, execute task if task is to collect snow and it is time
    a_degrees = allTasks[pos.street][A_MOTOR][a_motor_index][2];
    if (distance > allTasks[pos.street][A_MOTOR][a_motor_index][0] && a_turning == 0 && tasks[pos.street][0] == COLLECTSNOW) {
        //execute part 1 of task
        ev3_motor_rotate(a_motor, a_degrees, 80, false);
        a_turning = 1;
        if (distance > allTasks[pos.street][A_MOTOR][a_motor_index][1]) {
            //execute part 2 of task
            ev3_motor_rotate(a_motor, -a_degrees, 80, false);
            a_turning = 1;
            a_motor_index += 1;
        }
    }

    //check for d_motor task, execute task if task is to dispense material and back is loaded and it is time and it is the correct material
    d_degrees = allTasks[pos.street][D_MOTOR][d_motor_index][1];
    if (distance > allTasks[pos.street][D_MOTOR][d_motor_index][0] && d_turning == 0 && tasks[pos.street][0] == back_loaded) {
        //execute part 1 of task
        ev3_motor_rotate(d_motor, d_degrees, 100, false);
        d_turning = 1;
        if (d_turning == 0) {
            //execute part 2 of task
            ev3_motor_rotate(d_motor, -d_degrees, 100, false);
            d_turning = 1;
            d_motor_index += 1;
        }
    }

    //check for color_4 task, execute if it is time
    if (distance > allTasks[pos.street][COLOR_4][color_4_index][0]) {
        // TODO: check for cars
    }

    //check for car_motor task, execute if doCar is true and it is time
    if (distance > carArray[pos.street][car_motor_index][0] && doCar) {
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////COLLECT A CAR
    }
}

void init() {
    // Register button handlers
    ev3_button_set_on_clicked(BACK_BUTTON, button_clicked_handler, BACK_BUTTON);
    
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
    //ev3_motor_set_power(a_motor, -100);
    ev3_motor_set_power(d_motor, 100);
    tslp_tsk(1500);
    //ev3_motor_stop(a_motor, false);
    ev3_motor_stop(d_motor, false);

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
}

void display_sensors() {
    // declare variables
    char msg[100];
    int value;

    // wait for values to be refreshed
    tslp_tsk(3);

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

static void button_clicked_handler(intptr_t button) {
    switch(button) {
    case BACK_BUTTON:
            ev3_lcd_fill_rect(0, 0, 178, 128, EV3_LCD_WHITE);
            ev3_lcd_draw_string("Stopping Program", 10, 60);
            ev3_motor_stop(left_motor, false);
            ev3_motor_stop(right_motor, false);
            ev3_motor_stop(a_motor, false);
            ev3_motor_stop(d_motor, false);
            ev3_lcd_draw_string("Program  Stopped", 10, 60);
        exit(0);
        break;
    }
}
