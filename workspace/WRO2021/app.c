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
void run2021();
void init();
void display_sensors();
static void button_clicked_handler(intptr_t button);

//declare global variables
rgb_raw_t rgb1;
rgb_raw_t rgb4;

int purple = PURPLE;

void main_task(intptr_t unused) {
    init();
    run2021();
}

void run2021() {
    display_sensors();
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
    }
}



//     //////////          /////    //     /////   ///////   //////   //    //          //////////     //
//     //////////          //   /   //    //         //      //  //   ///   //          //////////     //
/////////////////          /////    //     /////     //      //  //   // // //          /////////////////
//     //////////          //       //         //    //      //  //   //   ///          //////////     //
//     //////////          //       //     /////     //      //////   //    //          //////////     //