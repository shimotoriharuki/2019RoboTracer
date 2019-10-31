/*
 * robotre_lib.h
 *
 *  Created on: Oct 24, 2019
 *      Author: robot
 */


#ifndef FUNCTIONS_H
#define FUNCTIONS_H

void LED(char);
void full_color(void);
char SW(char);
char R_SW(void);
int getEncorder_L(void);
int getEncorder_R(void);
void enc_reset(void);
void updata_enc_cnt(int *, int *, int *, int *, short *, short *);
char debug_lcd(void);
void updata_ADval(void);
char vcm_kill(char, float);
void maxon_ctrl(float, float);
void vcm_ctrl(float, float *);

void choice_following_mode(char);
void sensor_following(char);
void angle_ctrl(char, short);

void updata_curve_val(char, short, float, float *, float *);
void updata_straight_val(char, float, float *, float *, float, float, float);
void Line_trace(char);
void speed_ctrl(float, float, float, float *);
void trace(char, float, float, float, float *);
void debug_pcprintf(void);
void init(void);
char off_line_check(char);
char error_check(char *);
void hand_push_trace(char);
void increment_mytimer(void);
char getDigital(char);
char running_processing(void);
void flag_reset(void);
void flag_set(void);
void motor_reset(void);
float speed_select(void);
void error_reset(void);

char measurement_19mm(int, int, char);	//角度固定時
char measurement_19mm_2(int, int, char);	//スタート・ゴールマーカ検出時
char measurement_19mm_3(int, int, char);	//サイドマーカー検出時
char measurement_100mm(int, int, char);

char goal_area_processing(char);

void accelerator(char, float);

float getRadius(void);
short course_memory(char);
void updata_robot_speed(char);
void store_imu_data(char);
void store_imu_data_2(char);
void store_pot_data(char, short *);
float getRadius_imu(void);
void lowpass_filter(int *, float *, short, float, float, float);
float maximam(float *, short );
void create_speed_table(void);
void create_speed_table_2(void);
void mesurment_reset(void);

char encorder_correction(short, int);

short course_memory_const_distance(char enable);
void updata_imu_data_lowpassed(void);
float lowpass_filter_simple(float, float, float);



#endif

