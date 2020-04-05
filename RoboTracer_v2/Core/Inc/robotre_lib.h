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
void updata_enc_cnt(int *, int *, int *, int *, int32_t *, int32_t *, int *, int *);
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
void speed_ctrl(char, float, float, float, float *);
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
void updata_robot_speed_distance(char);
void store_imu_data(char, short *);
void store_log_data(char);
void store_pot_data(char, short *);
float getRadius_imu(void);
void lowpass_filter(float *, float *, short, float, float, float);
float maximam(float *, short );
void create_speed_table(void);
void create_speed_table_2(void);
void create_speed_table_func(void);
void create_speed_table_func_2(void);
void mesurment_reset(void);

char encorder_correction(short, int);
char encorder_correction_distance(char);

short course_memory_const_distance(char enable);
void updata_imu_data_lowpassed(void);
float lowpass_filter_simple(float, float, float);

void calc_feed_forward(float, float, float, float *, float *);

void updata_now_speed(float *, float *);

float velocity_func(float);
void fix_acceleration(void);
void fix_acceleration_2(void);
short side_sensor_memory(char);

float getAngle();
void angle_measurement(char);


void RotationSpeed(char, float *);
void TranslationSpeed(char, float *);
void BanquetArt(char);	//	宴会芸
float getTargetOmega();

void main_GetSelfLocation(double *, double *, double *);
void main_GetMeaPosition(double *, double *, double *);
void main_GetTruePosition(double *, double *, double *);

char CalcIMUoffset(char);
void GetPosition(double *, double *, double *, char enable);
void GetDR_Position(double *, double *, char enable);

#endif

