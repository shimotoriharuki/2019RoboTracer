/*
 * robotre_lib.c
 *
 *  Created on: Oct 24, 2019
 *      Author: robot
 */
#include <stddef.h>

#include <stdio.h>
#include "main.h"

#include "Macros.h"
#include "G_variable.h"

#include "AQM0802.h"
#include "math.h"
#include "ICM_20648.h"

#include "fatfs.h"
#include "HAL_SDcard_lib.h"

#include "robotre_lib.h"
#include <string.h>

#include "rt_nonfinite.h"
#include "GetSelfLocation.h"
#include "GetSelfLocation_terminate.h"
#include "GetSelfLocation_initialize.h"

#include "PathFollowing.h"             /* Model's header file */
#include "rtwtypes.h"

static void argInit_1x2_real_T(double result[2]);
static void argInit_3x1_real_T(double result[3]);
static void argInit_3x3_real_T(double result[9]);
static double argInit_real_T(void);

void rt_OneStep(void);

/*
#define BUFF_SIZE 128

FATFS myFatFS;  // file system
FIL myFile; // File
//FILINFO fno;
//FRESULT fresult;  // result
UINT br, bw;  // File read/write count
UINT myBytes;

char buffer[BUFF_SIZE];

char myFileName[] = "TEST1.txt";
char myData[] = "OK\r\n";
#define DATA_SIZE_ 10000
short data[DATA_SIZE_];
short temp[DATA_SIZE_];
*/

short TargetCnt = 0;

//************************************************************************/
//* 役割　：　LCDでデバッグする関数
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
char debug_lcd(){
	signed char ready = 0;
	static short tim = 0;

	if(timer.lcd > LCD_WAIT){
		timer.lcd = 0;
		//--------------------------------------------------------
		switch(R_SW()){
			case 0:

				lcd_clear();
				lcd_locate(0,0);
				lcd_printf("%4d%4d", Line2, Line3);
				lcd_locate(0,1);
				lcd_printf("%4d%4d", Line1, Line4);
				LED('B');

				if(SW(1)){	//m系列信号
					LED('R');
					HAL_Delay(300);

					flag.angle_enable = 1;
					flag.log_store = 1;

					for(int16_t i = 0; i < M_LEN; i++){
						int16_t pulsewidth = 300;
						int16_t in = pulsewidth * msig[i];
						check_in = in;
						maxon_ctrl(in, in);
						HAL_Delay(350);
					}

					LED('B');

					//HAL_Delay(3000);

					flag.angle_enable = 0;
					flag.log_store = 0;

					//vcm_ctrl(0, &monitoring_vcm_pulse_width);
					maxon_ctrl(0, 0);

					//sd_write_array("Angle log", "Angle.txt", ANGLE_ARRAY_DATA, angle_array, OVER_WRITE);
					//sd_write_array("Encorder log", "left_M.txt", MEMORY_ARRAY_SIZE_2, various_memory1, OVER_WRITE);
					sd_write_array("Translation Log", "output.txt", MEMORY_ARRAY_SIZE_2, various_memory1, OVER_WRITE);
					sd_write_array("Translation Log", "input.txt", MEMORY_ARRAY_SIZE_2, various_memory2, OVER_WRITE);
				}
				else if(SW(2)){	//	同定後チェック
					LED('R');
					HAL_Delay(300);
					flag.art = 1;
					TargetOmega = 0;
				}

				else if(SW(3)){	//	プリ同定
					LED('R');
					HAL_Delay(300);

					flag.log_store = 1;

					maxon_ctrl(300, 300);

					HAL_Delay(4000);

					flag.log_store = 0;

					maxon_ctrl(0, 0);

					sd_write_array("Translation Log", "pre_velo.txt", MEMORY_ARRAY_SIZE_2, various_memory1, OVER_WRITE);
				}



			break;

			//--------------------------------------------------------
			case 2:
				if(SW(1)){
					LED('R');
					HAL_Delay(300);

					flag.angle_enable = 1;
					flag.log_store = 1;
					//vcm_ctrl(200, &monitoring_vcm_pulse_width);	//1200

					for(int16_t i = 0; i < M_LEN; i++){
						int16_t pulsewidth = 500;
						int16_t in = pulsewidth * msig[i];
						check_in = in;
						maxon_ctrl(in, -in);
						HAL_Delay(200);

					}

					LED('B');


					//HAL_Delay(3000);

					flag.angle_enable = 0;
					flag.log_store = 0;

					//vcm_ctrl(0, &monitoring_vcm_pulse_width);
					maxon_ctrl(0, 0);

					//sd_write_array("Angle log", "Angle.txt", ANGLE_ARRAY_DATA, angle_array, OVER_WRITE);
					//sd_write_array("Encorder log", "left_M.txt", MEMORY_ARRAY_SIZE_2, various_memory1, OVER_WRITE);
					sd_write_array("Identification Log", "output.txt", MEMORY_ARRAY_SIZE_2, various_memory1, OVER_WRITE);
					sd_write_array("Identification Log", "input.txt", MEMORY_ARRAY_SIZE_2, various_memory2, OVER_WRITE);
				}
				else if(SW(2)){
					LED('R');
					HAL_Delay(300);
					flag.art = 1;
					flag.log_store = 1;
					TargetOmega = 15;
					HAL_Delay(1000);

					TargetOmega = 20;
					HAL_Delay(1000);

					TargetOmega = 10;
					HAL_Delay(1000);

					flag.art = 0;
					flag.log_store = 0;
					TargetOmega = 0;
					maxon_ctrl(0, 0);
					sd_write_array("Identification Log", "Omega152010_H.txt", MEMORY_ARRAY_SIZE_2, various_memory1, OVER_WRITE);
				}
				else if(SW(3)){
					LED('R');
					HAL_Delay(300);

					flag.log_store = 1;

					maxon_ctrl(500, -500);

					HAL_Delay(3000);

					flag.log_store = 0;

					maxon_ctrl(0, 0);

					sd_write_array("Identification Log", "Omega.txt", MEMORY_ARRAY_SIZE_2, various_memory1, OVER_WRITE);
				}



				LED('G');
			break;

			//--------------------------------------------------------
			case 4:

				lcd_clear();
				lcd_locate(0,0);
				lcd_printf("%lf", TargetVelo[0]);
				lcd_locate(0,1);
				lcd_printf("%lf", TargetVelo[1]);



				if(SW(1)){
					LED('R');
					TargetVelo[0] = 1;
					TargetVelo[1] = 0.1;
					flag.GSL_enable = 1;
					HAL_Delay(5000);

					LED('B');
					sd_write_array("EstimationLocation", "x.txt", MEMORY_ARRAY_SIZE_2, various_memory1, OVER_WRITE);
					sd_write_array("EstimationLocation", "y.txt", MEMORY_ARRAY_SIZE_2, various_memory2, OVER_WRITE);
					sd_write_array("EstimationLocation", "th.txt", MEMORY_ARRAY_SIZE_2, various_memory3, OVER_WRITE);

					flag.GSL_enable = 0;
				}
				else{
					flag.GSL_enable = 0;
					TargetVelo[0] = 0;
					TargetVelo[1] = 0;
				}

				LED('Y');
			break;

			//--------------------------------------------------------
			case 6:
				lcd_clear();
				lcd_locate(0,0);
				lcd_printf("Go Ready");
				lcd_locate(0,1);
				lcd_printf("SW3");

				if(SW(3))	ready = 1;
				LED('R');


			break;

		}
	}
	return ready;

}

/************************************************************************/
//* 役割　：　pc,printfデバッグ
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
void debug_pcprintf(){
	//printf("\r\n");	//

	//printf("EncL = %d, EncR = %d  L = %d R = %d\r\n", total_encL, total_encR, getEncorder_L(), getEncorder_R());

	//printf("L0 = %d L1 = %d L2 = %d L3 = %d O0 = %d O1 = %d O2 = %d\r\n", ad2[0], ad2[1], ad2[2], ad2[3], ad1[0], ad1[1], ad1[2]);
	//printf("%4d %4d %4d %4d %4d %4d %4d\r\n", Line1, Line2, Line3, Line4, SideL, SideR, Def_ref);
	//printf("imu : %f\r\n",xg_l);

}

//************************************************************************/
//* 役割　：　走行中処理
//* 引数　：　void:
//* 戻り値：　char: 走行が終わったら1 それ以外0
//* 備考 : なし
//************************************************************************/
char running_processing(){
	char ret = 0;
	static char start_goal_cnt = 0;
	static short side_line_cnt = 0;
	static char cross_line_ignore = 0;
	static char start_accele_finish = 0;
	//static unsigned short ref_pot = 0;
	static char side_off_cnt = 0;
	//static char a = 0, b = 0;

	int32_t ave_total = 0;
	static int32_t pre_ave_total = 0;

	static short straight_cnt = 0;
	char correction = 0;

	//robot_speed = target_robot_speed;
	/*-----------------	リセット-------------------*/
	if(flag.runnning_reset == 1){
		start_goal_cnt = 0;
		read_startgoal_line = 1;
		read_side_line = 1;
		start_accele_finish = 0;


		//increment_acc = CONTROL_CYCLE * START_ACCELERATION;

		flag.runnning_reset = 0;
		flag.start_line_flag = 0;
		LED('N');
	}

	/*-----------------	通常加速度に更新-------------------*/
	/*
	if(robot_speed >= target_robot_speed && start_accele_finish == 0){
		increment_acc = CONTROL_CYCLE * NORMAL_ACCELERATION;
		start_accele_finish = 1;
		//LED('R');
	}
	*/
	/*---------------	スタートマーカ読んだらいろいろリセット-------------------*/
	if(start_goal_cnt == 1 && flag.enc_memory_reset == 1){	//エンコーダ初期化
		total_encL_memory = total_encR_memory = total_encL = total_encR = total_encL_distance = total_encR_distance = total_encL_reset = total_encR_reset = 0;
		pre_ave_total = 0;
		flag.enc_memory_reset = 0;
		yaw_angle = 0;
		flag.start_line_flag = 1;
		timer.correction = 0;

		/*-----------------	再生走行時リセット(2走目以降)-------------------*/
		if(flag.speed_updata == 1){
			flag.speed_updata = 0;
			data_access_dis = 0;	//		記録走行の配列に使う
			flag.updata_robot_speed_reset = 1;

			flag.course_memory_distance = 0;
			flag.record_running_enable = 1;
			flag.after_finish_sd_wtire = 0;
		}
	}

	/*-----------------	一定距離で処理-------------------*/
	ave_total = (total_encL_distance + total_encR_distance) / 2;	//累計値

	//if((ave_total) >= COUNT_TO_RECORD && flag.start_line_flag == 1){
	if((ave_total - pre_ave_total) >= COUNT_TO_RECORD && flag.start_line_flag == 1){
		course_memory_const_distance(flag.course_memory_distance);
		updata_robot_speed_distance(flag.record_running_enable);

		pre_ave_total = ave_total;
		//timer.distance = 0;
	}

	if(timer.distance < 50){
		//LED('R');
	}
	else{
		//LED('N');
	}

	if(yaw_angle < 0.006 && yaw_angle > -0.006){
		straight_cnt++;
	}
	else{
		straight_cnt = 0;
	}

	if(straight_cnt > 1000){
		flag.yaw_zero = 1;
		//LED('R');
	}
	else{
		flag.yaw_zero = 0;
		//LED('N');
	}


	if(timer.correction <= 50){
		LED('G');
	}
	else{
		LED('N');
	}

	/*-----------------marker record-------------------*/
	//updata_robot_speed(flag.record_running_enable);

	/*----------------	クロスライン無視処理-------------------*/
	if(flag.angle == 1 && cross_line_ignore == 0){	//	ラインセンサーがクロスライン読んだら
		measurement_100mm(total_encL, total_encR, 0);	//100mm計測スタート
		cross_line_ignore = 1;
	}
	else if(cross_line_ignore == 1){
		if(measurement_100mm(total_encL, total_encR, 0)){
			cross_line_ignore = 0;
		}
		//LED('R');
	}
	else{
		//LED('W');
	}

	/*----------------	マーカー処理-------------------*/
	if(cross_line_ignore == 0){
		/*--------	スタート・ゴールマーカ---------*/
#ifndef REVERCE_RUN
		//	通常周回
		if(getDigital('R') == 1 && getDigital('L') == 0 && read_startgoal_line == 1){
			start_goal_cnt++;

			measurement_19mm_2(total_encL, total_encR, 0 );
			read_startgoal_line = 0;
		}
		else if(read_startgoal_line == 0){
			read_startgoal_line = measurement_19mm_2(total_encL, total_encR, 0);
			//LED('R');
		}
		else{
			//LED('N');
		}
		/*-------	サイドマーカー---------*/
		if(getDigital('R') == 0 && getDigital('L') == 1 && read_side_line == 1){
			side_line_cnt++;
			measurement_19mm_3(total_encL, total_encR, 0);
			read_side_line = 0;
			flag.side = 1;
			//max_access = course_memory(flag.course_memory);	//	エンコーダ値と半径を記録

			max_access = side_sensor_memory(flag.course_memory_distance);
			encorder_correction_distance(flag.record_running_enable);
		}
		else if(read_side_line == 0){
			if(getDigital('R') == 0 && getDigital('L') == 0){	//	マーカーが白から黒になったら
				side_off_cnt++;
			}
			else side_off_cnt = 0;

			//if(measurement_19mm_3(total_encL, total_encR) || side_off_cnt >= 2){
			if(side_off_cnt >= 2){
				read_side_line = 1;
				flag.side = 0;
			}

			//LED('R');
		}
		else{
			//LED('N');
		}

#else
		//	逆走周回
		if(getDigital('L') == 1 && getDigital('R') == 0 && read_startgoal_line == 1){
			start_goal_cnt++;

			measurement_19mm_2(total_encL, total_encR, 0 );
			read_startgoal_line = 0;
		}
		else if(read_startgoal_line == 0){
			read_startgoal_line = measurement_19mm_2(total_encL, total_encR, 0);
			//LED('R');
		}
		else{
			//LED('N');
		}
		/*-------	サイドマーカー---------*/
		if(getDigital('L') == 0 && getDigital('R') == 1 && read_side_line == 1){
			side_line_cnt++;
			measurement_19mm_3(total_encL, total_encR, 0);
			read_side_line = 0;
			flag.side = 1;
			//max_access = course_memory(flag.course_memory);	//	エンコーダ値と半径を記録

			max_access = side_sensor_memory(flag.course_memory_distance);
			encorder_correction_distance(flag.record_running_enable);
		}
		else if(read_side_line == 0){
			if(getDigital('L') == 0 && getDigital('R') == 0){	//	マーカーが白から黒になったら
				side_off_cnt++;
			}
			else side_off_cnt = 0;

			//if(measurement_19mm_3(total_encL, total_encR) || side_off_cnt >= 2){
			if(side_off_cnt >= 2){
				read_side_line = 1;
				flag.side = 0;
			}

			//LED('R');
		}
		else{
			//LED('N');
		}
#endif
	}

	if(flag.error)	LED('M');


	if(flag.record_running_enable == 1){
		gain.kp = 13.5;	//11, 12
		gain.ki = 10;
		gain.kd = 0.62;	//0.45, 0.6

	}
	else{
		gain.kp = 12;	//3.3
		gain.ki = 10;
		gain.kd = 0.63;	//0.075
	}

	/*//	速度を変えるやつ
	if(flag.record_running_enable == 1){
		gain.kp = 2.6;	//3.3
		gain.ki = 0;
		gain.kd = 0.08;	//0.075
	}
	else{
		gain.kp = 2.6;	//3.3
		gain.ki = 0;
		gain.kd = 0.08;	//0.075
	}
	*/

/*
	if(target_robot_speed == HIGH_SPEED_DISTANCE){	////float kp = 30, ki  = 100, kd = 0.5;	//beforer float float kp = 2.5, ki  = 20, kd = 0.04;
		gain.kp = 8;	//8
		gain.ki = 20;	//50
		gain.kd = 0.5;	//0,45
		//LED('R');
	}
	else if(target_robot_speed == COMM_H_SPEED_DISTANCE){
		gain.kp = 4.5;
		gain.ki = 10;
		gain.kd = 0.2;
		//LED('Y');
	}
	else if(target_robot_speed == LOW_SPEED_DISTANCE || target_robot_speed == COMM_L_SPEED_DISTANCE ){

		gain.kp = 4;	//3.7
		gain.ki = 0;
		gain.kd = 0.12;	//0.12

		/*when straight
		gain.kp = 3.8;	//4
		gain.ki = 10;
		gain.kd = 0.15;	//0.15
		*/
		/*//1450 to 2100
		gain.kp = 3.8;	//3.6
		gain.ki = 0;
		gain.kd = 0.13;	//0.1
		*/
		/*//3000
		gain.kp = 4.5;	//3.6
		gain.ki = 0;
		gain.kd = 0.2;	//0.1
		*/
		/*//タイヤすり減ってるとき
		gain.kp = 3.4;
		gain.ki = 0;
		gain.kd = 0.07;
		*/

		//LED('B');
//	}

	/*
	else if(target_robot_speed == HIGH_SPEED_2){
		gain.kp = 8;	//8
		gain.ki = 20;	//20
		gain.kd = 0.5;	//0.5
		//LED('R');
	}
	else if(target_robot_speed == COMM_SPEED_2){
		gain.kp = 3.6;	//3.6
		gain.ki = 10;	//10
		gain.kd = 0.1;	//0.1
		//LED('R');
	}else if(target_robot_speed == LOW_SPEED_2){
		gain.kp = 3.6;	//3.6
		gain.ki = 10;	//10
		gain.kd = 0.1;	//0.1
		//LED('R');
	}
	*/

	/*
	if(flag.side == 1)	LED('R');
	else LED('N');
	*/

	/*----------------ゴール判定-------------------*/

	if(start_goal_cnt >= 2)	ret = 1;
	//if((total_encL + total_encR) / 2 >= 95000)	ret = 1;
	//if(flag.log_store == 0) ret = 1;
	if(flag.error == 1)	ret = -1;

	return ret;
}

//************************************************************************/
//* 役割　：　ゴール後処理　一定距離進んで止まる
//* 引数　：　void:
//* 戻り値：　char: 処理がが終わったら1 それ以外0
//* 備考 : なし
//************************************************************************/
char goal_area_processing(char reset){
	char ret = 0;
	float ave_enc = 0;
	static char Flag = 0;
	static int keep_encL, keep_encR;


	if(Flag == 0){
		keep_encL = total_encL;
		keep_encR = total_encR;
		Flag = 1;
	}

	else{
		ave_enc = ((total_encL - keep_encL) + (total_encR - keep_encR)) / 2;
		if(ave_enc > MM500_PER_MPP){
			ret = 1;
			Flag = 0;
		}
	}

	if(reset == 1){
		keep_encL = 0;
		keep_encR = 0;
		Flag = 0;
	}

	return ret;

}

//************************************************************************/
//* 役割　：　タイマ割り込みコールバック関数 1ms
//* 引数　：　よくわからない
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){	//タイマー割り込みで呼び出される
//void TIM6_DAC1_IRQHandler(void){

	#define AVE_NUM 10
	static int sum_pot = 0;

	timer.check_01ms++;

	//sum_pot += (2048 - ad1[6]);

//	if(timer.check_01ms >= 9){
//		average_pot = sum_pot / AVE_NUM;
//		sum_pot = 0;

//		timer.check_01ms = 0;

		increment_mytimer();
		//accelerator(flag.acc, increment_acc);
	/
		if(timer.pot_timer >= 2){	//1msごと 1sぶん格納できる
			store_pot_data(flag.pot_store, &number_stored);
			timer.pot_timer = 0;
		}
	/
		flag.error = error_check(&error_number);

		updata_ADval();
		//updata_now_speed(&now_speed_L, &now_speed_R);

		read_gyro_data();
		read_accel_data();
		updata_imu_data_lowpassed();

		store_imu_data(flag.pot_store, &number_stored);

		/
		if(timer.log >= 100 && flag.sd_record == 1){
			float now_speed_L = MM_PER_PULS * 1000. * getEncorder_L();			//[mm/s]
			float now_speed_R = MM_PER_PULS * 1000. * getEncorder_R(); 		//[mm/s]
			float ave_speed = (now_speed_L + now_speed_R) / 2;

			sd_write(1, &ave_speed, ADD_WRITE);

			timer.log = 0;
		}
		/

		store_log_data(flag.log_store);

		choice_following_mode(flag.following_start);
		sensor_following(flag.following);
		angle_ctrl(flag.angle, 0);

		//calc_feed_forward(ff_accele_L, ff_accele_R, 0, &ff_duty_L, &ff_duty_R);
		Line_trace(flag.line_trace);

		//speed_ctrl(flag.speed_ctrl_enable, speed_L, speed_R, robot_speed, &speed_val);//普通は消す

		hand_push_trace(flag.hand_push);

		updata_enc_cnt(&total_encL, &total_encR, &total_encL_memory, &total_encR_memory, &total_encL_distance, &total_encR_distance);

		enc_reset();	//初期値にする
//	}
}
*/

//************************************************************************/
//* 役割　：　タイマ割り込みコールバック関数 0.1ms
//* 引数　：　よくわからない
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
/*
void TIM7_DAC2_IRQHandler(void){

	timer.check++;
}
*/
/************************************************************************/
//* 役割　：　イニシャライズ関数
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
void init(){
	HAL_TIM_Base_Start_IT(&htim6);	//	タイマ㿼割り込みスターヿ
	HAL_TIM_Base_Start_IT(&htim7);	//	タイマ㿼割り込みスターヿ
	lcd_init();			//起動に時間がかかる
	imu_check = IMU_init();
	sd_mount_check = sd_mount();
	//flag.imu_store_2 = 1;

	flag_reset();

	robot_speed = 0;
	Def_speed_L = Def_speed_R = speed_L = speed_R = robot_speed;

	increment_acc = CONTROL_CYCLE * START_ACCELERATION;
	deceleration_cnt = (1 / MM_PER_PULS) * DECELERATION_DISTANCE;
	continued_cnt =(1 / MM_PER_PULS) * CONTINUED_DISTANCE;
	flag.course_memory = 1;

	for(short i = 0; i < MEMORY_ARRAY_SIZE_DISTANCE; i++){
		imu_radius_memory_distance[i] = 0;
		speed_table_distance[i] = 0;
	}

	maxon_ctrl(0, 0);
	vcm_ctrl(0, &monitoring_vcm_pulse_width);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

	TIM4 -> CNT = ENCORDER_OFFSET;
	TIM5 -> CNT = ENCORDER_OFFSET;



	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);	//エンコーヿースターヿ
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);	//エンコーヿースターヿ
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);	//センサーFETON
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);	//pwm for sensor
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);	//pwm for Maxon1
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);	//pwm for Maxon2
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);	//pwm for VCM
	HAL_ADC_Start_DMA(&hadc1,  (uint32_t *)ad1, DATA_SIZE1);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);

	GetSelfLocation_initialize();
	PathFollowing_initialize();
}

//************************************************************************/
//* 役割　：　フラグを全部0にする
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
void flag_reset(){
	flag.following = 0;
	flag.hand_push = 0;
	flag.line_trace = 0;
	flag.error = 0;
	//flag.run = 0;
	flag.ready = 0;
	flag.angle = 0;
	flag.following_start = 0;
	flag.error_check_enable = 1;
	flag.acc = 0;
	//total_encL = total_encR = 0;
	flag.sd_record  = 0;
	flag.course_memory_distance = 0;
	flag.record_running_enable = 0;


	//SD
	//user_fclose();
}


//************************************************************************/
//* 役割　：　走行に必要なフラグをセットする
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
void flag_set(){
	flag.following_start = 1;
	flag.line_trace = 1;
	flag.curve = 1;
	flag.straight = 1;
	flag.curve_to_straight = 0;
	flag.acc = 1;
	flag.pot_store = 1;
	flag.trace = 1;
	flag.enc_memory_reset = 1;
	flag.speed_ctrl_enable = 1;
	flag.sd_record  = 1;
	flag.log_store = 1;
	flag.after_finish_sd_wtire = 1;


	//SD
	//user_fopen("running_log", "log.txt");

}

//************************************************************************/
//* 役割　：　モータを入力値を0にする
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
void motor_reset(){
	maxon_ctrl(0, 0);
	vcm_ctrl(0, &monitoring_vcm_pulse_width);
}

//************************************************************************/
//* 役割　：　ローパスフィルタ
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
void lowpass_filter(float in_data[], float out_data[], short size, float freq, float samplerate, float q){
	// フィルタ係数を計算する
	float omega = 2.0f * 3.14159265f * freq / samplerate;
	float alpha = sin(omega) / (2.0f * q);

	float a0 =  1.0f + alpha;
	float a1 = -2.0f * cos(omega);
	float a2 =  1.0f - alpha;
	float b0 = (1.0f - cos(omega)) / 2.0f;
	float b1 =  1.0f - cos(omega);
	float b2 = (1.0f - cos(omega)) / 2.0f;

	// フィルタ計算用のバッファ変数。
	float in1  = 0.0f;
	float in2  = 0.0f;
	float out1 = 0.0f;
	float out2 = 0.0f;

	// フィルタを適用
	for(int i = 0; i < size; i++)
	{
		// 入力信号にフィルタを適用し、出力信号として書き出す。
		out_data[i] = b0/a0 * in_data[i] + b1/a0 * in1  + b2/a0 * in2 - a1/a0 * out1 - a2/a0 * out2;

		in2  = in1;       // 2つ前の入力信号を更新
		in1  = in_data[i];  // 1つ前の入力信号を更新

		out2 = out1;      // 2つ前の出力信号を更新
		out1 = out_data[i]; // 1つ前の出力信号を更新
	}

}

//************************************************************************/
//* 役割　：　最大値を返す
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
float maximam(float in[], short size){
	float max = 0;

	for(short access = 0; access < size; access++){
		if(in[access] < 0) in[access] *= -1;

		if(in[access] > max) max = in[access];
	}

	return max;

}

//************************************************************************/
//* 役割　：　記憶したポテンショの値から速度を配列に打ち込む
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
void create_speed_table(){
	//char minus = 0;
	short temp = 0;
	//short i = 1;
	short access = 0;
	short speed_access = 0;

	for(access = 0; access < MEMORY_ARRAY_SIZE; access++){
		temp = radius_memory[access];
		if(temp < 0){
			temp *= -1;
		}

		if(temp < 8){	//6, 8
			speed_table[access] = HIGH_SPEED;
		}
		else if(temp < 40){	//25, 35
			speed_table[access] = COMM_SPEED;
		}
		else if(temp < 300){	//300, 300
			speed_table[access] = LOW_SPEED;
		}
		else{
			speed_table[access] = LOW_SPEED;
		}
	}
	/*
	//減速距離がたりない区間があったら修正する
	for(access = 0; access < MEMORY_ARRAY_SIZE; access++){
		if(speed_table[access] < speed_table[access - 1] && encorder_playback[access] - encorder_playback[access - 1] < deceleration_cnt){
			if(access - 2 >= 0){
				encorder_playback[access - 2] = encorder_playback[access - 1];

				for(short i = 0; i < MEMORY_ARRAY_SIZE - access; i++){
					speed_table[access + i - 1] = speed_table[access + i];
					encorder_playback[access + i - 1] = encorder_playback[access + i];
					radius_memory[access + i - 1] = radius_memory[access + i];
				}
				access--;
			}
			else{
				speed_table[access - 1] = LOW_SPEED;
			}
		}
	}
	*/
	/*
	access = 0;
	while(access < MEMORY_ARRAY_SIZE){
		if(speed_table[speed_access + 1] < speed_table[speed_access] && encorder_playback[access + 1] - encorder_playback[access] <= deceleration_cnt){
			speed_table[access + 1] = speed_table[access];
			LED('B');
			access++;
			//speed_access++;
		}
		else{
			access++;
			speed_access = access;
			LED('N');
		}
	}
	*/

}

//************************************************************************/
//* 役割　：　記憶したポテンショの値から速度を配列に打ち込む　速いバーション
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
void create_speed_table_2(){
	short temp = 0;
	short access = 0;
	short speed_access = 0;

	for(access = 0; access < MEMORY_ARRAY_SIZE; access++){
		temp = radius_memory[access];
		if(temp < 0){
			temp *= -1;
		}

		if(temp < 7){	//6, 8
			speed_table[access] = HIGH_SPEED_2;
		}
		else if(temp < 100){	//25, 35
			speed_table[access] = COMM_SPEED_2;
		}
		else if(temp < 300){	//300, 300
			speed_table[access] = LOW_SPEED_2;
		}
		else{
			speed_table[access] = LOW_SPEED_2;
		}
	}
	/*
	//減速距離がたりない区間があったら修正する
	for(access = 0; access < MEMORY_ARRAY_SIZE; access++){
		if(speed_table[access] < speed_table[access - 1] && encorder_playback[access] - encorder_playback[access - 1] < deceleration_cnt){
			if(access - 2 >= 0){
				encorder_playback[access - 2] = encorder_playback[access - 1];

				for(short i = 0; i < MEMORY_ARRAY_SIZE - access; i++){
					speed_table[access + i - 1] = speed_table[access + i];
					encorder_playback[access + i - 1] = encorder_playback[access + i];
					radius_memory[access + i - 1] = radius_memory[access + i];
				}
				access--;
			}
			else{
				speed_table[access - 1] = LOW_SPEED;
			}
		}
	}
	*/
	/*
	access = 0;
	while(access < MEMORY_ARRAY_SIZE){
		if(speed_table[speed_access + 1] < speed_table[speed_access] && encorder_playback[access + 1] - encorder_playback[access] <= deceleration_cnt){
			speed_table[access + 1] = speed_table[access];
			LED('B');
			access++;
			//speed_access++;
		}
		else{
			access++;
			speed_access = access;
			LED('N');
		}
	}
	*/

}

//************************************************************************/
//* 役割　：　記憶したポテンショの値から速度を配列に打ち込む　関数バージョン
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
void create_speed_table_func(){
	float temp = 0;
	int access = 0;

	for(access = 0; access < MEMORY_ARRAY_SIZE_DISTANCE; access++){
		temp = imu_radius_memory_distance[access];
		if(temp < 0){
			temp *= -1;
		}
		//speed_table_distance[access] = velocity_func(temp);

		if(temp < 250){
			speed_table_distance[access] = 1500;
		}
		else if(temp < 350){
			speed_table_distance[access] = 1500;
		}
		else if(temp < 500){
			speed_table_distance[access] = 1800;
		}
		else if(temp < 800){
			speed_table_distance[access] = 2000;
		}
		else if(temp < 1100){
			speed_table_distance[access] = 2200;
		}
		else if(temp < 1300){
			speed_table_distance[access] = 3000;
		}
		else if(temp < 1400){
			speed_table_distance[access] = 4500;
		}
		else if(temp < 2000){
			speed_table_distance[access] = 5000;
		}
		else if(temp < 3000){
			speed_table_distance[access] = 5000;
		}
		else if(temp < 5000){
			speed_table_distance[access] = 5000;
		}
		else{
			speed_table_distance[access] = 5000;
		}

		if(speed_table_distance[access] > 5000) speed_table_distance[access] = 5000;
		if(speed_table_distance[access] < 1500) speed_table_distance[access] = 1500;
/*
		if(temp < 250){	//走るやつ
			speed_table_distance[access] = 1300;
		}
		else if(temp < 350){
			speed_table_distance[access] = 1300;
		}
		else if(temp < 500){
			speed_table_distance[access] = 1300;
		}
		else if(temp < 800){
			speed_table_distance[access] = 1500;
		}
		else if(temp < 1100){
			speed_table_distance[access] = 1500;
		}
		else if(temp < 1300){
			speed_table_distance[access] = 2000;
		}
		else if(temp < 1400){
			speed_table_distance[access] = 4500;
		}
		else if(temp < 2000){
			speed_table_distance[access] = 5000;
		}
		else if(temp < 3000){
			speed_table_distance[access] = 5000;
		}
		else if(temp < 5000){
			speed_table_distance[access] = 5000;
		}
		else{
			speed_table_distance[access] = 5000;
		}

		if(speed_table_distance[access] > 5000) speed_table_distance[access] = 5000;
		if(speed_table_distance[access] < 1300) speed_table_distance[access] = 1300;
*/
	}

	sd_write_array("speed_plan", "original.txt", MEMORY_ARRAY_SIZE_DISTANCE, speed_table_distance, OVER_WRITE);

	fix_acceleration();

	sd_write_array("speed_plan", "fixed.txt", MEMORY_ARRAY_SIZE_DISTANCE, speed_table_distance, OVER_WRITE);
}


//************************************************************************/
//* 役割　：　記憶したポテンショの値から速度を配列に打ち込む　速いバージョン
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
void create_speed_table_func_2(){
	float temp = 0;
	int access = 0;

	for(access = 0; access < MEMORY_ARRAY_SIZE_DISTANCE; access++){
		temp = imu_radius_memory_distance[access];
		if(temp < 0){
			temp *= -1;
		}
		//speed_table_distance[access] = velocity_func(temp);
		 if(temp < 250){
			speed_table_distance[access] = 1600;
		}
		else if(temp < 350){
			speed_table_distance[access] = 1800;
		}
		else if(temp < 500){
			speed_table_distance[access] = 2500;
		}
		else if(temp < 700){
			speed_table_distance[access] = 2500;
		}
		else if(temp < 1000){
			speed_table_distance[access] = 3200;
		}
		else if(temp < 1250){
			speed_table_distance[access] = 6000;
		}
		else if(temp < 1400){
			speed_table_distance[access] = 6000;
		}
		else if(temp < 2000){
			speed_table_distance[access] = 6000;
		}
		else if(temp < 3000){
			speed_table_distance[access] = 6000;
		}
		else if(temp < 5000){
			speed_table_distance[access] = 6000;
		}
		else{
			speed_table_distance[access] = 6000;
		}

		if(speed_table_distance[access] > 6000) speed_table_distance[access] = 6000;
		if(speed_table_distance[access] < 1600) speed_table_distance[access] = 1600;

/*
		if(temp < 250){	//走るやつ「
			speed_table_distance[access] = 1450;
		}
		else if(temp < 350){
			speed_table_distance[access] = 1450;
		}
		else if(temp < 500){
			speed_table_distance[access] = 1500;
		}
		else if(temp < 700){
			speed_table_distance[access] = 1600;
		}
		else if(temp < 800){
			speed_table_distance[access] = 1800;
		}
		else if(temp < 1250){
			speed_table_distance[access] = 5000;
		}
		else if(temp < 1400){
			speed_table_distance[access] = 5000;
		}
		else if(temp < 2000){
			speed_table_distance[access] = 5000;
		}
		else if(temp < 3000){
			speed_table_distance[access] = 5000;
		}
		else if(temp < 5000){
			speed_table_distance[access] = 5000;
		}
		else{
			speed_table_distance[access] = 5000;
		}

		if(speed_table_distance[access] > 5000) speed_table_distance[access] = 5000;
		if(speed_table_distance[access] < 1450) speed_table_distance[access] = 1450;
*/
	}

	sd_write_array("speed_plan", "original.txt", MEMORY_ARRAY_SIZE_DISTANCE, speed_table_distance, OVER_WRITE);

	fix_acceleration_2();

	sd_write_array("speed_plan", "fixed.txt", MEMORY_ARRAY_SIZE_DISTANCE, speed_table_distance, OVER_WRITE);
}

//************************************************************************/
//* 役割　：　速度計画関数
//* 引数　：　float:
//* 戻り値：　float:
//* 備考 :
//************************************************************************/
float velocity_func(float x){

	//return  0.0000022123 * x * x * x + -0.00832175933646596 * x * x +  10.3811700493996 * x + 342.850707598025;
	//return  0.00000093892 * x * x * x + -0.00408630937142440 * x * x +  6.52370274430785 * x + 300.255968985736;
	//return  0.0000010467 * x * x * x + -0.00374431721535731 * x * x + 4.04645026468450 * x + 11.0242268447084;
	//return   0.000001095 * x * x * x + -0.00394201751103049 * x * x + 4.34751868943531 * x + 365.290656310387;
	//	いいやつreturn   0.00000025993 * x * x * x + -0.00097709 * x * x + 1.28403578560394 * x + 1268;
	//return  3.74545574335812e-08 * x * x * x + -0.000452431704301710 * x * x + 1.63713171672511 * x + 1419.18556658185;
	return -3.08141448955387e-12 * x * x * x *x + 6.83419689801159e-08 * x * x * x + -0.000526616973180803 * x * x + 1.71692148820843 * x + 964.630505334039;
	//return 0.000000958783462728946f * powf(x, 3) + (-0.00361845365313069f) * powf(x, 2) + 5.28713093017124f * x + 235.903467671659f;



}

//************************************************************************/
//* 役割　：　加速度修正
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
void fix_acceleration(){
	#define LENGTH 10 //[mm]
	int i = 0, j = 0;
	float vel_diff = 0;
	float time = 0;
	float now_acc = 0;
	short cnt = 0;
	float speed_temp = 0;
	char Flag = 0;

	speed_table_distance[0] = 1000.;

	for(i = 0; i < MEMORY_ARRAY_SIZE_DISTANCE - 1; i++){
		vel_diff = fabs(speed_table_distance[i + 1] - speed_table_distance[i]);
		if(vel_diff != 0)
			time = LENGTH / vel_diff;
		else time = 999999;

		now_acc = vel_diff / time;

		if (abs(now_acc) > MAX_ACC && speed_table_distance[i + 1] > speed_table_distance[i]){
			speed_table_distance[i + 1] =  sqrt(MAX_ACC * LENGTH) + speed_table_distance[i];
		}
	}

	for(i = MEMORY_ARRAY_SIZE_DISTANCE - 1; i >= 0; i--){	//	ゆるやかに減速させる
		vel_diff = fabs(speed_table_distance[i - 1] - speed_table_distance[i]);
		if(vel_diff != 0)
			time = LENGTH / vel_diff;
		else time = 999999;
		now_acc = vel_diff / time;

		if (abs(now_acc) > MAX_DEC && speed_table_distance[i - 1] > speed_table_distance[i]){
			speed_table_distance[i - 1] =  sqrt(MAX_DEC * LENGTH) + speed_table_distance[i];
		}
	}

	for(i = MEMORY_ARRAY_SIZE_DISTANCE - 1; i >= 0; i--){
		if(Flag == 0){
			vel_diff = fabs(speed_table_distance[i - 1] - speed_table_distance[i]);
		}
		else{
			vel_diff = fabs(speed_table_distance[i - 1] - speed_temp);
		}

		if(vel_diff != 0){
			time = LENGTH / vel_diff;
		}
		else{
			time = 999999;
		}

		now_acc = vel_diff / time;

		if(speed_table_distance[i - 1] > speed_table_distance[i] && Flag == 0){//	減速するとき
			if(abs(now_acc) > MAX_DEC - 10){	//	加速が足りないとき
				speed_table_distance[i - 1] = speed_table_distance[i];
				speed_temp = sqrt(MAX_DEC * LENGTH) + speed_table_distance[i];

				Flag = 1;
			}
		}
		else if(speed_table_distance[i - 1] > speed_temp && Flag == 1){
			if(abs(now_acc) > MAX_DEC - 10){
				speed_table_distance[i - 1] = speed_table_distance[i];
				speed_temp = sqrt(MAX_DEC * LENGTH) + speed_temp;
			}
		}
		else{
			Flag = 0;
		}

	}


}

//************************************************************************/
//* 役割　：　加速度修正 せめせめ
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
void fix_acceleration_2(){
	#define LENGTH 10 //[mm]
	int i = 0, j = 0;
	float vel_diff = 0;
	float time = 0;
	float now_acc = 0;
	short cnt = 0;
	float speed_temp = 0;
	char Flag = 0;

	speed_table_distance[0] = 1000.;

	for(i = 0; i < MEMORY_ARRAY_SIZE_DISTANCE - 1; i++){
		vel_diff = fabs(speed_table_distance[i + 1] - speed_table_distance[i]);
		if(vel_diff != 0)
			time = LENGTH / vel_diff;
		else time = 999999;

		now_acc = vel_diff / time;

		if (abs(now_acc) > MAX_ACC_2 && speed_table_distance[i + 1] > speed_table_distance[i]){
			speed_table_distance[i + 1] =  sqrt(MAX_ACC_2 * LENGTH) + speed_table_distance[i];
		}
	}

	for(i = MEMORY_ARRAY_SIZE_DISTANCE - 1; i >= 0; i--){	//	ゆるやかに減速させる
		vel_diff = fabs(speed_table_distance[i - 1] - speed_table_distance[i]);
		if(vel_diff != 0)
			time = LENGTH / vel_diff;
		else time = 999999;
		now_acc = vel_diff / time;

		if (abs(now_acc) > MAX_DEC_2 && speed_table_distance[i - 1] > speed_table_distance[i]){
			speed_table_distance[i - 1] =  sqrt(MAX_DEC_2 * LENGTH) + speed_table_distance[i];
		}
	}

	for(i = MEMORY_ARRAY_SIZE_DISTANCE - 1; i >= 0; i--){
		if(Flag == 0){
			vel_diff = fabs(speed_table_distance[i - 1] - speed_table_distance[i]);
		}
		else{
			vel_diff = fabs(speed_table_distance[i - 1] - speed_temp);
		}

		if(vel_diff != 0){
			time = LENGTH / vel_diff;
		}
		else{
			time = 999999;
		}

		now_acc = vel_diff / time;

		if(speed_table_distance[i - 1] > speed_table_distance[i] && Flag == 0){//	減速するとき
			if(abs(now_acc) > MAX_DEC_2 - 10){	//	加速が足りないとき
				speed_table_distance[i - 1] = speed_table_distance[i];
				speed_temp = sqrt(MAX_DEC_2 * LENGTH) + speed_table_distance[i];

				Flag = 1;
			}
		}
		else if(speed_table_distance[i - 1] > speed_temp && Flag == 1){
			if(abs(now_acc) > MAX_DEC_2 - 10){
				speed_table_distance[i - 1] = speed_table_distance[i];
				speed_temp = sqrt(MAX_DEC_2 * LENGTH) + speed_temp;
			}
		}
		else{
			Flag = 0;
		}

	}


}
//************************************************************************/
//* 	役割　：　ポテンショの値から半径を計算して返す
//* 	引数　：　void:
//* 	戻り値：　float: 半径[mm]
//* 	備考 :
//************************************************************************/
float getRadius(){
	float radius = 0;
	float theta = 0;	//[rad]
	int ave_pot = 0;
	int sum_pot = 0;
	//static char cnt = 0;

	for(short i = 0; i < RETENTION_ARRAY_SIZE; i++){
		sum_pot += retention_pot[i];
		retention_pot[i] = 0;
	}

	ave_pot = sum_pot / RETENTION_ARRAY_SIZE;

	//theta = Pot * RAD_PER_AD;
	theta = ave_pot * RAD_PER_AD;
	radius = ROTATION_POINT_FROM_AXEL / tan(theta);

	return radius;
}

//************************************************************************/
//* 	役割　：　ポテンショの値から半径を計算して返す
//* 	引数　：　void:
//* 	戻り値：　float: 半径[mm]
//* 	備考 :
//************************************************************************/
float getAngle(){
	float theta = 0;	//[rad]

	//theta = Pot * RAD_PER_AD;
	theta = ad1[6];

	return theta;
}

//************************************************************************/
//* 役割　：　imuの値から半径を計算して返す
//* 引数　：　void:
//* 戻り値：　float: 半径[mm]
//* 備考 :
//************************************************************************/
float getRadius_imu(){
	float radius = 0, omega = 0;
	/*
	if(zg < 1000 && zg > -1000){
		omega = 99999999;
	}
	else{
		omega = 0.0012 * zg - 0.0377;
	}
	*/
	omega = 0.0012 * zg - 0.0377;
	radius = robot_speed / omega;

	return radius;
}

//************************************************************************/
//* 役割　：　1msごとに角度を配列に格納
//* 引数　：　void:
//* 戻り値：　float: 半径[mm]
//* 備考 :
//************************************************************************/
void angle_measurement(char enable){

	static short size;

	if(enable){
		angle_array[size] = getAngle();
		size++;
	}
	else{
		size = 0;
	}

	if(size >= ANGLE_ARRAY_DATA) size = ANGLE_ARRAY_DATA - 1;
}
//************************************************************************/
//* 役割　：　コース記憶に関するデータを配列にぶちこむ
//* 引数　：　char: enable or disable
//* 戻り値：　short: array number
//* 備考 :
//************************************************************************/
short course_memory(char enable){
	static short memory_access = 0;
	float len = 0;
	float ave = 0;

	if(enable){
		//lowpass_filter(retention_imu, lowpassed_array, RETENTION_ARRAY_SIZE, FREQ, SAMPLERATE, Q);

		for(short i = 0; i < number_stored; i++){
			add_imu += lowpassed_array[i];
		}

		add_imu /= number_stored;	//平均

		len = MM_PER_PULS * (total_encR_memory + total_encL_memory) / 2;	//扇の長さ

		//radius_memory[memory_access] = (add_imu / number_stored);
/*
		if(yaw_angle != 0)	radius_memory[memory_access] = add_imu;
		else radius_memory[memory_access] = 99999999;
*/
		if(yaw_angle != 0)	radius_memory[memory_access] = len / yaw_angle;
		else radius_memory[memory_access] = 99999999;

		len = 0;
		yaw_angle = 0;
		add_imu = 0;

		if(add_imu < 0){
			encorder_playback[memory_access] = total_encR_memory;
		}
		else{
			encorder_playback[memory_access] = total_encL_memory;
		}


		encorder_memoryL[memory_access] = total_encL_memory;
		encorder_memoryR[memory_access] = total_encR_memory;

		memory_access++;
		if(memory_access >= MEMORY_ARRAY_SIZE)	memory_access = MEMORY_ARRAY_SIZE - 1;

		for(short i = 0; i < RETENTION_ARRAY_SIZE; i++){
			retention_imu[i] = 0;
			lowpassed_array[i] = 0;
		}

		total_encL_memory = total_encR_memory = 0;
		flag.retention_reset = 1;
		number_stored = 0;
	}

	return memory_access;
}

//************************************************************************/
//* 役割　：　コース記憶に関するデータを配列にぶちこむ 距離バージョン
//* 引数　：　char: enable(1) or reset(0)
//* 戻り値：　short: array number
//* 備考 :
//************************************************************************/
short course_memory_const_distance(char enable){
	static short data_access = 0;
	float len = 0;

	if(enable){
//		if(ave_total >= COUNT_TO_RECORD){
			len = MM_PER_PULS * (total_encR_reset + total_encL_reset) / 2;	// 	弧 の長さ

			if(yaw_angle != 0){
				imu_radius_memory_distance[data_access] = len / yaw_angle;
			}
			else{
				imu_radius_memory_distance[data_access] = 5000;
			}

			if(imu_radius_memory_distance[data_access] < 0) imu_radius_memory_distance[data_access] *= -1;
			if(imu_radius_memory_distance[data_access] > 5000)	imu_radius_memory_distance[data_access] = 5000;
			//if(imu_radius_memory_distance[data_access] < -5000)	imu_radius_memory_distance[data_access] = -5000;

			data_access++;
		}
//	}
	else {
		data_access = 0;
	}
	total_encR_reset = total_encL_reset = 0;
	yaw_angle = 0;

	if(data_access >= MEMORY_ARRAY_SIZE_DISTANCE)	data_access = MEMORY_ARRAY_SIZE_DISTANCE - 1;

	return data_access;
}

//************************************************************************/
//* 役割　：　サイドセンサー読んだ時の距離を記録する。
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
short side_sensor_memory(char enable){
	static short access = 0;

	if(enable){
		side_line_memory_L[access] = total_encL_distance;
		side_line_memory_R[access] = total_encR_distance;
		access++;

		if(access >= SIDE_LINE_MEMORY_SIZE ) access = SIDE_LINE_MEMORY_SIZE - 1;
	}

	return access;

}

//************************************************************************/
//* 役割　：　簡易ローパスフィルタを通したimuのデータを更新する
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
void updata_imu_data_lowpassed(){

	static float pre_xg = 0, pre_yg = 0, pre_zg = 0;
	static float pre_xa = 0, pre_ya = 0, pre_za = 0;

	omega_x = (xg / 16.4) * PI / 180;
	omega_y = (yg / 16.4) * PI / 180;
	omega_z = (zg / 16.4) * PI / 180;

	alpha_x = (xa / 2048) / 9.806;
	alpha_y = (ya / 2048) / 9.806;
	alpha_z = (za / 2048) / 9.806;

	omega_x_l = lowpass_filter_simple(omega_x, pre_xg, R);
	omega_y_l = lowpass_filter_simple(omega_y, pre_yg, R);
	omega_z_l = lowpass_filter_simple(omega_z, pre_zg, R);

	alpha_x_l = lowpass_filter_simple(alpha_x, pre_xa, R);
	alpha_y_l = lowpass_filter_simple(alpha_y, pre_ya, R);
	alpha_z_l = lowpass_filter_simple(alpha_z, pre_za, R);

	//omega_z_l -= imu_calibration;
	yaw_angle += omega_z_l * DELTA_T;

	pre_xg = omega_x_l;
	pre_yg = omega_y_l;
	pre_zg = omega_z_l;

	pre_xa = alpha_x_l;
	pre_ya = alpha_y_l;
	pre_za = alpha_z_l;

}

//************************************************************************/
//* 役割　：　簡易ローパスフィルタ
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
float lowpass_filter_simple(float x, float x0, float r){

	return ((r)*(x) + (1.0 - (r))* (x0));

}
//************************************************************************/
//* 役割　：　再生走行中白線読んだらエンコーダ値補正する
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
char encorder_correction(short access, int reference){
	char ret = 0;

	float ave_memory = (encorder_memoryL[data_access] + encorder_memoryR[data_access]) / 2;
	float ave_total = (total_encL_memory + total_encR_memory) / 2;

	if((reference - 2500 < encorder_playback[access]) && (encorder_playback[access] < reference + 2500)){
	//if((ave_total - 2500 < ave_memory) && (ave_memory < ave_total + 2500)){
			total_encL_memory = encorder_memoryL[access];
			total_encR_memory = encorder_memoryR[access];
			ret = 1;
		}

		return ret;
}

//************************************************************************/
//* 役割　：　再生走行中白線読んだらエンコーダ値補正する
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
char encorder_correction_distance(char enable){
	char ret = 0;
	short line_access = 0;
	static short pre_line_access = 0;
	float ave_now;
	float ave_memory;
	char Flag = 1;

	if(enable){
		ave_now = (total_encL_distance + total_encR_distance) / 2;
		line_access = pre_line_access + 1;

		while(Flag){
			ave_memory = (side_line_memory_L[line_access] + side_line_memory_R[line_access]) / 2;

			if((ave_now - 1000 < ave_memory) && (ave_memory < ave_now + 1000)){
				total_encL_distance = side_line_memory_L[line_access];
				total_encR_distance = side_line_memory_R[line_access];
				ret = 1;
				Flag = 0;
				timer.correction = 0;
				pre_line_access = line_access;
			}

			line_access++;

			if(line_access >= SIDE_LINE_MEMORY_SIZE){
				Flag = 0;
			}

		}
	}

	return ret;

}

//************************************************************************/
//* 役割　：　今の計測速度を更新
//* 引数　：　char: フラグ enable
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
void updata_now_speed(float *l, float *r){
	*l = MM_PER_PULS * 1000. * getEncorder_L();			//[mm/s]
	*r = MM_PER_PULS * 1000. * getEncorder_R(); 		//[mm/s]
}

//************************************************************************/
//* 役割　：　記憶したデータをもとにtarget_robot_speedを更新する
//* 引数　：　char: フラグ enable
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
void updata_robot_speed(char Flag){

	//static short data_access = 0;
	//static float ave_enc;
	//static short pre;
	//static signed char dec = 0;
	//static signed char ac = 0;
	static char ignore = 0;
	static char cont = 0;
	static short straight = 0;
	float ave_memory = 0, ave_memory_after = 0;;
	float pre_ave_memory = 0;
	float ave_total = 0;
	static char check = 0;
	int reference_encorder = 0;
	static char ignore_cnt = 0;
	static char status = 0;

	float ave_velo = 0;

	float now_speed_L, now_speed_R;

	now_speed_L = MM_PER_PULS * 1000. * getEncorder_L();		//[mm/s]
	now_speed_R = MM_PER_PULS * 1000. * getEncorder_R(); 		//[mm/s]

	ave_velo = (now_speed_L + now_speed_R) / 2;


	if(flag.updata_robot_speed_reset){
		ignore = 0;
		cont = 0;
		straight = 0;
		check = 0;
		ignore_cnt = 0;

		flag.updata_robot_speed_reset = 0;

	}
	if(Flag){
		//参照するエンコーダ値更新
		if(radius_memory[data_access] < 0)	reference_encorder = total_encR_memory;
		else reference_encorder = total_encL_memory;


		//---------------------------エンコーダー値補正---------------------//
		/*
		if(flag.side == 1){
			if(ignore == 0){
				check = encorder_correction(data_access, reference_encorder);
			}
			flag.side = 0;
		}
		*/

		//減速したあと白線読むまで待つ
		if(ignore == 1){
			ave_memory = (encorder_memoryL[data_access - 1] + encorder_memoryR[data_access - 1]) / 2;
			ave_total = (total_encL_memory + total_encR_memory) / 2;
			if(reference_encorder >= encorder_playback[data_access - 1]){
			//if(ave_total >= ave_memory){
				if(flag.side == 1){
					check = encorder_correction(data_access - 1, reference_encorder);
					flag.side = 0;
				}

				ignore = 0;
				//flag.side = 0;

				status = 1;
			}
		}
		//加速する前にまっすぐになるまでまつ 減速する暇がある
		else if(cont == 1){
			ave_memory = (encorder_memoryL[data_access] + encorder_memoryR[data_access]) / 2;
			ave_total = (total_encL_memory + total_encR_memory) / 2;

			if((Pot < 20 && Pot > -20)){
				straight++;
			}
			else straight = 0;

			if(straight >= 500 && ave_total - ave_memory >= continued_cnt){
				data_access++;
				cont = 0;
				straight = 0;
				flag.side = 0;
			}

			status = 2;
		}
		//減速するときの処理
		else if(speed_table[data_access + 1] < speed_table[data_access]){
			ave_memory = (encorder_memoryL[data_access] + encorder_memoryR[data_access]) / 2;
			ave_memory_after = (encorder_memoryL[data_access + 1] + encorder_memoryR[data_access + 1]) / 2;
			ave_total = (total_encL_memory + total_encR_memory) / 2;

			//if(ave_total >= ave_memory - deceleration_cnt){
			if(reference_encorder >= encorder_playback[data_access] - deceleration_cnt){
				if(flag.side == 1){
					check = encorder_correction(data_access, reference_encorder);
					flag.side = 0;
				}

				data_access++;
				ignore = 1;
				flag.side = 0;
				//}
				/*
				else if(ave_memory_after - ave_memory < deceleration_cnt){
					data_access++;
					speed_table[data_access] = LOW_SPEED;
					ignore = 2;
				}
				*/
			}

			status = 3;
		}
		//同じか加速するとき
		else{
			ave_memory = (encorder_memoryL[data_access] + encorder_memoryR[data_access]) / 2;
			ave_memory_after = (encorder_memoryL[data_access + 1] + encorder_memoryR[data_access + 1]) / 2;
			ave_total = (total_encL_memory + total_encR_memory) / 2;

			//if(flag.side == 1){
			//if(ave_total >= ave_memory){
			if(reference_encorder >= encorder_playback[data_access]){
				if(flag.side == 1){
					check = encorder_correction(data_access, reference_encorder);
					flag.side = 0;
				}
				//if(ave_memory_after - ave_memory < deceleration_cnt || speed_table[data_access + 1] == speed_table[data_access]){	//減速する暇がない　or 同じ速度のとき
				if(speed_table[data_access + 1] == speed_table[data_access]){	//同じ速度のとき
					cont = 0;
					data_access++;
					flag.side = 0;
				}

				else if(ave_memory_after - ave_memory < deceleration_cnt && speed_table[data_access + 1] > speed_table[data_access]){//減速する暇がない　and 加速するとき
					cont = 0;
					data_access++;
					speed_table[data_access] = LOW_SPEED;
					flag.side = 0;
				}

				else{	//減速する暇がある　かつ　加速するとき
					cont = 1;
				}
				//flag.side = 0;
			}

			status = 4;
		}


		if(check == 1){
			LED('R');
			if(timer.check_timer > 100){
				check = 0;
				timer.check_timer = 0;
			}
		}
		else{
			timer.check_timer = 0;
			LED('N');
		}

		//target_robot_speed = speed_table[data_access];

		if(data_access >= MEMORY_ARRAY_SIZE) data_access = MEMORY_ARRAY_SIZE - 1;

	}
}

//************************************************************************/
//* 役割　：　一定距離ごとに記憶したデータをもとにtarget_robot_speedを更新する
//* 引数　：　char: フラグ enable
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
void updata_robot_speed_distance(char enable){
	//static short data_access = 0;
	static float pre_robot_speed = 0;
	float now_speed_L, now_speed_R;

	if(enable){
		now_speed_L = MM_PER_PULS * 1000. * getEncorder_L();		//[mm/s]
		now_speed_R = MM_PER_PULS * 1000. * getEncorder_R(); 		//[mm/s]

		if(flag.yaw_zero == 1){
			robot_speed = speed_table_distance[data_access_dis];
		}


		pre_robot_speed = robot_speed;

		data_access_dis++;
		if(data_access_dis >= MEMORY_ARRAY_SIZE_DISTANCE) data_access_dis = MEMORY_ARRAY_SIZE_DISTANCE - 1;
	}
	else{
		data_access_dis = 0;
	}

}
//************************************************************************/
//* 役割　：　imuデータを配列に格納する
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
/*
void store_imu_data(char enable, short *store_num){
	static short access;
	static unsigned long cnt = 0;
	static char over_flag = 0;

	if(enable){
		if(flag.retention_reset == 1){
			access = 0;
			cnt = 0;
			over_flag = 0;
			flag.retention_reset = 0;
		}

		if(over_flag == 0){
			*store_num = access;
		}
		else *store_num = RETENTION_ARRAY_SIZE;

		//retention_imu[access] = omega_z_l;
		retention_imu[access] = yaw_angle;
		access++;
		cnt++;

		if(access >= RETENTION_ARRAY_SIZE){
			access = 0;
			over_flag = 1;
		}
		if(cnt >= LONG_MAX)	cnt = LONG_MAX;

		//add_pot += Pot;
		//if(add_pot >= INT_MAX)	add_pot = INT_MAX;
		//if(add_pot <= INT_MIN)	add_pot = INT_MIN;

	}
}
*/
//************************************************************************/
//* 役割　：　imuデータを配列に格納する2
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
void store_log_data(char Flag){
	static short access;
	float omega = 0, velo = 0;
	float now_speed_L, now_speed_R;
	static int totalL, totalR;
	static int8_t cnt = 1;

	if(Flag){

		now_speed_L = MM_PER_PULS * 1000. * getEncorder_L();		//[mm/s]
		now_speed_R = MM_PER_PULS * 1000. * getEncorder_R(); 		//[mm/s]

		omega = now_speed_R / (TRED/2);
		velo = (now_speed_L + now_speed_R) / 2;

		totalL += getEncorder_L();
		totalR += getEncorder_R();

		//if(cnt >= 5){	//5ms 0.005 //10ms 0.01s
			//various_memory1[access] = getEncorder_L();
			various_memory1[access] = velo;
			various_memory2[access] = check_in;

			//various_memory2[access] = getEncorder_R();
			cnt = 0;
			access++;
		//}
		cnt++;

		//various_memory3[access] = speed_L;
		//various_memory4[access] = speed_R;



		if(access >= MEMORY_ARRAY_SIZE_2 - 1){
			access = 0;
			flag.log_store = 0;
		}
	}

}
//************************************************************************/
//* 役割　：　imuデータを配列に格納する
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
void store_pot_data(char enable, short *store_num){
	static short access;
	static unsigned long cnt = 0;
	static char over_flag = 0;

	if(enable){
		if(flag.retention_reset == 1){
			access = 0;
			cnt = 0;
			over_flag = 0;
			flag.retention_reset = 0;
		}

		if(over_flag == 0){
			*store_num = access;
		}
		else *store_num = RETENTION_ARRAY_SIZE;

		retention_pot[access] = Pot;
		access++;
		cnt++;

		if(access >= RETENTION_ARRAY_SIZE){
			access = 0;
			over_flag = 1;
		}
		if(cnt >= LONG_MAX)	cnt = LONG_MAX;

		//add_pot += Pot;
		//if(add_pot >= INT_MAX)	add_pot = INT_MAX;
		//if(add_pot <= INT_MIN)	add_pot = INT_MIN;

	}
}
//************************************************************************/
//* 役割　：　AD値をデジタルに変換したものを返す
//* 引数　：　char: '1'(Line1), '4'(Line4), 'L'(SideL), 'R'(SideR)
//* 戻り値：　short: 0 or 1
//* 備考 : なし
//************************************************************************/
char getDigital(char name){
	char ret = 0;
	static char cnt1, cnt4, cntL, cntR;

	switch(name){
		case '1':
			if(Line1 < LINE_DIGITAL_THRESHOLD){
				if(cnt1 > 1) ret = 1;
				cnt1++;
			}
			else cnt1 = 0;
		break;
		case '4':
			if(Line4 < LINE_DIGITAL_THRESHOLD){
				if(cnt4 > 1) ret = 1;
				cnt4++;
			}
			else cnt4 = 0;
		break;
		case 'L':
			if(SideL < SIDE_DIGITAL_THRESHOLD_LEFT){
				if(cntL > 1) ret = 1;
				cntL++;
			}
			else cntL = 0;
		break;
		case 'R':
			if(SideR < SIDE_DIGITAL_THRESHOLD_RIGHT){
				if(cntR > 1) ret = 1;
				cntR++;
			}
			else cntR = 0;
		break;
	}

	return ret;
}

//************************************************************************/
//* 役割　：　クロスライン時分岐処理
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 : 追従かクロスラインかのフラグを上げ下げする
//************************************************************************/
void choice_following_mode(char Flag){
	static char state1 = 10;
	//static char state2 = 10;
	//static int encL, encR;
	//int ave_enc = 0;

	if(Flag){
		/*--------------------	ライン追従・角度固定分岐------------------*/
		switch(state1){
			case 10:
				if(getDigital('1') == 1 && getDigital('4') == 1){
					state1 = 20;
					measurement_19mm(total_encL, total_encR, 0);

					flag.following = 0;
				}

				else flag.following = 1;

			break;

			case 20:
				if(measurement_19mm(total_encL, total_encR, 0)){
					state1 = 10;
					flag.angle = 0;
				}
				else{
					flag.angle = 1;
				}

			break;
		}

		/*--------------------直線、カーブ選択------------------*/
		flag.curve = 1;
		flag.straight = 1;

		/*--------------------直線、カーブ分岐------------------*/
		/*
		if(Pot < -30 && Pot > 30){
			flag.straight = 1;
			flag.curve = 0;
		}
		else{
			flag.straight = 0;
			flag.curve = 1;
		}
		*/
		//flag.straight = 1;
		//flag.curve = 0;

		/*--------------------カーブ曲がったあとステアがまっすぐ付近になったら直線モードに移行------------------*/
		/*
		switch(state2){
			case 10:
				LED('W');

				flag.straight = 1;
				flag.curve = 0;

				if(Pot > 15 || Pot < -15){ 	//ある程度ステアがきれたら
					if(timer.my_timer1 > 50){
						state2 = 20;
					}
				}
				else timer.my_timer1 = 0;

			break;
			case 20:
				LED('R');

				flag.straight = 0;
				flag.curve = 1;

				if(Pot < 100 && Pot > -100){	//真っ直ぐになったら
					if(timer.my_timer1 > 0){
						state2 = 10;
					}
				}
				else timer.my_timer1 = 0;


			break;

		}
		*/

/*
		else if((Pot < 20 && Pot > -20) && flag.curve_to_straight == 1){
			flag.straight = 1;
			flag.curve = 0;
			timer.my_timer1 = 0;
			flag.curve_to_straight = 2;
			LED('G');
		}

		else if((timer.my_timer1 > 1000) && flag.curve_to_straight == 2){
			flag.straight = 0;
			flag.curve = 1;
			flag.curve_to_straight = 0;
			LED('B');
		}
		else{

			flag.straight = 1;
			flag.curve = 0;
		}
*/


	}

}

//************************************************************************/
//* 役割　：　この関数が呼ばれてから19mm進んだら1を返す
//* 引数　：　int, int: 左エンコーダ, 右エンコーダ
//* 戻り値：　char:　1 or 0
//* 備考 : 19mm測り終えるまでは新しい計測は始められない
//************************************************************************/
char measurement_19mm(int encL, int encR, char reset){
	char ret = 0;
	float ave_enc = 0;
	static char Flag = 0;
	static int keep_encL, keep_encR;

	if(Flag == 0){
		keep_encL = encL;
		keep_encR = encR;
		Flag = 1;
	}

	else{
		ave_enc = ((total_encL - keep_encL) + (total_encR - keep_encR)) / 2;
		if(ave_enc > MM19_PER_MPP){
			ret = 1;
			Flag = 0;
		}
	}

	if(reset == 1){
		keep_encL = 0;
		keep_encR = 0;
		Flag = 0;
	}
	return ret;
}

//************************************************************************/
//* 役割　：　この関数が呼ばれてから19mm進んだら1を返す
//* 引数　：　int, int: 左エンコーダ, 右エンコーダ
//* 戻り値：　char:　1 or 0
//* 備考 : 19mm測り終えるまでは新しい計測は始められない
//************************************************************************/
char measurement_19mm_2(int encL, int encR, char reset){
	char ret = 0;
	float ave_enc = 0;
	static char Flag = 0;
	static int keep_encL, keep_encR;

	if(Flag == 0){
		keep_encL = encL;
		keep_encR = encR;
		Flag = 1;
	}

	else{
		ave_enc = ((total_encL - keep_encL) + (total_encR - keep_encR)) / 2;
		if(ave_enc > MM19_PER_MPP_PLUS){
			ret = 1;
			Flag = 0;
		}
	}

	if(reset == 1){
		keep_encL = 0;
		keep_encR = 0;
		Flag = 0;
	}
	return ret;
}

//************************************************************************/
//* 役割　：　この関数が呼ばれてから19mm進んだら1を返す
//* 引数　：　int, int: 左エンコーダ, 右エンコーダ
//* 戻り値：　char:　1 or 0
//* 備考 : 19mm測り終えるまでは新しい計測は始められない
//************************************************************************/
char measurement_19mm_3(int encL, int encR, char reset){
	char ret = 0;
	float ave_enc = 0;
	static char Flag = 0;
	static int keep_encL, keep_encR;


	if(Flag == 0){
		keep_encL = encL;
		keep_encR = encR;
		Flag = 1;
	}

	else{
		ave_enc = ((total_encL - keep_encL) + (total_encR - keep_encR)) / 2;
		if(ave_enc > MM19_PER_MPP2){
			ret = 1;
			Flag = 0;
		}
	}

	if(reset == 1){
		keep_encL = 0;
		keep_encR = 0;
		Flag = 0;
	}

	return ret;
}
//************************************************************************/
//* 役割　：　この関数が呼ばれてから100mm進んだら1を返す
//* 引数　：　int, int: 左エンコーダ, 右エンコーダ
//* 戻り値：　char:　1 or 0
//* 備考 : 19mm測り終えるまでは新しい計測は始められない
//************************************************************************/
char measurement_100mm(int encL, int encR, char reset){
	char ret = 0;
	float ave_enc = 0;
	static char Flag = 0;
	static int keep_encL, keep_encR;

	if(Flag == 0){
		keep_encL = encL;
		keep_encR = encR;
		Flag = 1;
	}

	else{
		ave_enc = ((total_encL - keep_encL) + (total_encR - keep_encR)) / 2;
		if(ave_enc > MM100_PER_MPP){
			ret = 1;
			Flag = 0;
		}
	}

	if(reset == 1){
		keep_encL = 0;
		keep_encR = 0;
		Flag = 0;
	}

	return ret;
}

void mesurment_reset(){
	char reset = 1;
	measurement_19mm(0, 0, reset);
	measurement_19mm_2(0, 0, reset);
	measurement_19mm_3(0, 0, reset);
	measurement_100mm(0, 0, reset);
	goal_area_processing(reset);
	reset = 0;
}

//************************************************************************/
//* 役割　：　センサーがライン追従をする
//* 引数　：　short: フラグ
//* 戻り値：　void:
//* 備考 : ゆくゆくはゲインを引数にするかも
//************************************************************************/
void sensor_following(char flag){
	float input_vcm = 0;
	float kp = 1.0, ki  = 10, kd = 0.002;	//float kp = 1.6, ki  = 10, kd = 0.004;, 3s first float kp = 1, ki  = 0, kd = 0.001; 2sbeforer float kp = 2.0, ki  = 10, kd = 0.004; 2s
	float p = 0, d = 0, i = 0;
	float devi = 0;
	static float pre_devi;

	devi = (Line2 + Line1)/2 - (Line4 + Line3)/2 ;

	p = kp * devi;
	d = kd * (devi - pre_devi) / DELTA_T;
	i += ki * DELTA_T * devi;

	if(i > 300)	i = 300;
	else if(i < -300)	i = -300;

	input_vcm = p + d + i;

	pre_devi = devi;

	if(flag) vcm_ctrl(input_vcm, &monitoring_vcm_pulse_width);

}

//************************************************************************/
//* 役割　：　角度制御
//* 引数　：　char, short: フラグ, 目標値
//* 戻り値：　void:
//* 備考 : ゆくゆくはゲインを引数にするかも
//************************************************************************/
void angle_ctrl(char flag, short target_pot){
	float input_vcm = 0;
	float kp = 1.6543, ki  = 0.10134, kd = 6.7514; //float kp = 10, ki  = 0, kd = 0.07;
	float p = 0, d = 0, i = 0;
	float devi = 0;
	static float pre_devi;

	//devi =  (target_pot - Pot);
	devi =  (target_pot - ad1[6]);

	p = kp * devi;
	d = kd * (devi - pre_devi) / DELTA_T;
	i += ki * DELTA_T * devi;

	if(i > 300)	i = 300;
	else if(i < -300)	i = -300;

	input_vcm = p + d + i;

	pre_devi = devi;

	if(flag) vcm_ctrl(input_vcm, &monitoring_vcm_pulse_width);
}

//************************************************************************/
//* 役割　：　センサーバーの角度によって左右のデフを更新する
//* 引数　：　short, float, float *, float *: ポテンショのAD値をデフ参照用に加工したもの, ロボット全体の目標速度, 更新されるデフをかけた左の速度, 更新されるデフをかけた左の速度
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
void updata_curve_val(char Flag, short ref, float _robot_speed, float *p_def_speedL, float *p_def_speedR){
	float radius = 0;

	if(Flag){

		if(ref != 0) radius = ROTATION_POINT_FROM_AXEL / tan(ref * RAD_PER_AD);
		else radius = 99999999;

		*p_def_speedL = _robot_speed * ((radius + (TRED / 2)) / radius);
		*p_def_speedR = _robot_speed * ((radius - (TRED / 2)) / radius);

		//*p_def_speedL = 500.;
		//*p_def_speedR = 500.;



		/*
		if(ref < 0){
			ref *= -1;
			if(ref > DEF_ARRAY_SIZE) ref = DEF_ARRAY_SIZE - 1;
			*p_def_speedL = _robot_speed * R_def[ref];
			*p_def_speedR = _robot_speed * L_def[ref];
		}
		else{
			if(ref > DEF_ARRAY_SIZE) ref = DEF_ARRAY_SIZE - 1;
			*p_def_speedL = _robot_speed * L_def[ref];
			*p_def_speedR = _robot_speed * R_def[ref];
		}
		*/
	}
}

//************************************************************************/
//* 役割　：　ライントレースするための左右の速度を更新する
//* 引数　：　float, float, float *, float *, : デフをかけた左の速度, デフをかけた右の速度, ポテンショのAD値をもとに出した左速度,ポテンショのAD値をもとに出した右速度
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
void updata_straight_val(char Flag, float _robot_speed, float *p_speedL, float *p_speedR, float kp, float ki, float kd){
	//float kp = 30, ki  = 100, kd = 0.5;	//beforer float float kp = 2.5, ki  = 20, kd = 0.04; 静かなゲインfloat kp = 2, ki  = 0, kd = 0.02;

	float p = 0, d = 0, i = 0;
	float devi = 0;
	static float pre_devi;
	static short access = 0;
	//#define STORE_POT_NUM 10
	//static short ave_access = 0;
	//static float store_pot[STORE_POT_NUM];
	//float ave_pot = 0;
	static char memory_Flag = 1;

	if(Flag){
		devi = 0 - Pot;		//目標値が0
		//devi = 0 - ave_pot;		//目標値が0

		p = kp * devi;
		d = (float)(kd * (devi - pre_devi) / DELTA_T);
		i += ki * DELTA_T * devi;

		if(i > 1000)	i = 1000;
		else if(i < -1000)	i = -1000;

		*p_speedL = _robot_speed - (p + d + i);
		*p_speedR = _robot_speed + (p + d + i);

/*
		if(memory_Flag){
			various_memory1[access] = p + d + i;
			various_memory2[access] = Pot;
			access++;
		}

		if(access >= MEMORY_ARRAY_SIZE_2){
			access = MEMORY_ARRAY_SIZE_2 - 1;
			memory_Flag = 0;
		}
*/
		pre_devi = devi;

	}

}

//************************************************************************/
//* 役割　：　ライントレースする
//* 引数　：　char:enable
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
void trace(char Flag, float kp, float ki, float kd, float *val){
	float p = 0, d = 0, i = 0;
	float devi = 0;
	static float pre_devi;
	float input_L = 0, input_R = 0;

	if(Flag){
		devi = 0 - Pot;		//目標値が0

		p = kp * devi;
		d = (float)(kd * (devi - pre_devi) / DELTA_T);
		i += ki * DELTA_T * devi;

		if(i > 1000)	i = 1000;
		else if(i < -1000)	i = -1000;

		input_L = - (p + d + i);
		input_R =   (p + d + i);
		*val = input_R;

		pre_devi = devi;

		//maxon_ctrl(input_L, input_R);
	}

}

/************************************************************************/
//* 役割　：　この関数が呼ばれたらライントレースする
//* 引数　：　short: フラグ
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
void Line_trace(char Flag){
	float input_L = 0, input_R = 0;
	float Rval = 0, Tval = 0;


	if(Flag){
		//updata_curve_val(flag.curve, Def_ref, robot_speed, &speed_L, &speed_R);	//カーブ

		//updata_straight_val(flag.straight, robot_speed, &speed_L, &speed_R, gain.kp, gain.ki, gain.kd);			//	/直線
		//trace(flag.trace, gain.kp, gain.ki, gain.kd, &trace_val);
		//printf("%f\r\n", *p_def_speedL);

		//speed_ctrl(flag.speed_ctrl_enable, speed_L, speed_R, robot_speed, &speed_val);							//	速度制御する

		TranslationSpeed(1, &Tval);
		RotationSpeed(1, &Rval);

		/*
		input_L = -trace_val + speed_val;
		input_R =  trace_val + speed_val;
		*/

		input_L = Tval + Rval;
		input_R = Tval - Rval;

		maxon_ctrl(input_L, input_R);

	}
}

float getTargetOmega(){
	float theta = 0;
	float Radius = 0;
	float SpeedL = 0, SpeedR = 0;
	float SpeedC = 0;
	//static float pre_theta;
	float ret = 0;

	SpeedL = MM_PER_PULS * 1000. * getEncorder_L();		//[mm/s]
	SpeedR = MM_PER_PULS * 1000. * getEncorder_R(); 	//[mm/s]
	SpeedC = (SpeedL + SpeedR) / 2;

	theta = Pot * RAD_PER_AD;
	if(theta != 0) Radius = ROTATION_POINT_FROM_AXEL / tan(theta);
	else Radius = 99999999;

	ret = SpeedC / Radius;

	//ret = (theta - pre_theta) / DELTA_T;

	//pre_theta = theta;

	return -ret;
	//return theta;

}

/************************************************************************/
//* 役割　：　手押しモード
//* 引数　：　short: フラグ
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
void hand_push_trace(char flag){

	float input_L = 0, input_R = 0;
	float kp = 3, ki  = 10, kd = 0.001;
	float p = 0, d = 0, i = 0;
	float devi = 0;
	static float pre_devi;

	devi = 0 - Pot;		//目標値が0

	p = kp * devi;
	d = (float)(kd * (devi - pre_devi) / DELTA_T);
	i += ki * DELTA_T * devi;

	input_L = -(p + d + i);
	input_R =   p + d + i;

	pre_devi = devi;

	if(flag) maxon_ctrl(input_L, input_R);

}

/************************************************************************/
//* 役割　：　速度制御コントロール
//* 引数　：　float, float: 左の目標速度, 右の目標速度
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
void speed_ctrl(char enable, float targetL, float targetR, float target, float *val){
	#define I_LIMIT 2000 //2000, 0.9, 150,

	float input_L = 0, input_R = 0, input = 0;
	float kp = 0.73598, ki = 3.7518, kd = 0;	//float kp = 1.7, ki = 140, kd = 0;
	float p_L = 0, p_R = 0, p = 0;
	static float i_L = 0, i_R = 0, i = 0;
	float d_L = 0, d_R = 0, d = 0;

	float _now_speed_L = 0, _now_speed_R = 0;
	float devi_L = 0, devi_R = 0, devi = 0;
	static float pre_devi_L = 0, pre_devi_R = 0, pre_devi = 0;
	float speed = 0;

	static short cnt;
	static float pre_speed;
	static char dec = 0;
	static int pre_encorder = 0;
	int ave_encorder = 0;

	//static float pre_robot_speed = 0;

	if(enable){

		_now_speed_L = MM_PER_PULS * 1000. * getEncorder_L();		//[mm/s]
		_now_speed_R = MM_PER_PULS * 1000. * getEncorder_R(); 		//[mm/s]

		//_now_speed_L = now_speed_L;
		//_now_speed_R = now_speed_R;

		speed = (_now_speed_L + _now_speed_R) / 2;

		//targetL *= 1.15;
		//targetR *= 1.15;
/*
		if(targetL > targetR){//補正
			targetL *= 1.1;
		}
		else if(targetR > targetL){//補正
			targetR *= 1.1;
		}
*/
		/*
		devi_L = targetL - _now_speed_L;
		p_L = devi_L * kp;
		i_L += devi_L * DELTA_T * ki;
		///d_L = kd * (devi_L - pre_devi_L) / DELTA_T;

		devi_R = targetR - _now_speed_R;
		p_R = devi_R * kp;
		i_R += devi_R * DELTA_T * ki;
		//d_R = kd * (devi_R - pre_devi_R) / DELTA_T;
		*/

		devi = target - speed;
		p = devi * kp;
		i += devi * DELTA_T * ki;
		//d = kd * (devi - pre_devi) / DELTA_T;

/*
		if(i_R > I_LIMIT)	i_R = I_LIMIT;
		else if(i_R < -I_LIMIT)	i_R = -I_LIMIT;
		if(i_L > I_LIMIT)	i_L = I_LIMIT;
		else if(i_L < -I_LIMIT)	i_L = -I_LIMIT;
*/
		//if(i > I_LIMIT)	i = I_LIMIT;
		//else if(i < -I_LIMIT)	i = -I_LIMIT;

//------------------------FF--------------------------//
		/*
		//ave_encorder = (total_encL_distance + total_encR_distance) / 2;

		if((target - pre_speed) < -800 && dec == 0){	//減速　かつ　現在速度が目標より速いとき
			dec = 1;
			//pre_encorder = ave_encorder;
			ff_duty = -3000.;
		}
		else{
			pre_speed = target;
		}

		if(dec == 1){
			/
			if(ave_encorder - pre_encorder > 200){
				dec = 0;
				ff_duty = 0.;
			}
			/

			if(speed <= target + 200){
				cnt++;
			}
			else{
				cnt = 0;
			}


			if(cnt >= 10){
				dec = 0;
				cnt = 0;
				ff_duty = 0.;
			}

		}



		if(ff_duty == -3000.){
			LED('R');
		}
		else{
			LED('N');
		}
		*/
		//calc_feed_forward(ff_accele_L, ff_accele_R, 0, &ff_duty_L, &ff_duty_R);

/*
		if(_now_speed_L >= targetL - 100 && _now_speed_L <= targetL + 100){//目標に近づいたら
			cntL++;
		}
		else{
			cntL = 0;
		}

		if(cntL >= 10){	//目標値付近
			ff_accele_L = 0;
			i_L = 0;
			cntL = 0;
		}
		else if(fabs(pre_speed_L - targetL) > 100){
			if(_now_speed_L > targetL){	//目標値より高い場合
				ff_accele_L = -10;
			}
			else if(_now_speed_L < targetL) {//目標値より低い場合
				ff_accele_L = 10;
			}
		}

		if(_now_speed_R >= targetR - 100 && _now_speed_R <= targetR + 100){
			cntR++;
		}
		else{
			cntR = 0;
		}

		if(cntR >= 10){
			ff_accele_R = 0;
			i_R = 0;
			cntR = 0;
		}
		else if(fabs(pre_speed_R - targetR) > 100){
			if(_now_speed_R > targetR){	//目標値より高い場合
				ff_accele_R = -10;
			}
			else if(_now_speed_R < targetR) {//目標値より低い場合
				ff_accele_R = 10;
			}
			//i_R = 0;
		}

		pre_speed_L = targetL;
		pre_speed_R = targetR;

		calc_feed_forward(ff_accele_L, ff_accele_R, 0, &ff_duty_L, &ff_duty_R);
*/
//--------------------------------------------------//

	/*
		if(((pre_speed_L + pre_speed_R)/2) - ((targetL + targetR)/2) > 100){
			ff_duty_L = ff_duty_R = -2000;
		}
		else ff_duty_L = ff_duty_R = 0;
		pre_speed_L = targetL;
		pre_speed_R = targetR;
	*/
/*
		if(speed > robot_speed + 100){	//	現在速度が目標よりある程度高い場合
		//if(pre_robot_speed > robot_speed){
			//ff_duty_L = ff_duty_R = -10000;
			//input_L = input_R = -2000;
			timer.distance = 0;
			dec = 1;
			LED('B');
		}
		else{
			//ff_duty_L = ff_duty_R = 0;
			dec = 0;
		}

		if(dec == 1){
			LED('B');
			ff_duty_L = ff_duty_R = -10000;
			if(speed <= robot_speed + 100){
				dec_cnt++;

			}
			else{
				dec_cnt = 0;
			}

			if(dec_cnt > 10){
				dec = 0;
				dec_cnt = 0;
			}
		}
		else{
			LED('N');
			ff_duty_L = ff_duty_R = 0;

		}

		pre_robot_speed = robot_speed;
*/
		//input_L = ((p_L + i_L) + ff_duty_L);
		//input_R = ((p_R + i_R) + ff_duty_R);
		//input_L = (p_L + i_L);
		//input_R = (p_R + i_R);
		//input_L = 1200;
		//input_R = 1200;

		*val = p + i + ff_duty;
		maxon_ctrl(p + i, p + i);
/*
		pre_devi_L = devi_L;
		pre_devi_R = devi_R;
		pre_devi = devi;
*/
		//maxon_ctrl(input_L, input_R);
	}
}

/************************************************************************/
//* 役割　：　速度フィードフォワード値計算
//* 引数　：　float:
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
void calc_feed_forward(float acc_L, float acc_R, float velo, float * duty_L, float * duty_R){
	float T_tire_L = 0, T_tire_R = 0;
	float T_motor_L = 0, T_motor_R = 0;
	float I_L = 0, I_R = 0;
	float V_velo_L = 0, V_velo_R = 0;
	float V_acc_L = 0, V_acc_R = 0;
	float Vr_L = 0, Vr_R = 0;
	float duty_acc_L = 0, duty_velo_L = 0, duty_acc_R = 0, duty_velo_R = 0;
	float n_target = 0;
	float n_now_L = 0, n_now_R = 0;

	//#define ACC 14
	//#define VELO 1

	T_tire_L = R_TIRE * (MACHINE_WEIGHT * acc_L) / 2;	//[Nm]
	T_tire_R = R_TIRE * (MACHINE_WEIGHT * acc_R) / 2;	//[Nm]
	T_motor_L = T_tire_L / GIRE_RATIO;
	T_motor_R = T_tire_R / GIRE_RATIO;

	I_L = T_motor_L / KM_L;
	I_R = T_motor_R / KM_R;

	n_now_L = 60 * getEncorder_L() / 2.048;	//rpm
	n_now_R = 60 * getEncorder_R() / 2.048;	//rpm

	Vr_L = Ke_L * n_now_L;
	Vr_R = Ke_L * n_now_R;

	V_acc_L = MOTOR_RESISTANCE * I_L + Vr_L;
	V_acc_R = MOTOR_RESISTANCE * I_R + Vr_R;

	duty_acc_L = MAX_DUTY * (V_acc_L / INPUT_VOLTAGE);
	duty_acc_R = MAX_DUTY * (V_acc_R / INPUT_VOLTAGE);


	//---velo--//
	n_target =(velo / WHEEL_D * PI) * 60;	//rpm


	n_target *= GIRE_RATIO;

	V_velo_L = n_target / KE_L;
	V_velo_R = n_target / KE_R;

	duty_velo_L = MAX_DUTY * (V_velo_L / INPUT_VOLTAGE);
	duty_velo_R = MAX_DUTY * (V_velo_R / INPUT_VOLTAGE);

	*duty_L = duty_acc_L;
	*duty_R = duty_acc_R;

}
/************************************************************************/
//* 役割　：　加速させる
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
void accelerator(char Flag, float accele){

#define OFFSET 500	//[mm/s]
	if(Flag){
		float now_speed_L = MM_PER_PULS * 1000. * getEncorder_L();			//[mm/s]
		float now_speed_R = MM_PER_PULS * 1000. * getEncorder_R(); 		//[mm/s]
		//static float pre_robot_speed = 0;

		if(fabs(now_speed_L - speed_L) > 500){	//目標速度と現在の速度の差がある程度あったら
			if(now_speed_L > speed_L){	//目標値より高い場合
				ff_accele_L = -10;
			}
			else if(now_speed_L < speed_L) {//目標値より低い場合
				ff_accele_L = 10;
			}
		}
		else{
			ff_accele_L = 0;
		}

		if(fabs(now_speed_R - speed_R) > 500){	//目標速度と現在の速度の差がある程度あったら
			if(now_speed_R > speed_R){	//目標値より高い場合
				ff_accele_R = -10;
			}
			else if(now_speed_R < speed_R) {//目標値より低い場合
				ff_accele_R = 10;
			}
		}
		else{
			ff_accele_R = 0;
		}

		//pre_robot_speed = robot_speed;
		/*
		if(robot_speed + OFFSET > now_speed_L && robot_speed - OFFSET < now_speed_L){	//目標値のとき
			ff_accele_L = 0;
		}
		else if(robot_speed + OFFSET < now_speed_L){	//目標値より高い場合
			ff_accele_L = -10;
		}
		else if(robot_speed - OFFSET > now_speed_L) {//目標値より低い場合
			ff_accele_L = 10;
		}

		if(robot_speed + OFFSET > now_speed_R && robot_speed - OFFSET < now_speed_R){	//目標値のとき
			ff_accele_R = 0;
		}
		else if(robot_speed + OFFSET < now_speed_R){	//目標値より高い場合
			ff_accele_R = -10;
		}
		else if(robot_speed - OFFSET > now_speed_R) {//目標値より低い場合
			ff_accele_R = 10;
		}
		*/
		//ff_accele_R = 0;
		//ff_accele_L = 0;
		//robot_speed = target_robot_speed;

		/*
		if(robot_speed > target_robot_speed){
			robot_speed -= accele;
			if(robot_speed <= target_robot_speed) robot_speed = target_robot_speed ;
		}
		else if(robot_speed < target_robot_speed){
			robot_speed += accele;
			if(robot_speed >= target_robot_speed) robot_speed = target_robot_speed ;
		}
		else {
			robot_speed = target_robot_speed;
		}
		*/

	}

}

/************************************************************************/
//* 役割　：　ラインから外れてしばらくしたら緊急停止
//* 引数　：　void:
//* 戻り値：　short: 異常時1　通常0
//* 備考 : なし
//************************************************************************/
char off_line_check(char Flag){
	static char ret = 0;
	static int error_cnt = 0, reset_cnt = 0;

	if(Flag){
		if((Line2 > LINE_SENSOR_THRESHOLD) && (Line3 > LINE_SENSOR_THRESHOLD))	error_cnt++;

		if(error_cnt > 100)	ret = 1;

		if((reset_cnt > 5000) && ret == 0){
			reset_cnt = 0;
			error_cnt = 0;
		}
		reset_cnt++;
	}
	else{
		ret = 0;
		reset_cnt = 0;
		error_cnt = 0;
	}

	return ret;
}

//************************************************************************/
//* 役割　：　vcm異常検知したら停止させる
//* 引数　：　float: 現在のPWMパルス幅
//* 戻り値：　short: 異常時1　通常0
//* 備考 : なし
//************************************************************************/
char vcm_kill(char Flag, float PulseWidth){
	static char ret = 0;
	static int error_cnt = 0, reset_cnt = 0;

	if(Flag){
		if((Pot < ANGLE_LIMIT_LOW) && (PulseWidth >= VCM_MAX_RATIO - 50)){
			error_cnt += 1;
		}
		if((ANGLE_LIMIT_HIGH < Pot) && (PulseWidth >= VCM_MAX_RATIO - 50)){
			error_cnt += 1;
		}

		if(PulseWidth >= VCM_MAX_RATIO - 50){
			error_cnt += 1;
		}

		//if(error_cnt > 200) ret = 1;		//異常検知

		if((reset_cnt > 5000) && ret == 0){	//5秒ごとにリセット
			reset_cnt = 0;
			error_cnt = 0;
		}
		reset_cnt++;
	}
	else{
		ret = 0;
		reset_cnt = 0;
		error_cnt = 0;
	}

	return ret;
}

//************************************************************************/
//* 役割　：　error総合監視関数
//* 引数　：　short *: エラー番号
//* 戻り値：　short: 異常時1　通常0
//* 備考 :
//************************************************************************/
char error_check(char *error_num){
	char ret = 0;

	*error_num = 0;
	if(vcm_kill(flag.error_check_enable, monitoring_vcm_pulse_width)){
		//ret = 1;
		*error_num |= 0x01;
	}
	if(off_line_check(flag.error_check_enable)){
		ret = 1;
		*error_num |= 0x02;
	}

	//if(ret == 1)	LED('M');
	return ret;
}

//************************************************************************/
//* 役割　：　errorリセット
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 :
//************************************************************************/
void error_reset(){
	flag.error_check_enable = 0;

	vcm_kill(flag.error_check_enable, monitoring_vcm_pulse_width);
	off_line_check(flag.error_check_enable);

	flag.error_check_enable = 1;
	flag.error = 0;
}

//************************************************************************/
//* 役割　：　スピードセレクト
//* 引数　：　void:
//* 戻り値：　float: 速度[mm/s]
//* 備考 :
//************************************************************************/
float speed_select(){
	static float speed = 0;

	if(timer.lcd > LCD_WAIT){
		timer.lcd = 0;
		switch(R_SW()){
			//--------------------------------------------------------
				case 0:	//	記録後に走行

					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("SPEED");
					lcd_locate(0,1);
					lcd_printf("memory");

					speed = speed_table_distance[0];

					if(SW(1)){
						LED('B');

						create_speed_table_func();
						flag.speed_updata = 1;

						HAL_Delay(1000);
					}
					if(SW(2)){
						LED('M');


						create_speed_table_func_2();
						flag.speed_updata = 1;

						HAL_Delay(1000);
						/*
						for(short i = 0; i < max_access; i++){
							printf("%d		%f\r\n", i, speed_table[i]);
						}
						*/
					}
					/*
					if(flag.speed_updata)	LED('M');
					else LED('C');
					*/
					LED('C');
				break;
				/*
				case 1:
					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("        ");
					lcd_locate(0,1);
					lcd_printf("        ");

					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("SPEED");
					lcd_locate(0,1);
					lcd_printf("600mm/s");

					speed = 600;

				break;
				*/

				//--------------------------------------------------------
				case 2:	//引継ぎ走行
					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("SPEED");
					lcd_locate(0,1);
					lcd_printf("1400mm/s");

					speed = speed_table_distance[0];
					if(SW(1)){	//安全
						LED('B');
						sd_read_array_int("Take_over", "side_line_memory_L.txt", SIDE_LINE_MEMORY_SIZE, side_line_memory_L);
						sd_read_array_int("Take_over", "side_line_memory_R.txt", SIDE_LINE_MEMORY_SIZE, side_line_memory_R);
						sd_read_array("Take_over", "radius.txt", MEMORY_ARRAY_SIZE_DISTANCE, imu_radius_memory_distance);
						/*
						sd_write_array_int("check", "side_line_memory_L.txt", SIDE_LINE_MEMORY_SIZE, side_line_memory_L, OVER_WRITE);
						sd_write_array_int("check", "side_line_memory_R.txt", SIDE_LINE_MEMORY_SIZE, side_line_memory_R, OVER_WRITE);
						sd_write_array("check", "radius.txt", MEMORY_ARRAY_SIZE_DISTANCE, imu_radius_memory_distance, OVER_WRITE);
						*/

						create_speed_table_func();
						//sd_write_array("check", "speed_table.txt", MEMORY_ARRAY_SIZE_DISTANCE, speed_table_distance, OVER_WRITE);
						flag.speed_updata = 1;

						HAL_Delay(1000);
						/*
						for(short i = 0; i < max_access; i++){
							printf("%d		%f\r\n", i, speed_table[i]);
						}
						*/
					}
					if(SW(2)){	//速い
						LED('M');
						sd_read_array_int("Take_over", "side_line_memory_L.txt", SIDE_LINE_MEMORY_SIZE, side_line_memory_L);
						sd_read_array_int("Take_over", "side_line_memory_R.txt", SIDE_LINE_MEMORY_SIZE, side_line_memory_R);
						sd_read_array("Take_over", "radius.txt", MEMORY_ARRAY_SIZE_DISTANCE, imu_radius_memory_distance);

						/*
						sd_write_array_int("check", "side_line_memory_L.txt", SIDE_LINE_MEMORY_SIZE, side_line_memory_L, OVER_WRITE);
						sd_write_array_int("check", "side_line_memory_R.txt", SIDE_LINE_MEMORY_SIZE, side_line_memory_R, OVER_WRITE);
						sd_write_array("check", "radius.txt", MEMORY_ARRAY_SIZE_DISTANCE, imu_radius_memory_distance, OVER_WRITE);
						*/

						create_speed_table_func_2();
						flag.speed_updata = 1;

						HAL_Delay(1000);

					}
					LED('R');

				break;
				/*
				case 3:
					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("        ");
					lcd_locate(0,1);
					lcd_printf("        ");

					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("SPEED");
					lcd_locate(0,1);
					lcd_printf("800mm/s");

					speed = 800;

				break;
				*/
				//--------------------------------------------------------
				case 4:	//	読み出し走行
					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("SPEED");
					lcd_locate(0,1);
					lcd_printf("1400mm/s");

					speed = speed_table_distance[0];
					if(SW(1)){	//long
						LED('M');

						sd_read_array("replay", "radius_long.txt", MEMORY_ARRAY_SIZE_DISTANCE, imu_radius_memory_distance);
						create_speed_table_func();
						flag.speed_updata = 1;

						HAL_Delay(1000);
						/*
						for(short i = 0; i < max_access; i++){
							printf("%d		%f\r\n", i, speed_table[i]);
						}
						*/
					}
					if(SW(2)){	//short
						LED('M');

						sd_read_array("replay", "radius_long2.txt", MEMORY_ARRAY_SIZE_DISTANCE, imu_radius_memory_distance);
						sd_read_array_int("replay", "side_line_memory_L_long2.txt", SIDE_LINE_MEMORY_SIZE, side_line_memory_L);
						sd_read_array_int("replay", "side_line_memory_R_long2.txt", SIDE_LINE_MEMORY_SIZE, side_line_memory_R);

						create_speed_table_func();
						flag.speed_updata = 1;

						HAL_Delay(1000);
						/*
						for(short i = 0; i < max_access; i++){
							printf("%d		%f\r\n", i, speed_table[i]);
						}
						*/
					}
					LED('G');
				break;
				/*
				case 5:
					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("        ");
					lcd_locate(0,1);
					lcd_printf("        ");

					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("SPEED");
					lcd_locate(0,1);
					lcd_printf("1000mm/s");

					speed = 1000;

				break;
				*/
				//--------------------------------------------------------
				case 6:	//	記録
					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("SPEED");
					lcd_locate(0,1);
					lcd_printf("1400mm/s");

					speed = LOW_SPEED_DISTANCE;

					flag.course_memory_distance = 1;	//	ライン記憶enable

					LED('B');

				break;
				/*
				case 7:
					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("        ");
					lcd_locate(0,1);
					lcd_printf("        ");

					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("SPEED");
					lcd_locate(0,1);
					lcd_printf("1200mm/s");

					speed = 1200;

				break;
				*/
				/*
				case 8:
					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("        ");
					lcd_locate(0,1);
					lcd_printf("        ");

					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("SPEED");
					lcd_locate(0,1);
					lcd_printf("1300mm/s");

					speed = 1300;

				break;
				*/
				/*
				case 9:
					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("        ");
					lcd_locate(0,1);
					lcd_printf("        ");

					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("SPEED");
					lcd_locate(0,1);
					lcd_printf("1400mm/s");

					speed = 1400;

				break;
				*/
				/*
				case 10:
					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("        ");
					lcd_locate(0,1);
					lcd_printf("        ");

					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("SPEED");
					lcd_locate(0,1);
					lcd_printf("1500mm/s");

					speed = 1500;

				break;
				*/
				/*
				case 11:
					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("        ");
					lcd_locate(0,1);
					lcd_printf("        ");

					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("SPEED");
					lcd_locate(0,1);
					lcd_printf("1600mm/s");

					speed = 1600;

				break;
				*/
				/*
				case 12:
					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("        ");
					lcd_locate(0,1);
					lcd_printf("        ");

					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("SPEED");
					lcd_locate(0,1);
					lcd_printf("1500mm/s");

					speed = 1500;

				break;
				*/
				/*
				case 13:
					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("        ");
					lcd_locate(0,1);
					lcd_printf("        ");

					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("SPEED");
					lcd_locate(0,1);
					lcd_printf("1600mm/s");

					speed = 1600;

				break;
				*/
				/*
				case 14:
					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("        ");
					lcd_locate(0,1);
					lcd_printf("        ");

					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("SPEED");
					lcd_locate(0,1);
					lcd_printf("1700mm/s");

					speed = 1700;

				break;
				*/
				/*
				case 15:
					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("        ");
					lcd_locate(0,1);
					lcd_printf("        ");

					lcd_clear();
					lcd_locate(0,0);
					lcd_printf("SPEED");
					lcd_locate(0,1);
					lcd_printf("1000mm/s");

					speed = 1000;

				break;
				*/
				default:
					//LED('N');
				break;

			}
		}
	return speed;
}

//************************************************************************/
//* 役割　：　汎用タイマーインクリメント
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 : 1msごとにインクリメントされる
//************************************************************************/
void increment_mytimer(){
	timer.lcd++;
	timer.my_timer1++;
	timer.my_timer2++;
	timer.imu_timer++;
	timer.pot_timer++;
	timer.check_timer++;
	timer.log++;
	timer.distance++;
	timer.correction++;
	timer.spi++;
	timer.mtim++;

}

//************************************************************************/
//* 役割　：　フルカラーLEDを順番にチカチカする
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 : 呼び出す周期によって色が変わる周期もかわる
//************************************************************************/
void full_color(){
	static int cnt = 0;
	cnt++;
	if(cnt > 1000){
		LED('R');
		cnt = 0;
	}
	else if(cnt > 830){
		LED('G');
	}
	else if(cnt > 664){
		LED('B');
	}
	else if(cnt > 498){
		LED('M');
	}
	else if(cnt > 332){
		LED('Y');
	}
	else if(cnt > 166){
		LED('C');
	}
	else if(cnt > 0){
		LED('W');
	}
}

//************************************************************************/
//* 役割　：　フルカラーLED制御
//* 引数　：　char: 'R'(Red), 'G'(Green), 'B'(Blue), 'M'(Magenta), 'Y'(Yellow), 'C'(Cyan), 'W'(White), 'O'(Off)
//* 戻り値：　void:
//* 備考 : よくわからない引数がきたら消灯
//************************************************************************/
void LED(char state){

	switch(state){
		case 'R':	//RED
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,  GPIO_PIN_RESET);		//R
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);		//G
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);		//B
		break;

		case 'G':	//GREEN
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,  GPIO_PIN_SET);
		break;

		case 'B':	//BLUE
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,  GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		break;

		case 'Y':	//YELLOW
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,  GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		break;

		case 'M':	//MAZENTA
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,  GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		break;

		case 'C':	//CYAN
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,  GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		break;

		case 'W':	//WHITE
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,  GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		break;

		case 'O': //Off
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,  GPIO_PIN_SET);
		break;

		default:	//よくわからないのがきたら消灯
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,  GPIO_PIN_SET);
		break;

	}
}

//************************************************************************/
//* 役割　：　タクトスイッチ処理
//* 引数　：　short: スイッチの番号 1, 2, 3
//* 戻り値：　short: 0 or 1
//* 備考 : よくわからない引数が来たら戻り値は0
//************************************************************************/
char SW(char num){
	char ret = 0;
	//static short sw_flag1, sw_flag2, sw_flag3;
  //static int sw_cnt;

	switch(num){
		case 1:
			if(!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)) ret = 1;
		break;

		case 2:
			if(!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)) ret = 1;
		break;

		case 3:
			if(!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)) ret = 1;
		break;
	}

	return ret;

}

//************************************************************************/
//* 役割　：　ロータリスイッチ処理
//* 引数　：　void:
//* 戻り値：　char: 0~15
//* 備考 : なし
//************************************************************************/
char R_SW(){
	char r_sw = 0;

	if(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1))	r_sw |= 0x01;
	if(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3))	r_sw |= 0x02;
	if(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0))	r_sw |= 0x04;
	if(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2))	r_sw |= 0x08;


	return r_sw;
}

//************************************************************************/
//* 役割　：　左モータの1制御周期間のエンコーダ値を返す
//* 引数　：　void:
//* 戻り値：　int:　エンコーダのカウント
//* 備考 : なし
//************************************************************************/
int getEncorder_L(){	//1周期分のカウントを返す
	return (TIM5 -> CNT)- ENCORDER_OFFSET;
}

//************************************************************************/
//* 役割　：　右モータの1制御周期間のエンコーダ値を返す
//* 引数　：　void:
//* 戻り値：　int:　エンコーダのカウント
//* 備考 : なし
//************************************************************************/
int getEncorder_R(){
	return (60000 - (TIM4 -> CNT)) - ENCORDER_OFFSET;
}

//************************************************************************/
//* 役割　：　エンコーダの積算値を1制御周期ごとにリセットする
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 : getEncorder_L()とかのまえで宣言しちゃうと永遠にエンコーダ値が0になっちゃう
//************************************************************************/
void enc_reset(){	//1周期でリセット
	TIM5->CNT = ENCORDER_OFFSET;
	TIM4->CNT = ENCORDER_OFFSET;
}

//************************************************************************/
//* 役割　：　1制御周期分のエンコーダ値を足してエンコーダ値のトータルを更新
//* 引数　：　long *, long *:　更新したい変数のポインタ
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
void updata_enc_cnt(int*encL, int *encR, int*encL_memory, int *encR_memory, int32_t *encL_distance, int32_t *encR_distance, int *encL_reset, int *encR_reset){
	//static unsigned short pre_L, pre_R;	//オーバーフロー対策

	*encL += getEncorder_L();
	*encR += getEncorder_R();
	*encL_memory += getEncorder_L();
	*encR_memory += getEncorder_R();
	*encL_distance += getEncorder_L();
	*encR_distance += getEncorder_R();
	*encL_reset += getEncorder_L();
	*encR_reset += getEncorder_R();

	/*　//オーバーフロー対策
	if(*encL - pre_L < -10000) enc_overL++;
	else if(*encL - pre_L > 10000) enc_overL--;

	pre_L = *encL;
	pre_R	= *encR;
	*/
}

//************************************************************************/
//* 役割　：　Maxonモータコントロール
//* 引数　：　float, float: モータに入力する値（-MAXON_MAX_RATIO ~  MAXON_MAX_RATIO）
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
void maxon_ctrl(float power_maxonL, float power_maxonR){
	float PulseWidth_maxonL = 0, PulseWidth_maxonR = 0;   //PWMパルス

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);	//SR set
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);	//SR set

	if(power_maxonL < 0){	//maxonL
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);	//motor dir
		PulseWidth_maxonL = power_maxonL * -1;
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);	//motor_ctrl dir
		PulseWidth_maxonL = power_maxonL;
	}
	if(power_maxonR < 0){	//maxonR
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);	//motor dir
		PulseWidth_maxonR = power_maxonR * -1;
	}
	else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);	//motor_ctrl dir
		PulseWidth_maxonR = power_maxonR;
	}


	if(PulseWidth_maxonL > MAXON_MAX_RATIO)	PulseWidth_maxonL = MAXON_MAX_RATIO;
	if(PulseWidth_maxonR > MAXON_MAX_RATIO)	PulseWidth_maxonR = MAXON_MAX_RATIO;

	if(flag.error == 1){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PulseWidth_maxonR);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PulseWidth_maxonL);
	}

}

//************************************************************************/
//* 役割　：　vcmコントロール
//* 引数　：　float, float: モータに入力する値（-VCM_MAX_RATIO ~  VCM_MAX_RATIO）, パルス幅監視用ポインタ
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
void vcm_ctrl(float power_vcm, float *now_pulsewidth){
	float PulseWidth_vcm = 0;   //PWMパルス

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);	//SR set

	if(power_vcm < 0){	//vcm
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);	//motor dir
		PulseWidth_vcm = power_vcm * -1;
	}
	else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);	//motor_ctrl dir
		PulseWidth_vcm = power_vcm;
	}

	if(PulseWidth_vcm > VCM_MAX_RATIO)	PulseWidth_vcm = VCM_MAX_RATIO;

	if(flag.error == 1){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PulseWidth_vcm);
	}

	*now_pulsewidth = PulseWidth_vcm;
}


//************************************************************************/
//* 役割　：　AD値を更新
//* 引数　：　void
//* 戻り値：　void:
//* 備考 : グローバル変数を更新
//************************************************************************/
void updata_ADval(){

	static float pre_pot = 0;

	Line1 = ad1[0];
	Line2 = ad1[1];
	Line3 = ad1[2];
	Line4 = ad1[3];
/*
	Line1 = average_sens1;
	Line2 = average_sens2;
	Line3 = average_sens3;
	Line4 = average_sens4;
*/
	SideL = ad1[5];
	SideR = ad1[4];


	//Pot	= 2048 - ad1[6];
	Pot = average_pot;
	/*
	Pot = lowpass_filter_simple(2048 - ad1[6], pre_pot, 1);
	pre_pot = Pot;
	*/
	Def_ref = (2048 - ad1[6]);


	//移動平均分散用配列に格納]
	/*
	radius_memory[retention_access] = Pot;
	retention_access++;
	if(retention_access >= MEMORY_ARRAY_SIZE)	retention_access = 0;
	*/

}

//************************************************************************/
//* 役割　：　printfでteratermにUARTで表示するのに必要(IDEから)
//* 引数　：　よくわからない
//* 戻り値：　よくわからない
//* 備考 : よくわからない
//************************************************************************/
/*
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1,(uint8_t *)ptr,len,10);
  return len;
}
*/
//************************************************************************/
//* 役割　：　printfでteratermにUARTで表示するのに必要
//* 引数　：　よくわからない
//* 戻り値：　よくわからない
//* 備考 : よくわからない
//************************************************************************/
PUTCHAR_PROTOTYPE {
 HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
 return ch;
}

//************************************************************************/
//* 役割　：　ADC変換後のコールバック関数
//* 引数　：　よくわからない
//* 戻り値：　よくわからない
//* 備考 : よくわからない
//************************************************************************/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {	//ADC後に呼ばれる


}

void RotationSpeed(char enable, float *val){
	if(enable){
		float input = 0;
		float kp = -218.8515, ki  = -4309.9877, kd = 0;
		float p = 0, d = 0;
		static float i = 0;
		float devi = 0;
		static float pre_devi = 0;

		devi = TargetOmega - omega_z;

		p = kp * devi;

		d = (float)(kd * (devi - pre_devi) / DELTA_T);

		i += ki * DELTA_T * devi;

		input = p + d + i;

		pre_devi = devi;

		*val = input;

	}

}
void TranslationSpeed(char enable, float *val){
	float kp = 0.73598, ki = 3.7518, kd = 0;	//float kp = 1.7, ki = 140, kd = 0;
	float p = 0, d = 0;;
	static float i = 0;

	float _now_speed_L = 0, _now_speed_R = 0;
	float devi = 0;

	float now_speed = 0;

	if(enable){

		_now_speed_L = MM_PER_PULS * 1000. * getEncorder_L();		//[mm/s]
		_now_speed_R = MM_PER_PULS * 1000. * getEncorder_R(); 		//[mm/s]

		now_speed = (_now_speed_L + _now_speed_R) / 2;

		devi = robot_speed - now_speed;
		p = devi * kp;
		i += devi * DELTA_T * ki;

		*val = p + i ;

	}

}

void BanquetArt(char enable){
	if(enable){
		float input = 0;
		float kp = -218.8515, ki  = -4309.9877, kd = 0;
		float p = 0, d = 0;
		static float i = 0;
		float devi = 0;
		static float pre_devi = 0;

		devi = TargetOmega - omega_z;

		p = kp * devi;

		d = (float)(kd * (devi - pre_devi) / DELTA_T);

		i += ki * DELTA_T * devi;

		input = p + d + i;

		maxon_ctrl(input, -input);

		pre_devi = devi;

	}
}



/* Function Definitions */

/*
 * Arguments    : double result[2]
 * Return Type  : void
 */
static void argInit_1x2_real_T(double result[2])
{
  double result_tmp;

  /* Loop over the array to initialize each element. */
  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result_tmp = argInit_real_T();
  result[0] = result_tmp;

  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result[1] = result_tmp;
}

/*
 * Arguments    : double result[3]
 * Return Type  : void
 */
static void argInit_3x1_real_T(double result[3])
{
  double result_tmp;

  /* Loop over the array to initialize each element. */
  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result_tmp = argInit_real_T();
  result[0] = result_tmp;

  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result[1] = result_tmp;

  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result[2] = argInit_real_T();
}

/*
 * Arguments    : double result[9]
 * Return Type  : void
 */
static void argInit_3x3_real_T(double result[9])
{
  int idx0;
  double result_tmp;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 3; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result_tmp = argInit_real_T();
    result[idx0] = result_tmp;

    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0 + 3] = result_tmp;

    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0 + 6] = argInit_real_T();
  }
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_GetSelfLocation(double velo[2])
{
  static uint16_t access;
  static double PrePosition[3];
  static double PrePt[9];
  static double PreZt;
  //double velo[2];
  double EstPosition[3];
  double EstPt[9];
  double ObsZt;

  /* Initialize function 'GetSelfLocation' input arguments. */
  /* Initialize function input argument 'PrePosition'. */
  /* Initialize function input argument 'PrePt'. */
  /* Initialize function input argument 'velo'. */
  /* Call the entry-point 'GetSelfLocation'. */
  argInit_3x1_real_T(PrePosition);
  argInit_3x3_real_T(PrePt);
  argInit_1x2_real_T(velo);

  GetSelfLocation(PrePosition, PrePt, PreZt, velo, EstPosition, EstPt, &ObsZt);

  various_memory1[access] = EstPosition[0];
  various_memory2[access] = EstPosition[1];
  various_memory3[access] = EstPosition[2];
  access++;
  if(access >= MEMORY_ARRAY_SIZE_2 - 1) access = MEMORY_ARRAY_SIZE_2 - 1;

  memcpy(PrePosition, EstPosition, sizeof(EstPosition));
  memcpy(PrePt, EstPt, sizeof(EstPt));
  PreZt = ObsZt;

}

/********Following*********/
void rt_OneStep(void)
{
  static boolean_T OverrunFlag = false;

  /* Disable interrupts here */

  /* Check for overrun */
  if (OverrunFlag) {
    rtmSetErrorStatus(rtM, "Overrun");
    return;
  }

  OverrunFlag = true;

  /* Save FPU context here (if necessary) */
  /* Re-enable timer or interrupt here */
  /* Set model inputs here */

  /* Step the model */
  PathFollowing_step();

  /* Get model outputs here */

  /* Indicate task complete */
  OverrunFlag = false;

  /* Disable interrupts here */
  /* Restore FPU context here (if necessary) */
  /* Enable interrupts here */
}
