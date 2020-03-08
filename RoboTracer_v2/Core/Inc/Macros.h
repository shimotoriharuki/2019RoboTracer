
# ifdef __GNUC__
 #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
# else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
# endif /*__GNUC__*/

//#define REVERCE_RUN	//逆走するときONにする
 
#define PI 3.1415926535
 
#define STOP_CNT 5000
#define	MAXON_MAX_RATIO 1200
#define	VCM_MAX_RATIO 1200
#define ANGLE_LIMIT_HIGH 100	//438
#define ANGLE_LIMIT_LOW -100	//-300
#define ENC_LIMIT 65535
#define ENCORDER_OFFSET 30000
#define MM_PER_PULS 0.017795715859364		//1パルスで進む距離[mm] (タイヤ1周で進タ距離 / タイヤ1周のパルス数)	0.017795715859364	1mm進んだ時のカウント　56.19556055
#define MM19_PER_MPP 1200				//19[mm]で来るパルス数　　(19 / MM_PER_PULS = 1089.693641)	1090
#define MM19_PER_MPP2 2500				//19[mm]で来るパルス数　　(19 / MM_PER_PULS = 1089.693641)
#define MM19_PER_MPP_PLUS 2000				//19[mm]で来るパルス数　多め
#define MM100_PER_MPP 8000				//だいたい100+α[mm]
#define MM500_PER_MPP 28000				//ゴール後進む距離のパルス だいたい500+α[mm]
//#define RAD_PER_AD 0.001363538
#define LINE_SENSOR_THRESHOLD 500		//緊急停止用
#define LINE_DIGITAL_THRESHOLD 500		//	デジタル変換用
#define SIDE_DIGITAL_THRESHOLD_LEFT 300		//	デジタル変換用
#define SIDE_DIGITAL_THRESHOLD_RIGHT 350		//	デジタル変換用

#define START_ACCELERATION 100 			//[m/ss]	5
#define NORMAL_ACCELERATION 100		//[m/ss]
#define CONTROL_CYCLE 0.5 //[ms]Control cycle

#define CHAR_MAX 127
#define CHAR_MIN -128
#define INT_MAX 2147483647
#define INT_MIN -2147483648
#define LONG_MAX 4294967295

#define DELTA_T (float)0.001
#define LCD_WAIT 100
#define DATA_SIZE1 7	//adの配列数
#define DATA_SIZE2 3
#define DEF_ARRAY_SIZE 128 //デフ格納配列の要素数
#define MEMORY_ARRAY_SIZE 0
#define MEMORY_ARRAY_SIZE_2 4100
#define MEMORY_ARRAY_SIZE_DISTANCE 0	//6000
#define SIDE_LINE_MEMORY_SIZE 600	//600

#define RETENTION_ARRAY_SIZE 0
#define DECELERATION_DISTANCE 500 //[mm]//500
#define CONTINUED_DISTANCE 60

#define ANGLE_ARRAY_DATA 1000

#define GRAVITY_ACCELERATION 2000	//imuからくる生の値

#define ONE_RAD_PER_S 64300				//1[rad/s]のときimuからくる生の角速度の値
#define FIVE_RAD_PER_S 61000			//5[rad/s]のときimuからくる生の角速度の値
#define TEN_RAD_PER_S 57000				//10[rad/s]のときimuからくる生の角速度の値

#define ONE_ROTATION_COUNT 19772	//機体が1回転するカウント

#define FREQ 3
#define SAMPLERATE 500
#define Q 1

#define HIGH_SPEED 6000	//6000
#define COMM_SPEED 2000	//2000
#define LOW_SPEED 1400	//1450

#define HIGH_SPEED_2 6000	//6000
#define COMM_SPEED_2 2000	//2000
#define LOW_SPEED_2 1600	//1450

#define HIGH_SPEED_DISTANCE 6000	//6000
#define COMM_H_SPEED_DISTANCE 3000	//2000	R30
#define COMM_L_SPEED_DISTANCE 2000	//2000	R15-R20
#define LOW_SPEED_DISTANCE 1000	//1450	R10

#define ROTATION_POINT_FROM_AXEL 39.5	//39.5 ~ 51.5 45.5よさげ 　33.5 だめ
#define TRED 112.	//[mm]112 124
#define AD_RESOLUTION 2048 //-2048-2048
#define DEG160_TO_RAD 2.792526803
#define RAD_PER_AD ((DEG160_TO_RAD / AD_RESOLUTION) * 1)
#define TEST_SIZE 1

#define COUNT_TO_RECORD 562 //		10[mm]:561.9556055 		20[mm]:1123.91121
#define R 0.07

#define MAX_ACC 1000 	//[mm/ss]
#define MAX_DEC 200 		//[mm/ss]

#define MAX_ACC_2 9000 	//[mm/ss]	1000
#define MAX_DEC_2 250 		//[mm/ss]	200

#define MACHINE_WEIGHT 0.205	//[kg]
#define R_TIRE 0.011	//[m]
#define GIRE_RATIO 1.9355
#define KM_L 0.002120	//[Nm/A]	//0.00352
#define KE_L 2168		//[rpm/V]	2710 2168
#define Ke_L (1/KE_L)	//[V/rpm]
#define KM_R 0.002120	//[Nm/A]	//0.00352
#define KE_R 2168		//[rpm/V]	2710 2168
#define Ke_R (1/KE_R)	//[V/rpm]
#define MOTOR_RESISTANCE 2.9	//[ohm]
#define MAX_DUTY 1200	//V_MAX : 1200
#define INPUT_VOLTAGE 8	//[V]
#define WHEEL_D 0.22	//[m]

#define M_LEN 50


//-----------------------------------------------------//
//SD folder name
#define FOLDER_0 "folder_0"
#define FOLDER_1 "folder_1"
#define FOLDER_2 "folder_2"
#define FOLDER_3 "folder_3"
#define FOLDER_4 "folder_4"
#define FOLDER_5 "folder_5"
#define FOLDER_6 "folder_6"
#define FOLDER_7 "folder_7"
#define FOLDER_8 "folder_8"
#define FOLDER_9 "folder_9"
#define FOLDER_10 "folder_10"
#define FOLDER_TEST "folder_test"
#define RUNNINNG_LOG "running_log"

//SD file name
#define ENCORDER_LOG_TXT "encorder_log.txt"
#define IMU_LOG1_TXT "imu_log1.txt"
#define IMU_LOG2_TXT "imu_log2.txt"
#define POT_LOG_TXT "pot_log.txt"
#define VELO_LOG_TXT "velo_log1.txt"
#define LINE_SENSOR_LOG_TXT "line_sensor_log.txt"
#define OTHER1_TXT "other1.txt"
#define OTHER2_TXT "other2.txt"
#define OTHER3_TXT "other3.txt"
#define OTHER4_TXT "other4.txt"
#define OTHER5_TXT "other5.txt"
#define OTHER6_TXT "other6.txt"


#define OVER_WRITE 0	//	上書き
#define ADD_WRITE 1		//	追加書き

//-----------------------------------------------------//




