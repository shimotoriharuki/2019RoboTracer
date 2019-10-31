
# ifdef __GNUC__
 #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
# else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
# endif /*__GNUC__*/

//#define REVERCE_RUN	//逆走するときONにする
 
 
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
#define LINE_DIGITAL_THRESHOLD 500		//デジタル変換用
#define SIDE_DIGITAL_THRESHOLD_LEFT 500		//デジタル変換用
#define SIDE_DIGITAL_THRESHOLD_RIGHT 400		//デジタル変換用

#define START_ACCELERATION 100 			//[m/ss]	5
#define NORMAL_ACCELERATION 100		//[m/ss]
#define CONTROL_CYCLE 0.5 //[ms]Control cycle

#define CHAR_MAX 127
#define CHAR_MIN -128
#define INT_MAX 2147483647
#define INT_MIN -2147483648
#define LONG_MAX 4294967295

#define DELTA_T (float)0.0005
#define LCD_WAIT 100
#define DATA_SIZE1 7	//adの配列数
#define DATA_SIZE2 3
#define DEF_ARRAY_SIZE 128 //デフ格納配列の要素数
#define MEMORY_ARRAY_SIZE 200 
#define RETENTION_ARRAY_SIZE 1000
#define DECELERATION_DISTANCE 500 //[mm]//500
#define CONTINUED_DISTANCE 60

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
#define LOW_SPEED 1450	//1450

#define HIGH_SPEED_2 6000	//6000
#define COMM_SPEED_2 2000	//2000
#define LOW_SPEED_2 1600	//1450

#define ROTATION_POINT_FROM_AXEL 25
#define TRED 112
#define AD_RESOLUTION 2048 //-2048-2048
#define DEG160_TO_RAD 2.792526803
#define RAD_PER_AD (DEG160_TO_RAD / AD_RESOLUTION)
#define TEST_SIZE 1

#define COUNT_TO_RECORD 56.19556055 //[mm]
#define R 0.8

#define MEMORY_ARRAY_SIZE_2 6000

//SD file name

#define ENCORDER_LOG 0
#define IMU_LOG 1
#define POT_LOG 2
#define LINE_SENSOR_LOG 3

#define FOLDER_0 '0'
#define FOLDER_1 '1'
#define FOLDER_2 '2'

#define ENCORDER_LOG_TXT "encorder_log.txt"
#define IMU_LOG_TXT "imu_log.txt"
#define POT_LOG_TXT "pot_log.txt"
#define LINE_SENSOR_LOG_TXT "line_sensor_log.txt"

//


#define OVER_WRITE 0	//上書き
#define ADD_WRITE 1		//追加書き

