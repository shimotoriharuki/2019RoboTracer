#include "main.h"
#include "Macros.h"

typedef struct {
	char line_trace;
	char following;
	char hand_push;
	char error;
	char run;
	char ready;
	char angle;
	char following_start;
	char error_check_enable;
	char curve;
	char straight;
	char curve_to_straight;
	char speed_ctrl;
	char acc;
	char imu_store;
	char imu_store_2;
	char pot_store;
	char speed_updata;
	char side;
	char course_memory;
	char runnning_reset;
	char enc_memory_reset;
	char record_running_enable;
	char mesurment_reset;
	char trace;
	char retention_reset;
	char updata_robot_speed_reset;
	char speed_ctrl_enable;
	char sd_record;
}Flag;

typedef struct{
	short lcd;
	short my_timer1;
	short my_timer2;
	short imu_timer;
	short pot_timer;
	short check_timer;
	
}Timer;

typedef struct{
	short Linepot_timer_L;
	short Line_R;
	short Side_L;
	short Side_R;
}Digital;

typedef struct{
	short sample1[5];
	short sample2;
}sample;

typedef struct{
	float kp;
	float ki;
	float kd;
}pid_parameter;

#ifdef DEF_EXTERN
/* 自動生成変数(CubeMXから変更を加えた場合変える必要があるかも) -------------------------------------------------------------*/

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* ----------- -------------------------------------------------------------*/

char error_number = 0;
uint32_t ad1[DATA_SIZE1];
uint32_t ad2[DATA_SIZE2];
int Line1, Line2, Line3, Line4, SideL, SideR, Pot;
int Def_ref;
int total_encL, total_encR;
int total_encL_memory, total_encR_memory;
short total_encL_distance, total_encR_distance;
signed char enc_overL, enc_overR;

int cycle_encL, cycle_encR;

float robot_speed;
float target_robot_speed;
float speed_L, speed_R;
float Def_speed_L, Def_speed_R;

/*ポテンショ512の分解能で計算して128までを使う*/
float L_def[DEF_ARRAY_SIZE] = {1.0, 1.012214, 1.024430, 1.036646, 1.048865, 1.061087, 1.073312, 1.085542, 1.097777, 1.110018, 1.122265, 1.134520, 1.146782, 1.159053, 1.171334, 1.183625, 1.195928, 1.208241, 1.220568, 1.232907, 1.245261, 1.257629, 1.270013, 1.282414, 1.294831, 1.307266, 1.319720, 1.332193, 1.344686, 1.357201, 1.369737, 1.382296, 1.394878, 1.407484, 1.420115, 1.432773, 1.445457, 1.458168, 1.470908, 1.483678, 1.496477, 1.509307, 1.522169, 1.535064, 1.547993, 1.560956, 1.573955, 1.586990, 1.600062, 1.613173, 1.626323, 1.639513, 1.652744, 1.666017, 1.679334, 1.692694, 1.706100, 1.719552, 1.733051, 1.746598, 1.760195, 1.773842, 1.787541, 1.801293, 1.815098, 1.828958, 1.842874, 1.856847, 1.870879, 1.884971, 1.899123, 1.913338, 1.927616, 1.941958, 1.956367, 1.970843, 1.985387, 2.000002, 2.014687, 2.029446, 2.044279, 2.059187, 2.074172, 2.089236, 2.104381, 2.119607, 2.134916, 2.150310, 2.165790, 2.181359, 2.197018, 2.212768, 2.228611, 2.244550, 2.260585, 2.276719, 2.292954, 2.309292, 2.325734, 2.342282, 2.358939, 2.375707, 2.392587, 2.409583, 2.426695, 2.443927, 2.461280, 2.478757, 2.496361, 2.514093, 2.531957, 2.549955, 2.568089, 2.586361, 2.604776, 2.623335, 2.642042, 2.660899, 2.679909, 2.699075, 2.718400, 2.737888, 2.757541, 2.777364, 2.797359, 2.817529, 2.837879, 2.858412};
float R_def[DEF_ARRAY_SIZE] = {1.0, 0.987786, 0.975570, 0.963354, 0.951135, 0.938913, 0.926688, 0.914458, 0.902223, 0.889982, 0.877735, 0.865480, 0.853218, 0.840947, 0.828666, 0.816375, 0.804072, 0.791759, 0.779432, 0.767093, 0.754739, 0.742371, 0.729987, 0.717586, 0.705169, 0.692734, 0.680280, 0.667807, 0.655314, 0.642799, 0.630263, 0.617704, 0.605122, 0.592516, 0.579885, 0.567227, 0.554543, 0.541832, 0.529092, 0.516322, 0.503523, 0.490693, 0.477831, 0.464936, 0.452007, 0.439044, 0.426045, 0.413010, 0.399938, 0.386827, 0.373677, 0.360487, 0.347256, 0.333983, 0.320666, 0.307306, 0.293900, 0.280448, 0.266949, 0.253402, 0.239805, 0.226158, 0.212459, 0.198707, 0.184902, 0.171042, 0.157126, 0.143153, 0.129121, 0.115029, 0.100877, 0.086662, 0.072384, 0.058042, 0.043633, 0.029157, 0.014613, -0.000002, -0.014687, -0.029446, -0.044279, -0.059187, -0.074172, -0.089236, -0.104381, -0.119607, -0.134916, -0.150310, -0.165790, -0.181359, -0.197018, -0.212768, -0.228611, -0.244550, -0.260585, -0.276719, -0.292954, -0.309292, -0.325734, -0.342282, -0.358939, -0.375707, -0.392587, -0.409583, -0.426695, -0.443927, -0.461280, -0.478757, -0.496361, -0.514093, -0.531957, -0.549955, -0.568089, -0.586361, -0.604776, -0.623335, -0.642042, -0.660899, -0.679909, -0.699075, -0.718400, -0.737888, -0.757541, -0.777364, -0.797359, -0.817529, -0.837879, -0.858412};

//float L_def[DEF_ARRAY_SIZE] = {1, 1.009545, 1.019090, 1.028634, 1.038179, 1.047725, 1.057270, 1.066815, 1.076361, 1.085907, 1.095454, 1.105000, 1.114547, 1.124095, 1.133643, 1.143192, 1.152741, 1.162290, 1.171840, 1.181391, 1.190943, 1.200495, 1.210048, 1.219602, 1.229156, 1.238712, 1.248268, 1.257825, 1.267383, 1.276943, 1.286503, 1.296064, 1.305627, 1.315190, 1.324755, 1.334321, 1.343888, 1.353456, 1.363026, 1.372597, 1.382170, 1.391744, 1.401319, 1.410896, 1.420474, 1.430054, 1.439636, 1.449219, 1.458804, 1.468391, 1.477979, 1.487569, 1.497161, 1.506755, 1.516351, 1.525949, 1.535548, 1.545150, 1.554754, 1.564359, 1.573967, 1.583577, 1.593190, 1.602804, 1.612421, 1.622040, 1.631661, 1.641285, 1.650911, 1.660539, 1.670170, 1.679804, 1.689440, 1.699078, 1.708720, 1.718364, 1.728010, 1.737660, 1.747312, 1.756967, 1.766625, 1.776285, 1.785949, 1.795616, 1.805285, 1.814958, 1.824633, 1.834312, 1.843994, 1.853679, 1.863368, 1.873059, 1.882754, 1.892452, 1.902154, 1.911859, 1.921567, 1.931279, 1.940995, 1.950714, 1.960437, 1.970163, 1.979893, 1.989626, 1.999364, 2.009105, 2.018850, 2.028599, 2.038352, 2.048109, 2.057869, 2.067634, 2.077403, 2.087176, 2.096953, 2.106734, 2.116520, 2.126310, 2.136104, 2.145902, 2.155705, 2.165512, 2.175323, 2.185140, 2.194960, 2.204785, 2.214615, 2.224450};
//float R_def[DEF_ARRAY_SIZE] = {1, 0.990455, 0.980910, 0.971366, 0.961821, 0.952275, 0.942730, 0.933185, 0.923639, 0.914093, 0.904546, 0.895000, 0.885453, 0.875905, 0.866357, 0.856808, 0.847259, 0.837710, 0.828160, 0.818609, 0.809057, 0.799505, 0.789952, 0.780398, 0.770844, 0.761288, 0.751732, 0.742175, 0.732617, 0.723057, 0.713497, 0.703936, 0.694373, 0.684810, 0.675245, 0.665679, 0.656112, 0.646544, 0.636974, 0.627403, 0.617830, 0.608256, 0.598681, 0.589104, 0.579526, 0.569946, 0.560364, 0.550781, 0.541196, 0.531609, 0.522021, 0.512431, 0.502839, 0.493245, 0.483649, 0.474051, 0.464452, 0.454850, 0.445246, 0.435641, 0.426033, 0.416423, 0.406810, 0.397196, 0.387579, 0.377960, 0.368339, 0.358715, 0.349089, 0.339461, 0.329830, 0.320196, 0.310560, 0.300922, 0.291280, 0.281636, 0.271990, 0.262340, 0.252688, 0.243033, 0.233375, 0.223715, 0.214051, 0.204384, 0.194715, 0.185042, 0.175367, 0.165688, 0.156006, 0.146321, 0.136632, 0.126941, 0.117246, 0.107548, 0.097846, 0.088141, 0.078433, 0.068721, 0.059005, 0.049286, 0.039563, 0.029837, 0.020107, 0.010374, 0.000636, -0.009105, -0.018850, -0.028599, -0.038352, -0.048109, -0.057869, -0.067634, -0.077403, -0.087176, -0.096953, -0.106734, -0.116520, -0.126310, -0.136104, -0.145902, -0.155705, -0.165512, -0.175323, -0.185140, -0.194960, -0.204785, -0.214615, -0.224450};

float monitoring_vcm_pulse_width;

float increment_acc;

float radius_memory[MEMORY_ARRAY_SIZE];
int encorder_memoryL[MEMORY_ARRAY_SIZE];
int encorder_memoryR[MEMORY_ARRAY_SIZE];
int encorder_playback[MEMORY_ARRAY_SIZE];
float speed_table[MEMORY_ARRAY_SIZE];

unsigned short max_abs_pot = 0;
int retention_pot[RETENTION_ARRAY_SIZE];	
short retention_access;
int deceleration_cnt;
short continued_cnt;

Flag flag;
Timer timer;
Digital digital;
sample smp = {{1,10,200,543,888}, 862};
pid_parameter gain;

float imu_data[10000];
float lowpassed_pot[RETENTION_ARRAY_SIZE];	

char read_startgoal_line = 1;
char read_side_line = 1;
char a = 0, b = 0, c = 0;
short max_access;
short data_access = 0;
int add_pot;

char imu_check;

float trace_val;
float speed_val;

short number_stored;

int test_aaray[TEST_SIZE];

char sd_mount_check;
float xg_l, yg_l, zg_l;
//short encorder_memory_2[MEMORY_ARRAY_SIZE_2];
float radius_memory_2[MEMORY_ARRAY_SIZE_2];
float radius_memory_3[MEMORY_ARRAY_SIZE_2];
float various_memory1[MEMORY_ARRAY_SIZE_2];
float various_memory2[MEMORY_ARRAY_SIZE_2];

float omega_x, omega_y, omega_z;
float omega_x_l, omega_y_l, omega_z_l;

float ff_accele_L, ff_accele_R;
float ff_duty_L, ff_duty_R;

//char fileName[][64] = {ENCORDER_LOG_TXT, IMU_LOG_TXT, POT_LOG_TXT};

#else
/* 自動生成変数 (CubeMXから変更を加えた場合変える必要があるかも)-------------------------------------------------------------*/

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;

extern UART_HandleTypeDef huart1;

/* ----------- -------------------------------------------------------------*/

extern char error_number;
extern uint16_t ad1[DATA_SIZE1];
extern uint16_t ad2[DATA_SIZE2];
extern int Line1, Line2, Line3, Line4, SideL, SideR, Pot;
extern int Def_ref;
extern int total_encL, total_encR;
extern int total_encL_memory, total_encR_memory;
extern short total_encL_distance, total_encR_distance;
extern signed char enc_overL, enc_overR;

extern int cycle_encL, cycle_encR;

extern float robot_speed;
extern float target_robot_speed;
extern float speed_L, speed_R;
extern float Def_speed_L, Def_speed_R;

extern float L_def[128];
extern float R_def[128];

extern float monitoring_vcm_pulse_width;

extern float increment_acc;

extern float radius_memory[MEMORY_ARRAY_SIZE];
extern int encorder_memoryL[MEMORY_ARRAY_SIZE];
extern int encorder_memoryR[MEMORY_ARRAY_SIZE];
extern int encorder_playback[MEMORY_ARRAY_SIZE];
extern float speed_table[MEMORY_ARRAY_SIZE];

extern unsigned short max_abs_pot;
extern int retention_pot[RETENTION_ARRAY_SIZE];	
extern short retention_access;
extern int deceleration_cnt;
extern short continued_cnt;

extern Flag flag;
extern Timer timer;
extern Digital digital;
extern sample smp;
extern pid_parameter gain;

extern short fresult;

extern float imu_data[10000];
extern float lowpassed_pot[RETENTION_ARRAY_SIZE];	

extern char read_startgoal_line;
extern char read_side_line;
extern char a, b, c;
extern short max_access;
extern short data_access;
extern int add_pot;

extern char imu_check;

extern float trace_val;
extern float speed_val;
extern short number_stored;
extern int test_aaray[TEST_SIZE];

extern char sd_mount_check;
extern float xg_l, yg_l, zg_l;

//extern short encorder_memory_2[MEMORY_ARRAY_SIZE_2];
extern float radius_memory_2[MEMORY_ARRAY_SIZE_2];
extern float radius_memory_3[MEMORY_ARRAY_SIZE_2];
extern float various_memory1[MEMORY_ARRAY_SIZE_2];
extern float various_memory2[MEMORY_ARRAY_SIZE_2];

extern float omega_x, omega_y, omega_z;
extern float omega_x_l, omega_y_l, omega_z_l;

extern float ff_accele_L, ff_accele_R;
extern float ff_duty_L, ff_duty_R;

//extern char fileName[3];

#endif
