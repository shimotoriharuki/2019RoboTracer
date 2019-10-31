/*
 * HAL_SDcard_lib.c
 *
 *  Created on: 2019/10/25
 *      Author: UnderBird
 */

#include <stdio.h>
#include "fatfs.h"

#include "Macros.h"
#include "HAL_SDcard_lib.h"
#include "string.h"

#include "fatfs_sd.h"


#define BUFF_SIZE 256

FATFS fs;	//ファイルシステムのやつ
FIL fil;	//ファイルのやつ
FRESULT fresult;
char buffer[BUFF_SIZE];
UINT br, bw;

FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

char filepath[256];
char dirpath[256];



//************************************************************************/
//* 役割　：　SDに書き込む
//* 引数　：　char, short, float: ファイル選択, 配列の数, データ
//* 戻り値：　char: 状態チェック	0(SDカードがない) or 1(成功) or 2(マウント失敗)
//* 備考 : なし
//************************************************************************/
FRESULT sd_write(char f_sel, short size, float *data, char state){
	FRESULT ret = 0;

	if(state == OVER_WRITE) f_unlink("/encorder_log.txt\0");

	select_open_filename(f_sel);	//書き込むファイルを選択

	for(short i = 0 ; i < size; i++){
		snprintf(buffer, BUFF_SIZE, "%f\n", *(data + i));	//floatをstringに変換

		f_lseek(&fil, f_size(&fil));	//ファイルの最後に移動
		f_write(&fil, buffer, strlen(buffer), &bw);	//書き込む

		bufclear();	//書き込み用のバッファをクリア
	}
	f_close(&fil);	//ファイル閉じる

	return ret;
}

//************************************************************************/
//* 役割　：　SDに書き込む_2
//* 引数　：　char, short, float: ファイル選択, 配列の数, データ
//* 戻り値：　char: 状態チェック	0(SDカードがない) or 1(成功) or 2(マウント失敗)
//* 備考 : なし
//************************************************************************/
FRESULT sd_write_2(char folder_number_char, char file_number, short size, float *data, char state){
	FRESULT ret = 0;

	create_path(folder_number_char, file_number);

	if(state == OVER_WRITE) f_unlink(filepath);	//一回消す

	select_open_filename_2();	//書き込むファイルを選択

	for(short i = 0 ; i < size; i++){
		snprintf(buffer, BUFF_SIZE, "%f\n", *(data + i));	//floatをstringに変換

		f_lseek(&fil, f_size(&fil));	//ファイルの最後に移動
		f_write(&fil, buffer, strlen(buffer), &bw);	//書き込む

		bufclear();	//書き込み用のバッファをクリア
	}
	f_close(&fil);	//ファイル閉じる

	return ret;
}

//************************************************************************/
//* 役割　：　SDから読み込む
//* 引数　：　char, float *, short　: ファイル選択, データ, データサイズ
//* 戻り値：　char: 状態チェック	0(Sdカードがない) or 1(成功) or 2(マウント失敗)
//* 備考 : なし
//************************************************************************/
FRESULT sd_read(char f_sel, short size, float *data){
	FRESULT ret = 0;
	float test = 0;
	short i = 0;



	select_open_filename(f_sel);	//書き込むファイルを選択

	//for(short i = 0 ; i < size; i++){
		while(f_gets(buffer, sizeof(buffer), &fil) != NULL){
			sscanf(buffer, "%f", data + i);
			i++;
			if(i >= size) i = size - 1;
			//printf("%f\r\n", test);
		}


		bufclear();	//書き込み用のバッファをクリア
		//printf("%f\r\n", test);


		//sscanf(buffer, "%f", (data + i));

		//printf("%f\r\n", test);


	//}

	f_close(&fil);	//ファイル閉じる

	return ret;
}

//************************************************************************/
//* 役割　：　SDカードをマウント
//* 引数　：　char: ファイル選択
//* 戻り値：　char: 状態チェック	0(マウント失敗) or 1(成功)
//* 備考 : なし
//************************************************************************/
FRESULT sd_mount(){
	FRESULT ret = 0;

	if(f_mount(&fs, "", 1) == FR_OK) ret = 1;
	else ret = 0;

	return ret;
}

//************************************************************************/
//* 役割　：　SDカードをアンマウント
//* 引数　：　char: ファイル選択
//* 戻り値：　char: 状態チェック	0(マウント失敗) or 1(成功)
//* 備考 : なし
//************************************************************************/
FRESULT sd_unmount(char f_sel){
	FRESULT ret = 0;

	if(f_mount(NULL, "", 1) == FR_OK) ret = 1;
	else ret = 0;

	return ret;
}

//************************************************************************/
//* 役割　：　操作するパスの文字列を作る
//* 引数　：　char, char: フォルダー選択, ファイル選択
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
void create_path(char folder_number_char, char file_number){

	sprintf(dirpath, "log_folder_%c.txt", folder_number_char);

	switch (file_number){
		case ENCORDER_LOG:
			strcpy(filepath, ENCORDER_LOG_TXT);
		break;
		case IMU_LOG:
			strcpy(filepath, IMU_LOG_TXT);
		break;

		case POT_LOG:
			strcpy(filepath, POT_LOG_TXT);
		break;

		case LINE_SENSOR_LOG:
			strcpy(filepath, LINE_SENSOR_LOG_TXT);
		break;

	}

}
//************************************************************************/
//* 役割　：　操作するファイルを選択する
//* 引数　：　char: ファイル選択
//* 戻り値：　char: 状態チェック	0(マウント失敗) or 1(成功)
//* 備考 : なし
//************************************************************************/
void select_open_filename(char f_sel){
/*
	switch(f_sel){
		case ENCORDER_LOG:
			f_open(&fil, "encorder_log.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		break;

		case IMU_LOG:
			f_open(&fil, "imu_log.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		break;

		case POT_LOG:
			f_open(&fil, "pot_log.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		break;

		case LINE_SENSOR_LOG:
			f_open(&fil, "line_sensor_log.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		break;

	}
*/

}

//************************************************************************/
//* 役割　：　操作するファイルを選択する_2
//* 引数　：　char: ファイル選択
//* 戻り値：　char: 状態チェック	0(マウント失敗) or 1(成功)
//* 備考 : なし
//************************************************************************/
void select_open_filename_2(){	//mkdir

	f_mkdir(dirpath);

	f_chdir(dirpath);

	f_open(&fil, filepath, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

	f_chdir("..");


}
//************************************************************************/
//* 役割　：　バッファをクリア
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
void bufclear(void){
	for(int i = 0; i < BUFF_SIZE; i++){
		buffer[i] = '\0';
	}
}
