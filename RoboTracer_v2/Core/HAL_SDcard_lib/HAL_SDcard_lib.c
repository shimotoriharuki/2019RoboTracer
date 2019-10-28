/*
 * HAL_SDcard_lib.c
 *
 *  Created on: 2019/10/25
 *      Author: UnderBird
 */

#include <stdio.h>
#include "fatfs.h"

#include "HAL_SDcard_lib.h"
#include "fatfs_sd.h"
#include "string.h"
#include "Macros.h"

#define BUFF_SIZE 1024

FATFS fs;	//ファイルシステムのやつ
FIL fil;	//ファイルのやつ
FRESULT fresult;
char buffer[BUFF_SIZE];
UINT br, bw;

FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

//************************************************************************/
//* 役割　：　SDに書き込む
//* 引数　：　char, float: ファイル選択, データ
//* 戻り値：　char: 状態チェック	0(SDカードがない) or 1(成功) or 2(マウント失敗)
//* 備考 : なし
//************************************************************************/
FRESULT sd_write(char f_sel, float data){
	FRESULT ret = 0;

	snprintf(buffer, BUFF_SIZE, "%f\n", data);	//floatをstringに変換

	select_open_filename(f_sel);	//書き込むファイルを選択
	f_write(&fil, buffer, BUFF_SIZE, &bw);	//書き込む

	bufclear();	//書き込み用のバッファをクリア
	f_close(&fil);	//ファイル閉じる

	return ret;
}

//************************************************************************/
//* 役割　：　SDから読み込む
//* 引数　：　char, float *, short　: ファイル選択, データ, データサイズ
//* 戻り値：　char: 状態チェック	0(Sdカードがない) or 1(成功) or 2(マウント失敗)
//* 備考 : なし
//************************************************************************/
FRESULT sd_read(char f_sel, float *data){
	FRESULT ret = 0;

	select_open_filename(f_sel);	//書き込むファイルを選択
	f_read (&fil, buffer, BUFF_SIZE, &br);

	sscanf(buffer, "%f", data);

	bufclear();	//書き込み用のバッファをクリア
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
//* 役割　：　操作するファイルを選択する
//* 引数　：　char: ファイル選択
//* 戻り値：　char: 状態チェック	0(マウント失敗) or 1(成功)
//* 備考 : なし
//************************************************************************/
void select_open_filename(char f_sel){
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

}


//************************************************************************/
//* 役割　：　バッファをクリア
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
void bufclear(void){
	for(int i = 0; i < 1024; i++){
		buffer[i] = '\0';
	}
}
