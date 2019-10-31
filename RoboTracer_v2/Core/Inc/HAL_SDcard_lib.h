/*
 * HAL_SDcard_lib.h
 *
 *  Created on: 2019/10/25
 *      Author: UnderBird
 */


#ifndef HAL_SDCARD_LIB_H
#define HAL_SDCARD_LIB_H

void select_open_filename(char);		//ファイルを選択する
void select_open_filename_2(void);		//ファイルを選択する
void create_path(char, char);
FRESULT sd_mount(void);   					//SDをマウント
FRESULT sd_unmount(char);					  //SDをアンマウント
FRESULT sd_write(char, char , short, float *, char); 	 //SDに書き込み
FRESULT sd_read(char, char , short, float *);		//SDから読み込み
void bufclear(void);


#endif
