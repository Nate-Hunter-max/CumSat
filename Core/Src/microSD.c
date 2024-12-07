/*
 * microSD.c
 *
 *  Created on: Jun 9, 2024
 *      Author: Nate Hunter
 */
#include "microSD.h"

uint8_t microSD_Init(){
	FRESULT res = f_mount(&SDFatFS, SDPath, 1);
	return res == FR_OK;
}

FRESULT microSD_Write(void *buf, UINT len, const char* fileNameStr){
	UINT cnt;
	f_open(&SDFile, fileNameStr, FA_OPEN_ALWAYS | FA_WRITE);
	f_lseek(&SDFile, f_size(&SDFile));
	f_write(&SDFile, buf, len, &cnt);
	return f_close(&SDFile);
}

FRESULT microSD_RemoveFile(const char *fileNameStr){
	return f_unlink(fileNameStr);
}
