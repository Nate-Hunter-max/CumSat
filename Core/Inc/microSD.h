/*
 * microSD.h
 *
 *  Created on: Jun 9, 2024
 *      Author: Nate Hunter
 */

#pragma once
#include "main.h"
#include "fatfs.h"

uint8_t microSD_Init();
FRESULT microSD_Write(void *buf, UINT len, const char* fileNameStr);
FRESULT microSD_RemoveFile(const char *fileNameStr);
