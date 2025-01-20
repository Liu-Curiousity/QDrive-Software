#ifndef _RETARGET_H__
#define _RETARGET_H__

#ifdef __cplusplus
extern "C" {
#endif

#define USE_TinyPrintf 1

#define STDIO_SUPPORT 0

#include "stm32g4xx_hal.h"
#include <sys/stat.h>

#if USE_TinyPrintf == 1

#include "printf.h"

#endif

#if STDIO_SUPPORT == 1

#include <stdio.h>

#endif

void RetargetInit(UART_HandleTypeDef *huart);

#if STDIO_SUPPORT == 1

int _isatty(int fd);

int _write(int fd, char *ptr, int len);

int _close(int fd);

int _lseek(int fd, int ptr, int dir);

int _read(int fd, char *ptr, int len);

int _fstat(int fd, struct stat *st);

#endif

#ifdef __cplusplus
};
#endif

#endif //#ifndef _RETARGET_H__