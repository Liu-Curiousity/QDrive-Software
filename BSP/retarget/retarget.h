#ifndef _RETARGET_H__
#define _RETARGET_H__

#ifdef __cplusplus
extern "C" {
#endif

#define USE_TinyPrintf 1

#define STDIO_SUPPORT 1

#include "stm32g4xx_hal.h"
#include <sys/stat.h>

#if USE_TinyPrintf == 1

#include "printf.h"

#endif

#if STDIO_SUPPORT == 1

#include <stdio.h>

#endif

#if STDIO_SUPPORT == 1

int _isatty(int fd);

int _write(int fd, char *ptr, int len);

int _close(int fd);

int _lseek(int fd, int ptr, int dir);

int _read(int fd, char *ptr, int len);

int _fstat(int fd, struct stat *st);

#endif

void CDC_Receive_FS_Callback(uint8_t *Buf, uint32_t *Len);
void CDC_TransmitCplt_FS_Callback();
signed short shellRead(char *data, unsigned short len);
signed short shellWrite(char *data, unsigned short len);

#ifdef __cplusplus
};
#endif

#endif //#ifndef _RETARGET_H__
