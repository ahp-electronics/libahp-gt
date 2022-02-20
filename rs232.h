/*
*    MIT License
*
*    rs232 sources serial communication driver
*    Copyright (C) 2022  Ilia Platone
*
*    Permission is hereby granted, free of charge, to any person obtaining a copy
*    of this software and associated documentation files (the "Software"), to deal
*    in the Software without restriction, including without limitation the rights
*    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*    copies of the Software, and to permit persons to whom the Software is
*    furnished to do so, subject to the following conditions:
*
*    The above copyright notice and this permission notice shall be included in all
*    copies or substantial portions of the Software.
*
*    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*    SOFTWARE.
*/

#ifndef rs232_INCLUDED
#define rs232_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#ifndef _WIN32

#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>
#include <sys/file.h>

#else

#include <windows.h>
#undef UNICODE
#undef _UNICODE
#include <winsock2.h>
#endif

int RS232_SetupPort(int baudrate, const char *mode, int flowctrl);
int RS232_OpenComport(const char *comport);
void RS232_SetFD(int f, int bauds);
int RS232_GetFD();
int RS232_AlignFrame(int sof, int maxtries);
int RS232_RecvBuf(unsigned char *buf, int size);
int RS232_RecvByte();
int RS232_SendByte(unsigned char byte);
int RS232_SendBuf(unsigned char *buf, int size);
void RS232_CloseComport(void);
void RS232_flushRX(void);
void RS232_flushTX(void);
void RS232_flushRXTX(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
