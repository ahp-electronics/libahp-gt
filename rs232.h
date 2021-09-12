/*
***************************************************************************
*
* Author: Teunis van Beelen
*
* Copyright (C) 2005 - 2020 Teunis van Beelen
*
* Email: teuniz@protonmail.com
*
***************************************************************************
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
***************************************************************************
*/

/* Last revision: August 6, 2020 */

/* For more info and how to use this library, visit: http://www.teuniz.net/RS-232/ */


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
#endif

int RS232_SetupPort(int baudrate, const char *mode, int flowctrl);
int RS232_OpenComport(const char *comport);
void RS232_SetFD(int f);
int RS232_AlignFrame(int sof, int maxtries);
int RS232_RecvByte();
int RS232_RecvBuf(unsigned char *buf, int size);
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
