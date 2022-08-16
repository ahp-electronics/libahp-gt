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

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#if defined(__linux__) || defined(__linux) || defined(linux) || defined(__gnu_linux__)
    #define LINUX
#elif defined(__APPLE__) && defined(__MACH__)
    #define MACOS
#elif defined(_WIN32) || defined(_WIN64)
    #define WINDOWS
#endif

#ifndef WINDOWS

#include <termios.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <limits.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#else
#include <winsock2.h>
#include <windows.h>
#include <ws2tcpip.h>
#include <winsock.h>
#undef UNICODE
#undef _UNICODE
#endif
#include <pthread.h>
#include "libserialport/libserialport.h"

static pthread_mutexattr_t ahp_serial_mutex_attr;
static pthread_mutex_t ahp_serial_read_mutex;
static pthread_mutex_t ahp_serial_send_mutex;
static int ahp_serial_mutexes_initialized = 0;
static int ahp_serial_baudrate = 9600;
static char ahp_serial_mode[4] = { 0, 0, 0, 0 };
static int ahp_serial_flowctrl = -1;
static int ahp_serial_fd = -1;
static struct sp_port *serialport;

static int ahp_serial_SetupPort(int bauds, const char *m)
{
    ahp_serial_baudrate = bauds;
    sp_set_baudrate(serialport, bauds);
    sp_set_bits(serialport, m[0] - '0');
    int parity = SP_PARITY_NONE;
    switch (m[1]) {
    case 'O':
    case 'o':
        parity = SP_PARITY_ODD;
        break;
    case 'E':
    case 'e':
        parity = SP_PARITY_EVEN;
        break;
    case 'M':
    case 'm':
        parity = SP_PARITY_MARK;
        break;
    case 'S':
    case 's':
        parity = SP_PARITY_SPACE;
        break;
    default:
        break;
    }

    sp_set_parity(serialport, parity);
    sp_set_stopbits(serialport, m[2]-'0');

    return 0;
}

static void ahp_serial_flushRX()
{
    sp_flush(serialport, SP_BUF_INPUT);
}


static void ahp_serial_flushTX()
{
    sp_flush(serialport, SP_BUF_OUTPUT);
}


static void ahp_serial_flushRXTX()
{
    sp_flush(serialport, SP_BUF_INPUT|SP_BUF_OUTPUT);
}

static int ahp_serial_OpenComport(const char* devname)
{
    char dev_name[128];
    sprintf(dev_name, "%s", devname);
    int err = sp_get_port_by_name(devname, &serialport);
    if (err != SP_OK) {
        fprintf(stderr, "no such comport\n");
        return 1;
    }
    err = sp_open(serialport, SP_MODE_READ_WRITE);
    if (err != SP_OK) {
        fprintf(stderr, "unable to open comport: %s\n", strerror(errno));
        return 1;
    }
    err = sp_get_port_handle(serialport, &ahp_serial_fd);
    if (ahp_serial_fd < 0) {
        fprintf(stderr, "invalid file descriptor\n");
        return 1;
    }
    if(!ahp_serial_mutexes_initialized) {
        pthread_mutexattr_init(&ahp_serial_mutex_attr);
        pthread_mutexattr_settype(&ahp_serial_mutex_attr, PTHREAD_MUTEX_ERRORCHECK);
        pthread_mutex_init(&ahp_serial_read_mutex, &ahp_serial_mutex_attr);
        pthread_mutex_init(&ahp_serial_send_mutex, &ahp_serial_mutex_attr);
        ahp_serial_mutexes_initialized = 1;
    }
    return 0;
}

static void ahp_serial_CloseComport()
{
    if(ahp_serial_fd != -1)
        sp_close(serialport);
    if(ahp_serial_mutexes_initialized) {
        pthread_mutex_unlock(&ahp_serial_read_mutex);
        pthread_mutex_destroy(&ahp_serial_read_mutex);
        pthread_mutex_unlock(&ahp_serial_send_mutex);
        pthread_mutex_destroy(&ahp_serial_send_mutex);
        pthread_mutexattr_destroy(&ahp_serial_mutex_attr);
        ahp_serial_mutexes_initialized = 0;

    }
    strcpy(ahp_serial_mode, "   ");
    ahp_serial_flowctrl = -1;
    ahp_serial_baudrate = -1;
    ahp_serial_fd = -1;
}

static int ahp_serial_RecvBuf(unsigned char *buf, int size)
{
    int n = -ENODEV;
    int nbytes = 0;
    int ntries = size*2;
    int bytes_left = size;
    int err = 0;

    if(ahp_serial_mutexes_initialized) {
        while(pthread_mutex_trylock(&ahp_serial_read_mutex))
            usleep(100);
        while(bytes_left > 0 && ntries-->0) {
            usleep(12000000/ahp_serial_baudrate);
            n = read(ahp_serial_fd, buf+nbytes, bytes_left);
            if(n<1) {
                err = -errno;
                continue;
            }
            nbytes += n;
            bytes_left -= n;
        }
        pthread_mutex_unlock(&ahp_serial_read_mutex);
    }
    if(nbytes < size)
        return err;
    return nbytes;
}

static int ahp_serial_SendBuf(unsigned char *buf, int size)
{
    int n = -ENODEV;
    int nbytes = 0;
    int ntries = size*2;
    int bytes_left = size;
    int err = 0;
    if(ahp_serial_mutexes_initialized) {
        while(pthread_mutex_trylock(&ahp_serial_send_mutex))
            usleep(100);
        while(bytes_left > 0 && ntries-->0) {
            usleep(12000000/ahp_serial_baudrate);
            n = write(ahp_serial_fd, buf+nbytes, bytes_left);
            if(n<1) {
                err = -errno;
                continue;
            }
            nbytes += n;
            bytes_left -= n;
        }
        pthread_mutex_unlock(&ahp_serial_send_mutex);
    }
    if(nbytes < size)
        return err;
    return nbytes;
}

static int ahp_serial_RecvByte()
{
    int byte = 0;
    int n = ahp_serial_RecvBuf((unsigned char*)&byte, 1);
    if(n < 1)
    {
        return -errno;
    }
    return byte;
}

static int ahp_serial_SendByte(unsigned char byte)
{
    int n = ahp_serial_SendBuf(&byte, 1);
    if(n < 1)
    {
        return -errno;
    }
    return 0;
}

static int ahp_serial_AlignFrame(int sof, int maxtries)
{
    int c = 0;
    ahp_serial_flushRX();
    while(c != sof && maxtries-- > 0) {
        c = ahp_serial_RecvByte();
        if(c < 0) {
          if(errno == EAGAIN)
              continue;
          else
              return errno;
        }
    }
    return 0;
}

static void ahp_serial_SetFD(int f, int bauds)
{
    if(!ahp_serial_mutexes_initialized) {
        pthread_mutexattr_init(&ahp_serial_mutex_attr);
        pthread_mutexattr_settype(&ahp_serial_mutex_attr, PTHREAD_MUTEX_ERRORCHECK);
        pthread_mutex_init(&ahp_serial_read_mutex, &ahp_serial_mutex_attr);
        pthread_mutex_init(&ahp_serial_send_mutex, &ahp_serial_mutex_attr);
        ahp_serial_mutexes_initialized = 1;
    }
    ahp_serial_fd = f;
    ahp_serial_baudrate = bauds;
}

static int ahp_serial_GetFD()
{
    return ahp_serial_fd;
}

#ifdef __cplusplus
} /* extern "C" */
#endif
