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

static pthread_mutexattr_t ahp_serial_mutex_attr;
static pthread_mutex_t ahp_serial_read_mutex;
static pthread_mutex_t ahp_serial_send_mutex;
static int ahp_serial_mutexes_initialized = 0;
static int ahp_serial_baudrate = 230400;
static char ahp_serial_mode[4] = { 0, 0, 0, 0 };
static int ahp_serial_flowctrl = -1;
static int ahp_serial_fd = -1;

#ifndef WINDOWS
static int ahp_serial_error = 0;

static struct termios ahp_serial_new_port_settings, ahp_serial_old_port_settings;

static int ahp_serial_SetupPort(int bauds, const char *m, int fc)
{
    strcpy(ahp_serial_mode, m);
    ahp_serial_flowctrl = fc;
    ahp_serial_baudrate = bauds;
    int baudr, status;
    switch(ahp_serial_baudrate)
    {
    case      50 : baudr = B50;
                   break;
    case      75 : baudr = B75;
                   break;
    case     110 : baudr = B110;
                   break;
    case     134 : baudr = B134;
                   break;
    case     150 : baudr = B150;
                   break;
    case     200 : baudr = B200;
                   break;
    case     300 : baudr = B300;
                   break;
    case     600 : baudr = B600;
                   break;
    case    1200 : baudr = B1200;
                   break;
    case    1800 : baudr = B1800;
                   break;
    case    2400 : baudr = B2400;
                   break;
    case    4800 : baudr = B4800;
                   break;
    case    9600 : baudr = B9600;
                   break;
    case   19200 : baudr = B19200;
                   break;
    case   38400 : baudr = B38400;
                   break;
    case   57600 : baudr = B57600;
                   break;
    case  115200 : baudr = B115200;
                   break;
    case  230400 : baudr = B230400;
                   break;
#if defined(__linux__)
    case  460800 : baudr = B460800;
                   break;
    case  500000 : baudr = B500000;
                   break;
    case  576000 : baudr = B576000;
                   break;
    case  921600 : baudr = B921600;
                   break;
    case 1000000 : baudr = B1000000;
                   break;
    case 1152000 : baudr = B1152000;
                   break;
    case 1500000 : baudr = B1500000;
                   break;
    case 2000000 : baudr = B2000000;
                   break;
    case 2500000 : baudr = B2500000;
                   break;
    case 3000000 : baudr = B3000000;
                   break;
    case 3500000 : baudr = B3500000;
                   break;
    case 4000000 : baudr = B4000000;
                   break;
#endif
    default      : printf("invalid ahp_serial_baudrate\n");
                   return 1;
  }

  int cbits=CS8,  cpar=0, ipar=IGNPAR, bstop=0;

    if(strlen(ahp_serial_mode) != 3)
    {
        printf("invalid ahp_serial_mode \"%s\"\n", ahp_serial_mode);
        return 1;
    }

    switch(ahp_serial_mode[0])
    {
    case '8': cbits = CS8;
              break;
    case '7': cbits = CS7;
              break;
    case '6': cbits = CS6;
              break;
    case '5': cbits = CS5;
              break;
    default : printf("invalid number of data-bits '%c'\n", ahp_serial_mode[0]);
              return 1;
    }

    switch(ahp_serial_mode[1])
    {
    case 'N':
    case 'n': cpar = 0;
              ipar = IGNPAR;
              break;
    case 'E':
    case 'e': cpar = PARENB;
              ipar = INPCK;
              break;
    case 'O':
    case 'o': cpar = (PARENB | PARODD);
              ipar = INPCK;
              break;
    default : printf("invalid parity '%c'\n", ahp_serial_mode[1]);
              return 1;
    }

    switch(ahp_serial_mode[2])
    {
    case '1': bstop = 0;
              break;
    case '2': bstop = CSTOPB;
              break;
    default : printf("invalid number of stop bits '%c'\n", ahp_serial_mode[2]);
              return 1;
    }

    ahp_serial_error = tcgetattr(ahp_serial_fd, &ahp_serial_old_port_settings);
    if(ahp_serial_error==-1)
    {
        fprintf(stderr, "unable to read portsettings \n");
        return 1;
    }
    memset(&ahp_serial_new_port_settings, 0, sizeof(ahp_serial_new_port_settings));  /* clear the new struct */

    ahp_serial_new_port_settings.c_cflag = (tcflag_t)(cbits | cpar | bstop | CLOCAL | CREAD);
    if(ahp_serial_flowctrl)
    {
        ahp_serial_new_port_settings.c_cflag |= CRTSCTS;
    }
    ahp_serial_new_port_settings.c_iflag = (tcflag_t)ipar;
    ahp_serial_new_port_settings.c_oflag = 0;
    ahp_serial_new_port_settings.c_lflag = 0;
    ahp_serial_new_port_settings.c_cc[VMIN] = 0;      /* block untill n bytes are received */
    ahp_serial_new_port_settings.c_cc[VTIME] = 0;     /* block untill a timer expires (n * 100 mSec.) */

    cfsetispeed(&ahp_serial_new_port_settings, (speed_t)baudr);
    cfsetospeed(&ahp_serial_new_port_settings, (speed_t)baudr);

    ahp_serial_error = tcsetattr(ahp_serial_fd, TCSANOW, &ahp_serial_new_port_settings);
    if(ahp_serial_error==-1)
    {
        tcsetattr(ahp_serial_fd, TCSANOW, &ahp_serial_old_port_settings);
        fprintf(stderr, "unable to adjust portsettings \n");
        return 1;
    }

/* http://man7.org/linux/man-pages/man4/tty_ioctl.4.html */

    if(ioctl(ahp_serial_fd, TIOCMGET, &status) == -1)
    {
        tcsetattr(ahp_serial_fd, TCSANOW, &ahp_serial_old_port_settings);
        fprintf(stderr, "unable to get portstatus\n");
        return 1;
    }

    status |= TIOCM_DTR;    /* turn on DTR */
    status |= TIOCM_RTS;    /* turn on RTS */

    if(ioctl(ahp_serial_fd, TIOCMSET, &status) == -1)
    {
        tcsetattr(ahp_serial_fd, TCSANOW, &ahp_serial_old_port_settings);
        fprintf(stderr, "unable to set portstatus\n");
        return 1;
    }

    return 0;
}

static void ahp_serial_flushRX()
{
    tcflush(ahp_serial_fd, TCIFLUSH);
}


static void ahp_serial_flushTX()
{
    tcflush(ahp_serial_fd, TCOFLUSH);
}


static void ahp_serial_flushRXTX()
{
    tcflush(ahp_serial_fd, TCIOFLUSH);
}

#else

static DCB ahp_serial_new_port_settings, ahp_serial_old_port_settings;

static int ahp_serial_SetupPort(int bauds, const char *m, int fc)
{
    strcpy(ahp_serial_mode, m);
    ahp_serial_flowctrl = fc;
    ahp_serial_baudrate = bauds;
    HANDLE pHandle = (HANDLE)_get_osfhandle(ahp_serial_fd);

    memset(&ahp_serial_old_port_settings, 0, sizeof(DCB));

    if(!GetCommState(pHandle, &ahp_serial_old_port_settings))
    {
        printf("unable to get comport cfg settings\n");
        return 1;
    }
    memset(&ahp_serial_new_port_settings, 0, sizeof(DCB));
    ahp_serial_new_port_settings.DCBlength = sizeof(DCB);
    ahp_serial_new_port_settings.BaudRate = ahp_serial_baudrate;
    ahp_serial_new_port_settings.XonChar = 0x13;
    ahp_serial_new_port_settings.XoffChar = 0x19;
    ahp_serial_new_port_settings.fOutxCtsFlow = 0;
    ahp_serial_new_port_settings.fOutxDsrFlow = 0;
    ahp_serial_new_port_settings.fDsrSensitivity = 0;
    ahp_serial_new_port_settings.fOutX = 0;
    ahp_serial_new_port_settings.fInX = 0;
    ahp_serial_new_port_settings.fErrorChar = 0;
    ahp_serial_new_port_settings.fBinary = 1;
    ahp_serial_new_port_settings.fNull = 0;
    ahp_serial_new_port_settings.fAbortOnError = 0;
    ahp_serial_new_port_settings.XonLim = 0;
    ahp_serial_new_port_settings.XoffLim = 0;
    ahp_serial_new_port_settings.fTXContinueOnXoff = 1;

    switch(ahp_serial_mode[0]) {
    case '5': ahp_serial_new_port_settings.ByteSize = DATABITS_5; break;
    case '6': ahp_serial_new_port_settings.ByteSize = DATABITS_6; break;
    case '7': ahp_serial_new_port_settings.ByteSize = DATABITS_7; break;
    case '8': ahp_serial_new_port_settings.ByteSize = DATABITS_8; break;
    default:
        fprintf(stderr, "invalid byte size\n");
    return 1;
    }
    switch(tolower(ahp_serial_mode[1])) {
    case 'n': ahp_serial_new_port_settings.Parity = NOPARITY; ahp_serial_new_port_settings.fParity = 0; break;
    case 'o': ahp_serial_new_port_settings.Parity = ODDPARITY; ahp_serial_new_port_settings.fParity = 1; break;
    case 'e': ahp_serial_new_port_settings.Parity = EVENPARITY; ahp_serial_new_port_settings.fParity = 1; break;
    default:
        fprintf(stderr, "invalid parity\n");
    return 1;
    }
    switch(ahp_serial_mode[2]) {
    case '1': ahp_serial_new_port_settings.StopBits = ONESTOPBIT; break;
    case '2': ahp_serial_new_port_settings.StopBits = TWOSTOPBITS; break;
    default:
        fprintf(stderr, "invalid stop bits\n");
    return 1;
    }

    if(ahp_serial_flowctrl)
    {
        ahp_serial_new_port_settings.fOutxCtsFlow = TRUE;
        ahp_serial_new_port_settings.fDtrControl = DTR_CONTROL_HANDSHAKE;
        ahp_serial_new_port_settings.fRtsControl = RTS_CONTROL_HANDSHAKE;
    } else {
        ahp_serial_new_port_settings.fOutxCtsFlow = FALSE;
        ahp_serial_new_port_settings.fDtrControl = DTR_CONTROL_DISABLE;
        ahp_serial_new_port_settings.fRtsControl = RTS_CONTROL_DISABLE;
    }

    ahp_serial_new_port_settings.DCBlength = sizeof(ahp_serial_new_port_settings);

    if(!SetCommState(pHandle, &ahp_serial_new_port_settings))
    {
        fprintf(stderr, "unable to set comport cfg settings\n");
        return 1;
    }

    COMMTIMEOUTS Cptimeouts;
    if(!GetCommTimeouts(pHandle, &Cptimeouts))
    {
        printf("unable to get comport timeouts\n");
        return 1;
    }

    Cptimeouts.ReadTotalTimeoutMultiplier  = 1;
    Cptimeouts.ReadTotalTimeoutConstant    = 1;
    Cptimeouts.WriteTotalTimeoutMultiplier  = 1;
    Cptimeouts.WriteTotalTimeoutConstant    = 1;

    if(!SetCommTimeouts(pHandle, &Cptimeouts))
    {
        printf("unable to set comport timeouts\n");
        return 1;
    }


    return 0;
}

/*
https://msdn.microsoft.com/en-us/library/windows/desktop/aa363428%28v=vs.85%29.aspx
*/

static void ahp_serial_flushRX()
{
    HANDLE pHandle = (HANDLE)_get_osfhandle(ahp_serial_fd);
    PurgeComm(pHandle, PURGE_RXCLEAR | PURGE_RXABORT);
}


static void ahp_serial_flushTX()
{
    HANDLE pHandle = (HANDLE)_get_osfhandle(ahp_serial_fd);
    PurgeComm(pHandle, PURGE_TXCLEAR | PURGE_TXABORT);
}


static void ahp_serial_flushRXTX()
{
    HANDLE pHandle = (HANDLE)_get_osfhandle(ahp_serial_fd);
    PurgeComm(pHandle, PURGE_RXCLEAR | PURGE_RXABORT);
    PurgeComm(pHandle, PURGE_TXCLEAR | PURGE_TXABORT);
}

#endif

static int ahp_serial_OpenComport(const char* devname)
{
    char dev_name[128];
#ifndef _WIN32
    sprintf(dev_name, "/dev/%s", devname);
#else
    sprintf(dev_name, "\\\\.\\%s", devname);
#endif
    if(ahp_serial_fd == -1)
        ahp_serial_fd = open(dev_name, O_RDWR);

    if(ahp_serial_fd==-1) {
        fprintf(stderr, "unable to open comport: %s\n", strerror(errno));
        return 1;
    }
    if(!ahp_serial_mutexes_initialized) {
        pthread_mutexattr_init(&ahp_serial_mutex_attr);
        pthread_mutexattr_settype(&ahp_serial_mutex_attr, PTHREAD_MUTEX_ERRORCHECK);
        pthread_mutex_init(&ahp_serial_read_mutex, &ahp_serial_mutex_attr);
        pthread_mutex_init(&ahp_serial_send_mutex, &ahp_serial_mutex_attr);
        ahp_serial_mutexes_initialized = 1;
    }
    int value = 0x1000;
    setsockopt(ahp_serial_fd, SOL_SOCKET, SO_SNDBUF, (const char *)&value, sizeof(int));
    setsockopt(ahp_serial_fd, SOL_SOCKET, SO_RCVBUF, (const char *)&value, sizeof(int));
    return 0;
}

static void ahp_serial_CloseComport()
{
    if(ahp_serial_fd != -1)
        close(ahp_serial_fd);
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
    int value = 0x1000;
    setsockopt(ahp_serial_fd, SOL_SOCKET, SO_SNDBUF, (const char *)&value, sizeof(int));
    setsockopt(ahp_serial_fd, SOL_SOCKET, SO_RCVBUF, (const char *)&value, sizeof(int));
}

static int ahp_serial_GetFD()
{
    return ahp_serial_fd;
}

#ifdef __cplusplus
} /* extern "C" */
#endif
