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
/* Solved a problem related to FreeBSD and some baudrates not recognized. */
/* For more info and how to use this library, visit: http://www.teuniz.net/RS-232/ */


#include "rs232.h"
#include <pthread.h>

static pthread_mutexattr_t mutex_attr;
static pthread_mutex_t read_mutex;
static pthread_mutex_t send_mutex;
static int mutexes_initialized = 0;
static int baudrate = -1;
static char mode[4] = { 0, 0, 0, 0 };
static int flowctrl = -1;
static int fd = -1;

#ifndef _WIN32   /* Linux & FreeBSD */
static int error = 0;

static struct termios new_port_settings, old_port_settings;

int RS232_SetupPort(int bauds, const char *m, int fc)
{
    if(baudrate == bauds && !strcmp(mode, m) && fc == flowctrl)
        return 0;
    strcpy(mode, m);
    flowctrl = fc;
    baudrate = bauds;
    int baudr, status;
    switch(baudrate)
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
    case  460800 : baudr = B460800;
                   break;
#if defined(__linux__)
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
    default      : printf("invalid baudrate\n");
                   return 1;
  }

  int cbits=CS8,  cpar=0, ipar=IGNPAR, bstop=0;

    if(strlen(mode) != 3)
    {
        printf("invalid mode \"%s\"\n", mode);
        return 1;
    }

    switch(mode[0])
    {
    case '8': cbits = CS8;
              break;
    case '7': cbits = CS7;
              break;
    case '6': cbits = CS6;
              break;
    case '5': cbits = CS5;
              break;
    default : printf("invalid number of data-bits '%c'\n", mode[0]);
              return 1;
    }

    switch(mode[1])
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
    default : printf("invalid parity '%c'\n", mode[1]);
              return 1;
    }

    switch(mode[2])
    {
    case '1': bstop = 0;
              break;
    case '2': bstop = CSTOPB;
              break;
    default : printf("invalid number of stop bits '%c'\n", mode[2]);
              return 1;
    }

    error = tcgetattr(fd, &old_port_settings);
    if(error==-1)
    {
        fprintf(stderr, "unable to read portsettings \n");
        return 1;
    }
    memset(&new_port_settings, 0, sizeof(new_port_settings));  /* clear the new struct */

    new_port_settings.c_cflag = (tcflag_t)(cbits | cpar | bstop | CLOCAL | CREAD);
    if(flowctrl)
    {
        new_port_settings.c_cflag |= CRTSCTS;
    }
    new_port_settings.c_iflag = (tcflag_t)ipar;
    new_port_settings.c_oflag = 0;
    new_port_settings.c_lflag = 0;
    new_port_settings.c_cc[VMIN] = 0;      /* block untill n bytes are received */
    new_port_settings.c_cc[VTIME] = 0;     /* block untill a timer expires (n * 100 mSec.) */

    cfsetispeed(&new_port_settings, (speed_t)baudr);
    cfsetospeed(&new_port_settings, (speed_t)baudr);

    error = tcsetattr(fd, TCSANOW, &new_port_settings);
    if(error==-1)
    {
        tcsetattr(fd, TCSANOW, &old_port_settings);
        fprintf(stderr, "unable to adjust portsettings \n");
        return 1;
    }

/* http://man7.org/linux/man-pages/man4/tty_ioctl.4.html */

    if(ioctl(fd, TIOCMGET, &status) == -1)
    {
        tcsetattr(fd, TCSANOW, &old_port_settings);
        fprintf(stderr, "unable to get portstatus\n");
        return 1;
    }

    status |= TIOCM_DTR;    /* turn on DTR */
    status |= TIOCM_RTS;    /* turn on RTS */

    if(ioctl(fd, TIOCMSET, &status) == -1)
    {
        tcsetattr(fd, TCSANOW, &old_port_settings);
        fprintf(stderr, "unable to set portstatus\n");
        return 1;
    }

    return 0;
}

void RS232_flushRX()
{
    tcflush(fd, TCIFLUSH);
}


void RS232_flushTX()
{
    tcflush(fd, TCOFLUSH);
}


void RS232_flushRXTX()
{
    tcflush(fd, TCIOFLUSH);
}

#else

int RS232_SetupPort(int bauds, const char *m, int fc)
{
    if(baudrate == bauds && !strcmp(mode, m) && fc == flowctrl)
        return 0;
    strcpy(mode, m);
    flowctrl = fc;
    baudrate = bauds;
    HANDLE pHandle = (HANDLE)_get_osfhandle(fd);

    COMMTIMEOUTS Cptimeouts;

    Cptimeouts.ReadIntervalTimeout         = MAXDWORD;
    Cptimeouts.ReadTotalTimeoutMultiplier  = 0;
    Cptimeouts.ReadTotalTimeoutConstant    = 50;
    Cptimeouts.WriteTotalTimeoutMultiplier = 0;
    Cptimeouts.WriteTotalTimeoutConstant   = 50;

    if(!SetCommTimeouts(pHandle, &Cptimeouts))
    {
        printf("unable to set comport time-out settings\n");
        return 1;
    }

    DCB port_settings;
    memset(&port_settings, 0, sizeof(DCB));
    port_settings.DCBlength = sizeof(DCB);

    if(!GetCommState(pHandle, &port_settings))
    {
        printf("unable to get comport cfg settings\n");
        return 1;
    }
    port_settings.DCBlength = sizeof(DCB);
    port_settings.BaudRate = baudrate;
    port_settings.XonChar = 0x13;
    port_settings.XoffChar = 0x19;
    port_settings.fOutxCtsFlow = 0;
    port_settings.fOutxDsrFlow = 0;
    port_settings.fDsrSensitivity = 0;
    port_settings.fOutX = 0;
    port_settings.fInX = 0;
    port_settings.fErrorChar = 0;
    port_settings.fBinary = 0;
    port_settings.fNull = 0;
    port_settings.fAbortOnError = 0;
    port_settings.XonLim = 0;
    port_settings.XoffLim = 0;
    port_settings.fTXContinueOnXoff = 1;

    switch(mode[0]) {
    case '5': port_settings.ByteSize = DATABITS_5; break;
    case '6': port_settings.ByteSize = DATABITS_6; break;
    case '7': port_settings.ByteSize = DATABITS_7; break;
    case '8': port_settings.ByteSize = DATABITS_8; break;
    default:
        fprintf(stderr, "invalid byte size\n");
    return 1;
    }
    switch(tolower(mode[1])) {
    case 'n': port_settings.Parity = NOPARITY; port_settings.fParity = 0; break;
    case 'o': port_settings.Parity = ODDPARITY; port_settings.fParity = 1; break;
    case 'e': port_settings.Parity = EVENPARITY; port_settings.fParity = 1; break;
    default:
        fprintf(stderr, "invalid parity\n");
    return 1;
    }
    switch(mode[2]) {
    case '1': port_settings.StopBits = ONESTOPBIT; break;
    case '2': port_settings.StopBits = TWOSTOPBITS; break;
    default:
        fprintf(stderr, "unable stop bits\n");
    return 1;
    }

    if(flowctrl)
    {
        port_settings.fOutxCtsFlow = TRUE;
        port_settings.fDtrControl = DTR_CONTROL_HANDSHAKE;
        port_settings.fRtsControl = RTS_CONTROL_HANDSHAKE;
    } else {
        port_settings.fOutxCtsFlow = FALSE;
        port_settings.fDtrControl = DTR_CONTROL_DISABLE;
        port_settings.fRtsControl = RTS_CONTROL_DISABLE;
    }

    if(!SetCommState(pHandle, &port_settings))
    {
        fprintf(stderr, "unable to set comport cfg settings\n");
        return 1;
    }

    return 0;
}

/*
https://msdn.microsoft.com/en-us/library/windows/desktop/aa363428%28v=vs.85%29.aspx
*/

void RS232_flushRX()
{
    HANDLE pHandle = (HANDLE)_get_osfhandle(fd);
    PurgeComm(pHandle, PURGE_RXCLEAR | PURGE_RXABORT);
}


void RS232_flushTX()
{
    HANDLE pHandle = (HANDLE)_get_osfhandle(fd);
    PurgeComm(pHandle, PURGE_TXCLEAR | PURGE_TXABORT);
}


void RS232_flushRXTX()
{
    HANDLE pHandle = (HANDLE)_get_osfhandle(fd);
    PurgeComm(pHandle, PURGE_RXCLEAR | PURGE_RXABORT);
    PurgeComm(pHandle, PURGE_TXCLEAR | PURGE_TXABORT);
}

#endif

int RS232_OpenComport(const char* devname)
{
    char dev_name[128];
#ifdef _WIN32
    sprintf(dev_name, "\\\\.\\%s", devname);
#else
    sprintf(dev_name, "%s", devname);
#endif
    if(fd == -1)
        fd = open(dev_name, O_RDWR);

    if(fd==-1) {
        fprintf(stderr, "unable to open comport: %s\n", strerror(errno));
        return 1;
    }
    if(!mutexes_initialized) {
        pthread_mutexattr_init(&mutex_attr);
        pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_ERRORCHECK);
        pthread_mutex_init(&read_mutex, &mutex_attr);
        pthread_mutex_init(&send_mutex, &mutex_attr);
        mutexes_initialized = 1;
    }
    return 0;
}

void RS232_CloseComport()
{
    if(fd != -1)
        close(fd);
    if(mutexes_initialized) {
        pthread_mutex_unlock(&read_mutex);
        pthread_mutex_destroy(&read_mutex);
        pthread_mutex_unlock(&send_mutex);
        pthread_mutex_destroy(&send_mutex);
        pthread_mutexattr_destroy(&mutex_attr);
        mutexes_initialized = 0;

    }
    strcpy(mode, "   ");
    flowctrl = -1;
    baudrate = -1;
    fd = -1;
}

int RS232_RecvBuf(unsigned char *buf, int size)
{
    int n = -ENODEV;
    int nread = 0;
    int ntries = size;
    int to_read = size;
    if(mutexes_initialized) {
        while(pthread_mutex_trylock(&read_mutex))
            usleep(10000000/baudrate);
        while(to_read > 0 && ntries-->0) {
            usleep(10000000/baudrate);
            n = read(fd, buf+nread, (size_t)to_read);
            if(n<1) {
                if(errno == EAGAIN)
                    continue;
                else {
                    n = -errno;
                    break;
                }
            }
            nread += n;
            to_read -= n;
        }
        pthread_mutex_unlock(&read_mutex);
    }
    return n;
}

int RS232_SendBuf(unsigned char *buf, int size)
{
    int n = -ENODEV;
    int nsent = 0;
    int ntries = size;
    int to_send = size;
    if(mutexes_initialized) {
        while(pthread_mutex_trylock(&send_mutex))
            usleep(10000000/baudrate);
        while(to_send > 0 && ntries-->0) {
            n = write(fd, buf+nsent, (size_t)to_send);
            usleep(10000000/baudrate);
            if(n<1) {
                if(errno == EAGAIN)
                    continue;
                else {
                    n = -errno;
                    break;
                }
            }
            nsent += n;
            to_send -= n;
        }
        pthread_mutex_unlock(&send_mutex);
    }
    return n;
}

int RS232_AlignFrame(int sof, int maxtries)
{
    int c = 0;
    RS232_flushRX();
    while(c != sof && maxtries-- > 0) {
        c = RS232_RecvByte();
        if(c < 0) {
          if(errno == EAGAIN)
              continue;
          else
              return -errno;
        }
    }
    return 0;
}

int RS232_RecvByte()
{
    int byte = 0;
    int n = RS232_RecvBuf((unsigned char*)&byte, 1);
    if(n < 1)
    {
        return -errno;
    }
    return byte;
}

int RS232_SendByte(unsigned char byte)
{
    int n = RS232_SendBuf(&byte, 1);
    if(n < 1)
    {
        return -errno;
    }
    return 0;
}

void RS232_SetFD(int f)
{
    if(!mutexes_initialized) {
        pthread_mutexattr_init(&mutex_attr);
        pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_ERRORCHECK);
        pthread_mutex_init(&read_mutex, &mutex_attr);
        pthread_mutex_init(&send_mutex, &mutex_attr);
        mutexes_initialized = 1;
    }
    fd = f;
}
