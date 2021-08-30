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


static int baudrate = 57600;
#if defined(__linux__) || defined(__FreeBSD__)   /* Linux & FreeBSD */
static int error = 0;
static int fd = -1;

static struct termios new_port_settings, old_port_settings;

int RS232_SetupPort(int bauds, const char *mode, int flowctrl)
{
  int baudr,
      status;
   baudrate = bauds;
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
                   return(1);
  }

  int cbits=CS8,
      cpar=0,
      ipar=IGNPAR,
      bstop=0;

  if(strlen(mode) != 3)
  {
    printf("invalid mode \"%s\"\n", mode);
    return(1);
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
              return(1);
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
              return(1);
  }

  switch(mode[2])
  {
    case '1': bstop = 0;
              break;
    case '2': bstop = CSTOPB;
              break;
    default : printf("invalid number of stop bits '%c'\n", mode[2]);
              return(1);
  }

  error = tcgetattr(fd, &old_port_settings);
  if(error==-1)
  {
    fprintf(stderr, "unable to read portsettings \n");
    return(1);
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
    return(1);
  }

/* http://man7.org/linux/man-pages/man4/tty_ioctl.4.html */

  if(ioctl(fd, TIOCMGET, &status) == -1)
  {
    tcsetattr(fd, TCSANOW, &old_port_settings);
    fprintf(stderr, "unable to get portstatus\n");
    return(1);
  }

  status |= TIOCM_DTR;    /* turn on DTR */
  status |= TIOCM_RTS;    /* turn on RTS */

  if(ioctl(fd, TIOCMSET, &status) == -1)
  {
    tcsetattr(fd, TCSANOW, &old_port_settings);
    fprintf(stderr, "unable to set portstatus\n");
    return(1);
  }

  return(0);
}

int RS232_OpenComport(const char* devname)
{
    fd = open(devname, O_RDWR | O_NOCTTY | O_NDELAY);

    if(fd==-1)
    {
      fprintf(stderr, "unable to setup comport\n");
      return(1);
    }

    /* lock access so that another process can't also use the port */
    if(flock(fd, LOCK_EX | LOCK_NB) != 0)
    {
      close(fd);
      fprintf(stderr, "Another process has locked the comport.\n");
      return(1);
    }
    return 0;
}


int RS232_AlignFrame(int sof, int maxtries)
{
    int n;
    int c = 0;
    RS232_flushRX();
    while(c != sof && maxtries-- > 0) {
        n = RS232_PollComport((char*)&c, 1);
        if(n<0) {
          if(errno == EAGAIN)
              continue;
          else
              return 1;
        }
    }
    return 0;
}

int RS232_PollComport(char *buf, int size)
{
    int nread = 0;
    int ntries = size;
    int to_read = size;
    ssize_t n;
    while(to_read > 0 && ntries-->0) {
        usleep(10000000*to_read/(unsigned)baudrate);
        n = read(fd, buf+nread, (size_t)to_read);
        if(n<0) {
          if(errno == EAGAIN)
              return nread;
          else
              return (int)n;
        }
        nread += n;
        to_read -= n;
    }
    return nread;
}


int RS232_SendByte(unsigned char byte)
{
  ssize_t n = write(fd, &byte, (int)1);
  usleep(10000000/(unsigned)baudrate);
  if(n < 1)
  {
      return 1;
  }

  return(0);
}


int RS232_SendBuf(unsigned char *buf, int size)
{
  ssize_t n = write(fd, buf, (size_t)size);
  if(n < 0)
  {
    if(errno == EAGAIN)
    {
      return 0;
    }
    else
    {
      return -1;
    }
  }

  return((int)n);
}


void RS232_CloseComport()
{
  int status;
  if(ioctl(fd, TIOCMGET, &status) == -1)
  {
    fprintf(stderr, "unable to get portstatus\n");
  }

  status &= ~TIOCM_DTR;    /* turn off DTR */
  status &= ~TIOCM_RTS;    /* turn off RTS */

  if(ioctl(fd, TIOCMSET, &status) == -1)
  {
    fprintf(stderr, "unable to set portstatus\n");
  }

  tcsetattr(fd, TCSANOW, &old_port_settings);
  close(fd);

  flock(fd, LOCK_UN);  /* free the port so that others can use it. */
}

/*
Constant  Description
TIOCM_LE        DSR (data set ready/line enable)
TIOCM_DTR       DTR (data terminal ready)
TIOCM_RTS       RTS (request to send)
TIOCM_ST        Secondary TXD (transmit)
TIOCM_SR        Secondary RXD (receive)
TIOCM_CTS       CTS (clear to send)
TIOCM_CAR       DCD (data carrier detect)
TIOCM_CD        see TIOCM_CAR
TIOCM_RNG       RNG (ring)
TIOCM_RI        see TIOCM_RNG
TIOCM_DSR       DSR (data set ready)

http://man7.org/linux/man-pages/man4/tty_ioctl.4.html
*/

int RS232_IsDCDEnabled()
{
  int status;

  ioctl(fd, TIOCMGET, &status);

  if(status&TIOCM_CAR) return(1);
  else return(0);
}


int RS232_IsRINGEnabled()
{
  int status;

  ioctl(fd, TIOCMGET, &status);

  if(status&TIOCM_RNG) return(1);
  else return(0);
}


int RS232_IsCTSEnabled()
{
  int status;

  ioctl(fd, TIOCMGET, &status);

  if(status&TIOCM_CTS) return(1);
  else return(0);
}


int RS232_IsDSREnabled()
{
  int status;

  ioctl(fd, TIOCMGET, &status);

  if(status&TIOCM_DSR) return(1);
  else return(0);
}


void RS232_enableDTR()
{
  int status;

  if(ioctl(fd, TIOCMGET, &status) == -1)
  {
    fprintf(stderr, "unable to get portstatus\n");
  }

  status |= TIOCM_DTR;    /* turn on DTR */

  if(ioctl(fd, TIOCMSET, &status) == -1)
  {
    fprintf(stderr, "unable to set portstatus\n");
  }
}


void RS232_disableDTR()
{
  int status;

  if(ioctl(fd, TIOCMGET, &status) == -1)
  {
    fprintf(stderr, "unable to get portstatus\n");
  }

  status &= ~TIOCM_DTR;    /* turn off DTR */

  if(ioctl(fd, TIOCMSET, &status) == -1)
  {
    fprintf(stderr, "unable to set portstatus\n");
  }
}


void RS232_enableRTS()
{
  int status;

  if(ioctl(fd, TIOCMGET, &status) == -1)
  {
    fprintf(stderr, "unable to get portstatus\n");
  }

  status |= TIOCM_RTS;    /* turn on RTS */

  if(ioctl(fd, TIOCMSET, &status) == -1)
  {
    fprintf(stderr, "unable to set portstatus\n");
  }
}


void RS232_disableRTS()
{
  int status;

  if(ioctl(fd, TIOCMGET, &status) == -1)
  {
    fprintf(stderr, "unable to get portstatus\n");
  }

  status &= ~TIOCM_RTS;    /* turn off RTS */

  if(ioctl(fd, TIOCMSET, &status) == -1)
  {
    fprintf(stderr, "unable to set portstatus\n");
  }
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

void RS232_SetFD(unsigned long f)
{
    fd = f;
}

#else  /* windows */

#define RS232_PORTNR  32

HANDLE fd;

char mode_str[128];


int RS232_OpenComport(const char *devname)
{
    char *dev_name = strcat("\\\\.\\", devname);
    fd = CreateFileA(dev_name,
                        GENERIC_READ|GENERIC_WRITE,
                        0,                          /* no share  */
                        NULL,                       /* no security */
                        OPEN_EXISTING,
                        0,                          /* no threads */
                        NULL);                      /* no templates */

    if(fd==INVALID_HANDLE_VALUE)
    {
      printf("unable to open comport\n");
      return(1);
    }
    return(0);
}

int RS232_SetupPort(int bauds, const char *mode, int flowctrl)
{
    baudrate = bauds;
   switch(baudrate)
  {
    case     110 : strcpy(mode_str, "baud=110");
                   break;
    case     300 : strcpy(mode_str, "baud=300");
                   break;
    case     600 : strcpy(mode_str, "baud=600");
                   break;
    case    1200 : strcpy(mode_str, "baud=1200");
                   break;
    case    2400 : strcpy(mode_str, "baud=2400");
                   break;
    case    4800 : strcpy(mode_str, "baud=4800");
                   break;
    case    9600 : strcpy(mode_str, "baud=9600");
                   break;
    case   19200 : strcpy(mode_str, "baud=19200");
                   break;
    case   38400 : strcpy(mode_str, "baud=38400");
                   break;
    case   57600 : strcpy(mode_str, "baud=57600");
                   break;
    case  115200 : strcpy(mode_str, "baud=115200");
                   break;
    case  128000 : strcpy(mode_str, "baud=128000");
                   break;
    case  256000 : strcpy(mode_str, "baud=256000");
                   break;
    case  500000 : strcpy(mode_str, "baud=500000");
                   break;
    case  921600 : strcpy(mode_str, "baud=921600");
                   break;
    case 1000000 : strcpy(mode_str, "baud=1000000");
                   break;
    case 1500000 : strcpy(mode_str, "baud=1500000");
                   break;
    case 2000000 : strcpy(mode_str, "baud=2000000");
                   break;
    case 3000000 : strcpy(mode_str, "baud=3000000");
                   break;
    default      : printf("invalid baudrate\n");
                   return(1);
                   break;
  }

  if(strlen(mode) != 3)
  {
    printf("invalid mode \"%s\"\n", mode);
    return(1);
  }

  switch(mode[0])
  {
    case '8': strcat(mode_str, " data=8");
              break;
    case '7': strcat(mode_str, " data=7");
              break;
    case '6': strcat(mode_str, " data=6");
              break;
    case '5': strcat(mode_str, " data=5");
              break;
    default : printf("invalid number of data-bits '%c'\n", mode[0]);
              return(1);
              break;
  }

  switch(mode[1])
  {
    case 'N':
    case 'n': strcat(mode_str, " parity=n");
              break;
    case 'E':
    case 'e': strcat(mode_str, " parity=e");
              break;
    case 'O':
    case 'o': strcat(mode_str, " parity=o");
              break;
    default : printf("invalid parity '%c'\n", mode[1]);
              return(1);
              break;
  }

  switch(mode[2])
  {
    case '1': strcat(mode_str, " stop=1");
              break;
    case '2': strcat(mode_str, " stop=2");
              break;
    default : printf("invalid number of stop bits '%c'\n", mode[2]);
              return(1);
              break;
  }

  if(flowctrl)
  {
    strcat(mode_str, " xon=off to=off odsr=off dtr=on rts=off");
  }
  else
  {
    strcat(mode_str, " xon=off to=off odsr=off dtr=on rts=on");
  }

/*
http://msdn.microsoft.com/en-us/library/windows/desktop/aa363145%28v=vs.85%29.aspx

http://technet.microsoft.com/en-us/library/cc732236.aspx

https://docs.microsoft.com/en-us/windows/desktop/api/winbase/ns-winbase-_dcb
*/

  DCB port_settings;
  memset(&port_settings, 0, sizeof(port_settings));  /* clear the new struct  */
  port_settings.DCBlength = sizeof(port_settings);

  if(!BuildCommDCBA(mode_str, &port_settings))
  {
    printf("unable to set comport dcb settings\n");
    return(1);
  }

  if(flowctrl)
  {
    port_settings.fOutxCtsFlow = TRUE;
    port_settings.fRtsControl = RTS_CONTROL_HANDSHAKE;
  }

  if(!SetCommState(fd, &port_settings))
  {
    printf("unable to set comport cfg settings\n");
    return(1);
  }

  COMMTIMEOUTS Cptimeouts;

  Cptimeouts.ReadIntervalTimeout         = MAXDWORD;
  Cptimeouts.ReadTotalTimeoutMultiplier  = 0;
  Cptimeouts.ReadTotalTimeoutConstant    = 0;
  Cptimeouts.WriteTotalTimeoutMultiplier = 0;
  Cptimeouts.WriteTotalTimeoutConstant   = 0;

  if(!SetCommTimeouts(fd, &Cptimeouts))
  {
    printf("unable to set comport time-out settings\n");
    return(1);
  }

  return(0);
}

int RS232_AlignFrame(int sof, int maxtries)
{
    char c = 0;
    RS232_flushRX();
    while(c != sof && maxtries-- > 0) {
        RS232_PollComport(&c, 1);
    }
    return 0;
}

int RS232_PollComport(char *buf, int size)
{
  int n;

/* added the void pointer cast, otherwise gcc will complain about */
/* "warning: dereferencing type-punned pointer will break strict aliasing rules" */

  ReadFile(fd, buf, size, (LPDWORD)((void *)&n), NULL);

  return(n);
}


int RS232_SendByte(unsigned char byte)
{
  int n;

  WriteFile(fd, &byte, 1, (LPDWORD)((void *)&n), NULL);

  if(n<0)  return(1);

  return(0);
}


int RS232_SendBuf(unsigned char *buf, int size)
{
  int n;

  if(WriteFile(fd, buf, size, (LPDWORD)((void *)&n), NULL))
  {
    return(n);
  }

  return(-1);
}


void RS232_CloseComport()
{
  CloseHandle(fd);
}

/*
http://msdn.microsoft.com/en-us/library/windows/desktop/aa363258%28v=vs.85%29.aspx
*/

int RS232_IsDCDEnabled()
{
  int status;

  GetCommModemStatus(fd, (LPDWORD)((void *)&status));

  if(status&MS_RLSD_ON) return(1);
  else return(0);
}


int RS232_IsRINGEnabled()
{
  int status;

  GetCommModemStatus(fd, (LPDWORD)((void *)&status));

  if(status&MS_RING_ON) return(1);
  else return(0);
}


int RS232_IsCTSEnabled()
{
  int status;

  GetCommModemStatus(fd, (LPDWORD)((void *)&status));

  if(status&MS_CTS_ON) return(1);
  else return(0);
}


int RS232_IsDSREnabled()
{
  int status;

  GetCommModemStatus(fd, (LPDWORD)((void *)&status));

  if(status&MS_DSR_ON) return(1);
  else return(0);
}


void RS232_enableDTR()
{
  EscapeCommFunction(fd, SETDTR);
}


void RS232_disableDTR()
{
  EscapeCommFunction(fd, CLRDTR);
}


void RS232_enableRTS()
{
  EscapeCommFunction(fd, SETRTS);
}


void RS232_disableRTS()
{
  EscapeCommFunction(fd, CLRRTS);
}

/*
https://msdn.microsoft.com/en-us/library/windows/desktop/aa363428%28v=vs.85%29.aspx
*/

void RS232_flushRX()
{
  PurgeComm(fd, PURGE_RXCLEAR | PURGE_RXABORT);
}


void RS232_flushTX()
{
  PurgeComm(fd, PURGE_TXCLEAR | PURGE_TXABORT);
}


void RS232_flushRXTX()
{
  PurgeComm(fd, PURGE_RXCLEAR | PURGE_RXABORT);
  PurgeComm(fd, PURGE_TXCLEAR | PURGE_TXABORT);
}

void RS232_SetFD(unsigned long f)
{
    fd = (HANDLE)f;
}

#endif

void RS232_cputs(const char *text)  /* sends a string to serial port */
{
  while(*text != 0)   RS232_SendByte(*((const unsigned char*)text++));
}











