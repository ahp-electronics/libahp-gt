#ifndef RS232_H
#define RS232_H

#ifdef  __cplusplus
extern "C" {
#endif
#ifdef _WIN32
#include <winsock2.h>
#include <windows.h>
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT extern
#endif

#include <stdio.h>
#include <stdlib.h>
#include <serial.h>

DLL_EXPORT void serial_connect(char* port, int baudrate, const char *mode);
DLL_EXPORT int serial_is_open();
DLL_EXPORT int serial_write(char* buf, int len);
DLL_EXPORT int serial_read(char* buf, int len);
DLL_EXPORT void serial_flush();
DLL_EXPORT void serial_flush_rx();
DLL_EXPORT void serial_flush_tx();
DLL_EXPORT void serial_set_fd(int fd, int baudrate);
DLL_EXPORT int serial_get_fd();
DLL_EXPORT void serial_close();

#ifdef  __cplusplus
}
#endif
#endif // RS232_H
