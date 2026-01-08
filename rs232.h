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
#include <string.h>
#include <math.h>
#include <unistd.h>

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
#undef UNICODE
#undef _UNICODE
#include <winsock2.h>
#include <windows.h>
#include <ws2tcpip.h>
#include <winsock.h>
#endif
#include <pthread.h>

#ifdef AHP_DEBUG
static int ahp_debug = 0;
static char* ahp_app_name = NULL;
static FILE *out = NULL;
static FILE *err = NULL;
/**
* \brief log a message to the error or output streams
* \param x The log level
* \param str The string to print
*/
extern void ahp_print(int x, char* str);

void ahp_set_stdout(FILE *f)
{
    out = f;
}

void ahp_set_stderr(FILE *f)
{
    err = f;
}

void ahp_set_debug_level(int value)
{
    ahp_debug = value;
}

void ahp_set_app_name(char* name)
{
    ahp_app_name = name;
}

int ahp_get_debug_level()
{
    return ahp_debug;
}

char* ahp_get_app_name()
{
    return ahp_app_name;
}

void ahp_print(int x, char* str)
{
    if(x == 0 && out != NULL)
        fprintf(out, "%s", str);
    else if(x <= ahp_get_debug_level() && err != NULL)
        fprintf(err, "%s", str);
}

#define pdbg(x, ...) ({ \
char str[500]; \
struct timespec ts; \
time_t t = time(NULL); \
struct tm tm = *localtime(&t); \
clock_gettime(CLOCK_REALTIME, &ts); \
sprintf(str, "[%04d-%02d-%02dT%02d:%02d:%02d.%03ld ", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec/1000000); \
switch(x) { \
    case AHP_DEBUG_ERROR: \
    sprintf(&str[strlen(str)], "ERRO]"); \
        break; \
    case AHP_DEBUG_WARNING: \
    sprintf(&str[strlen(str)], "WARN]"); \
        break; \
    case AHP_DEBUG_DEBUG: \
    sprintf(&str[strlen(str)], "DEBG]"); \
        break; \
    default: \
    sprintf(&str[strlen(str)], "INFO]"); \
        break; \
} \
if(ahp_get_app_name() != NULL) \
    sprintf(&str[strlen(str)], "[%s]", ahp_get_app_name()); \
sprintf(&str[strlen(str)], " "); \
sprintf(&str[strlen(str)], __VA_ARGS__); \
ahp_print(x, str); \
})
#define pinfo(...) pdbg(AHP_DEBUG_INFO, __VA_ARGS__)
#define perr(...) pdbg(AHP_DEBUG_ERROR, __VA_ARGS__)
#define pwarn(...) pdbg(AHP_DEBUG_WARNING, __VA_ARGS__)
#define pgarb(...) pdbg(AHP_DEBUG_DEBUG, __VA_ARGS__)
#define pfunc pgarb("%s\n", __func__)
#define start_gettime
#define end_gettime
#endif

DLL_EXPORT void serial_connect(char* port, int baudrate, int stopbits, int parity, int bytesize);
DLL_EXPORT int serial_is_open();
DLL_EXPORT int serial_write(char* buf, int len);
DLL_EXPORT int serial_read(char* buf, int len);
DLL_EXPORT void serial_flush();
DLL_EXPORT void serial_flush_rx();
DLL_EXPORT void serial_flush_tx();
DLL_EXPORT void serial_set_fd(int fd);
DLL_EXPORT int serial_get_fd();
DLL_EXPORT void serial_close();

#ifdef  __cplusplus
}
#endif
#endif // RS232_H
