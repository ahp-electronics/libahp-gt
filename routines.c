/*
*    MIT License
*
*    AHP common routines
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

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#ifdef AHP_DEBUG
int ahp_debug = 0;
char* ahp_app_name = NULL;
FILE *out = NULL;
FILE *err = NULL;
/**
* \brief log a message to the error or output streams
* \param x The log level
* \param str The string to print
*/
void ahp_print(int x, char* str);

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
#else
#define pinfo(...) fprintf(stdout, __VA_ARGS__)
#define perr(...) fprintf(stderr, __VA_ARGS__)
#define pwarn(...) fprintf(stderr, __VA_ARGS__)
#define pgarb(...) fprintf(stderr, __VA_ARGS__)
#endif