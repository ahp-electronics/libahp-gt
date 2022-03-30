/**
*    MIT License
*
*    libahp_gt library to drive the AHP GT controllers
*    Copyright (C) 2021  Ilia Platone
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

#include "ahp_gt.h"
#include "rs232.c"
#include <pthread.h>

#define HEX(c) (int)(((c) < 'A') ? ((c) - '0') : ((c) - 'A') + 10)

typedef enum {
    Ra = 0,
    Dec = 1,
    num_axes = 2,
} SkywatcherAxis;

typedef struct {
    int rs232_polarity;
    int totalsteps[num_axes];
    int wormsteps[num_axes];
    int acceleration_value[num_axes];
    int divider[num_axes];
    int multiplier[num_axes];
    int address_value;
    int dividers;
    int direction_invert[num_axes];
    int stepping_conf[num_axes];
    int stepping_mode[num_axes];
    int version;
    int features[num_axes];
    unsigned char pwmfreq;
    double maxperiod[num_axes];
    double minperiod[num_axes];
    double speed_limit[num_axes];
    double acceleration[num_axes];
    double crown[num_axes];
    double steps[num_axes];
    double motor[num_axes];
    double worm[num_axes];
    double guide[num_axes];
    double maxspeed[num_axes];
    double maxspeed_value[num_axes];
    double accelsteps[num_axes];
    SkywatcherMotionMode motionmode[num_axes];
    SkywatcherAxisStatus axisstatus[num_axes];
    GT1Feature gt1feature[num_axes];
    MountType type;
    GT1Flags mount_flags;
} gt1_info;

static int mutexes_initialized = 0;
static pthread_mutexattr_t mutex_attr;
static pthread_mutex_t mutex;
static char command[32];
static char response[32];
static int dispatch_command(SkywatcherCommand cmd, int axis, int command_arg);
const double SIDEREAL_DAY = 86164.0916000;
static unsigned int ahp_gt_current_device = 0;
static unsigned int ahp_gt_connected = 0;
static unsigned int ahp_gt_detected[128] = { 0 };
static gt1_info devices[128];

static int Revu24str2long(char *s)
{
    int res = 0;
    res = HEX(s[4]);
    res <<= 4;
    res |= HEX(s[5]);
    res <<= 4;
    res |= HEX(s[2]);
    res <<= 4;
    res |= HEX(s[3]);
    res <<= 4;
    res |= HEX(s[0]);
    res <<= 4;
    res |= HEX(s[1]);
    return res;
}

static int Highstr2long(char *s)
{
    int res = 0;
    res = HEX(s[2]);
    res <<= 4;
    res |= HEX(s[0]);
    res <<= 4;
    res |= HEX(s[1]);
    return res;
}

static void long2Revu24str(unsigned int n, char *str)
{
    char hexa[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
    str[0]        = hexa[(n & 0xF0) >> 4];
    str[1]        = hexa[(n & 0x0F)];
    str[2]        = hexa[(n & 0xF000) >> 12];
    str[3]        = hexa[(n & 0x0F00) >> 8];
    str[4]        = hexa[(n & 0xF00000) >> 20];
    str[5]        = hexa[(n & 0x0F0000) >> 16];
    str[6]        = '\0';
}

static int read_eqmod()
{
    int err_code = 0, nbytes_read = 0;
    int max_err = 10;
    // Clear string
    response[0] = '\0';
    unsigned char c = 0;
    while(c != '\r' && err_code < max_err) {
        if(1 == ahp_serial_RecvBuf(&c, 1) && c != 0) {
            if(nbytes_read > 0 && !((c  >= 'A' && c <= 'F') || (c >= '0' && c <= '9') || (c >= 0 && c <= 9) || c == '\r'))
                return -2;
            response[nbytes_read++] = c;
        } else
            err_code++;
    }
    if (err_code == max_err)
    {
        return 0;
    }
    // Remove CR
    response[nbytes_read - 1] = '\0';

    fprintf(stderr, "%s\n", response);

    switch (response[0])
    {
        case '=':
            if(nbytes_read > 2) {
                if(nbytes_read > 5)
                    return Revu24str2long(response+1);
                else
                    return Highstr2long(response+1);
            }
            break;
        case '!':
            return -1;
        default:
        return -1;
    }

    return 0;
}

static int dispatch_command(SkywatcherCommand cmd, int axis, int arg)
{
    int ret = -1;
    int maxtries = 10;
    if(!mutexes_initialized) {
        pthread_mutexattr_init(&mutex_attr);
        pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_ERRORCHECK);
        pthread_mutex_init(&mutex, &mutex_attr);
        mutexes_initialized = 1;
    }
    while(pthread_mutex_trylock(&mutex))
        usleep(100);
    for (unsigned char i = 0; i < maxtries; i++)
    {
        // Clear string
        command[0] = '\0';
        char command_arg[28];
        int n;
        if (arg < 0) {
            snprintf(command, 32, ":%c%c\r", cmd, (char)(axis+'1'));
            n = 4;
        } else {
            arg = (int)fmin((double)0xffffff, arg);
            long2Revu24str((unsigned int)arg, command_arg);
            snprintf(command, 32, ":%c%c%s\r", (char)cmd, (char)(axis+'1'), command_arg);
            n = 10;
        }
        fprintf(stderr, "%s\n", command);

        ahp_serial_flushRXTX();
        if ((ahp_serial_SendBuf((unsigned char*)command, n)) < n)
        {
            if (i == maxtries-1)
            {
                ret = -1;
                break;
            }
            else
            {
                usleep(100);
                continue;
            }
        }
        usleep(10000);

        command[n-1] = '\0';

        ret = read_eqmod();
        if(ret == -2)
            continue;
        break;
    }
    pthread_mutex_unlock(&mutex);
    return ret;
}

static void optimize_values(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    double sidereal_period = SIDEREAL_DAY / devices[ahp_gt_get_current_device()].crown[axis];
    double baseclock = 1500000;
    double usteps = 62.0;
    double maxdiv = 14.0;
    devices[ahp_gt_get_current_device()].wormsteps [axis] = (int)(devices[ahp_gt_get_current_device()].steps [axis] * devices[ahp_gt_get_current_device()].worm [axis] / devices[ahp_gt_get_current_device()].motor [axis]);
    devices[ahp_gt_get_current_device()].totalsteps [axis] = (int)(devices[ahp_gt_get_current_device()].crown [axis] * devices[ahp_gt_get_current_device()].wormsteps [axis]);
    int maxsteps = 0xffffff;
    if(devices[ahp_gt_get_current_device()].stepping_mode[axis] != HalfStep) {
        maxsteps >>= 5;
    }
    double d = 1.0;
    if (ahp_gt_get_mc_version() > 0x30)
        d += fmin(maxdiv, (double)devices[ahp_gt_get_current_device()].totalsteps [axis] / maxsteps);
    devices[ahp_gt_get_current_device()].divider [axis] = floor(d);
    devices[ahp_gt_get_current_device()].multiplier [axis] = 1;
    if(devices[ahp_gt_get_current_device()].stepping_mode[axis] != HalfStep)
        devices[ahp_gt_get_current_device()].multiplier [axis] += (int)(usteps-(d-devices[ahp_gt_get_current_device()].divider [axis])*usteps);
    devices[ahp_gt_get_current_device()].wormsteps [axis] *= (double)devices[ahp_gt_get_current_device()].multiplier [axis] / (double)devices[ahp_gt_get_current_device()].divider [axis];
    devices[ahp_gt_get_current_device()].totalsteps [axis] = (int)(devices[ahp_gt_get_current_device()].crown [axis] * devices[ahp_gt_get_current_device()].wormsteps [axis]);

    devices[ahp_gt_get_current_device()].maxperiod [axis] = (int)sidereal_period;
    devices[ahp_gt_get_current_device()].speed_limit [axis] = (int)(800);
    devices[ahp_gt_get_current_device()].minperiod [axis] = 1;
    devices[ahp_gt_get_current_device()].maxspeed [axis] = fmin(devices[ahp_gt_get_current_device()].speed_limit [axis], devices[ahp_gt_get_current_device()].maxspeed [axis]);
    devices[ahp_gt_get_current_device()].maxspeed_value [axis] = (int)fmax(devices[ahp_gt_get_current_device()].minperiod [axis], (devices[ahp_gt_get_current_device()].multiplier [axis] * devices[ahp_gt_get_current_device()].maxperiod [axis] / devices[ahp_gt_get_current_device()].maxspeed [axis]));
    devices[ahp_gt_get_current_device()].guide [axis] = (int)(SIDEREAL_DAY * baseclock / devices[ahp_gt_get_current_device()].totalsteps [axis]);

    double accel = devices[ahp_gt_get_current_device()].acceleration [axis] / (M_PI * 2.0);
    double degrees = (int)(accel * (double)devices[ahp_gt_get_current_device()].totalsteps [axis] / (double)devices[ahp_gt_get_current_device()].multiplier [axis]);
    for (devices[ahp_gt_get_current_device()].acceleration_value [axis] = 0; devices[ahp_gt_get_current_device()].acceleration_value [axis] < 63 && degrees > 0; devices[ahp_gt_get_current_device()].acceleration_value [axis]++, degrees -= devices[ahp_gt_get_current_device()].acceleration_value [axis] * devices[ahp_gt_get_current_device()].divider [axis])
        ;
    devices[ahp_gt_get_current_device()].accelsteps [axis] = 0;
    if (devices[ahp_gt_get_current_device()].acceleration_value [axis] > 0) {
        devices[ahp_gt_get_current_device()].accelsteps [axis] = (int)fmax (1, fmin (0xff, devices[ahp_gt_get_current_device()].guide [axis] * devices[ahp_gt_get_current_device()].acceleration_value [axis]));
    }
    if (devices[ahp_gt_get_current_device()].version > 0x30) {
        devices[ahp_gt_get_current_device()].address_value &= 0x7f;
        devices[ahp_gt_get_current_device()].rs232_polarity &= 0x1;
        devices[ahp_gt_get_current_device()].dividers = devices[ahp_gt_get_current_device()].rs232_polarity | ((unsigned char)devices[ahp_gt_get_current_device()].divider [0] << 1) | (((unsigned char)devices[ahp_gt_get_current_device()].divider [1]) << 5) | (devices[ahp_gt_get_current_device()].address_value << 9);
    }
}

int Check(int pos)
{
    int ret = -1;
    int nchecks = 10;
    int ntries = 10;
    while (ret < 0 && nchecks-- > 0)
    {
        while (ret < 0 && ntries-- > 0)
        {
            ret = dispatch_command(GetVars, pos, -1);
            if(ret > -1)
                nchecks = 0;
        }
    }
    return ret;
}

int WriteAndCheck(int axis, int pos, int val)
{
     int ret = 0;
     int nchecks = 10;
     int ntries = 10;
     while (!ret && nchecks-- > 0)
     {
         while (!ret && ntries-- > 0)
         {
             ret = dispatch_command(FlashEnable, axis, -1);
             if (ret>-1)
             {
                 ret = dispatch_command(SetVars, pos, val) == 0;
                 if (ret>-1)
                 {
                     ret = Check(pos) == val;
                     if (ret)
                         nchecks = 0;
                 }
             }
         }
     }
     return ret;
}

void ahp_gt_write_values(int axis, int *percent, int *finished)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    int offset = axis * 8;
    *finished = 0;
    if (!WriteAndCheck (axis, offset + 0, devices[ahp_gt_get_current_device()].totalsteps [axis])) {
        *finished = -1;
        return;
    }
    *percent = axis * 50 + 6.25;
    if (!WriteAndCheck (axis, offset + 1, devices[ahp_gt_get_current_device()].wormsteps [axis])) {
        *finished = -1;
        return;
    }
    *percent = axis * 50 + 12.5;
    if (!WriteAndCheck (axis, offset + 2, devices[ahp_gt_get_current_device()].maxspeed_value [axis])) {
        *finished = -1;
        return;
    }
    *percent = axis * 50 + 18.75;
    if (!WriteAndCheck (axis, offset + 3, devices[ahp_gt_get_current_device()].guide [axis])) {
        *finished = -1;
        return;
    }
    *percent = axis * 50 + 25;
    if (!WriteAndCheck (axis, offset + 4, devices[ahp_gt_get_current_device()].wormsteps [axis])) {
        *finished = -1;
        return;
    }
    *percent = axis * 50 + 32.25;
    if (!WriteAndCheck (axis, offset + 5, ((int)devices[ahp_gt_get_current_device()].acceleration_value [axis] << 18) | ((int)devices[ahp_gt_get_current_device()].accelsteps [axis] << 10) | (((int)devices[ahp_gt_get_current_device()].multiplier [axis] & 0x7f) << 3) | ((devices[ahp_gt_get_current_device()].stepping_conf[axis] & 0x03) << 1) | (devices[ahp_gt_get_current_device()].direction_invert[axis] & 1))) {
        *finished = -1;
        return;
    }
    *percent = axis * 50 + 38.5;
    if (!WriteAndCheck (axis, offset + 6, (int)devices[ahp_gt_get_current_device()].features [axis])) {
        *finished = -1;
        return;
    }
    *percent = axis * 50 + 44.75;
    if (!WriteAndCheck (axis, offset + 7, ((((0xf-devices[ahp_gt_get_current_device()].pwmfreq) << 4) >> (2 * axis)) & 0x30) | ((int)devices[ahp_gt_get_current_device()].stepping_mode[axis] << 6) | (((devices[ahp_gt_get_current_device()].mount_flags >> axis)&1) << 3) | ((int)devices[ahp_gt_get_current_device()].gt1feature[axis] & 7) | (axis == 0?(((unsigned char)devices[ahp_gt_get_current_device()].type)<<16):((devices[ahp_gt_get_current_device()].mount_flags&0x3fc)<<14)) | (int)(((devices[ahp_gt_get_current_device()].dividers>>(8*axis))&0xff)<<8))) {
        *finished = -1;
        return;
    }
    *percent = axis * 50 + 50;
    dispatch_command (ReloadVars, axis, -1);
    *finished = 1;
}

void ahp_gt_read_values(int axis)
{
    if(!ahp_gt_is_connected())
        return;
    int offset = axis * 8;
    devices[ahp_gt_get_current_device()].totalsteps [axis] = dispatch_command(GetVars, offset + 0, -1);
    devices[ahp_gt_get_current_device()].wormsteps [axis] = dispatch_command(GetVars, offset + 1, -1);
    devices[ahp_gt_get_current_device()].maxspeed [axis] = dispatch_command(GetVars, offset + 2, -1);
    devices[ahp_gt_get_current_device()].guide [axis] = dispatch_command(GetVars, offset + 3, -1);
    int tmp = dispatch_command(GetVars, offset + 5, -1);
    devices[ahp_gt_get_current_device()].acceleration_value [axis] = ((tmp >> 18) & 0x3f)*64;
    devices[ahp_gt_get_current_device()].accelsteps [axis] =  (tmp >> 18) & 0xff;
    devices[ahp_gt_get_current_device()].multiplier [axis] = (tmp >> 3) & 0x7f;
    devices[ahp_gt_get_current_device()].direction_invert [axis] = (tmp >> 2) & 0x1;
    devices[ahp_gt_get_current_device()].stepping_conf [axis] = (tmp & 0x06)>>1;
    devices[ahp_gt_get_current_device()].features [axis] = dispatch_command(GetVars, offset + 6, -1);
    devices[ahp_gt_get_current_device()].gt1feature[axis] = dispatch_command(GetVars, offset + 7, -1) & 0x7;
    devices[ahp_gt_get_current_device()].stepping_mode[axis] = (dispatch_command(GetVars, offset + 7, -1) >> 6) & 0x03;
    devices[ahp_gt_get_current_device()].pwmfreq = (dispatch_command(GetVars, 7, -1) >> 4) & 0x3;
    devices[ahp_gt_get_current_device()].pwmfreq |= (dispatch_command(GetVars, 15, -1) >> 2) & 0xc;
    devices[ahp_gt_get_current_device()].pwmfreq = 15-devices[ahp_gt_get_current_device()].pwmfreq;
    devices[ahp_gt_get_current_device()].type = (dispatch_command(GetVars, offset + 7, -1) >> 16) & 0xff;
    devices[ahp_gt_get_current_device()].mount_flags = (dispatch_command(GetVars, 7, -1) & 0x8) >> 3;
    devices[ahp_gt_get_current_device()].mount_flags |= (dispatch_command(GetVars, 15, -1) & 0x8) >> 2;
    devices[ahp_gt_get_current_device()].dividers = (dispatch_command(GetVars, 7, -1) >> 8) & 0xff;
    devices[ahp_gt_get_current_device()].dividers |= dispatch_command(GetVars, 15, -1) & 0xff00;
    devices[ahp_gt_get_current_device()].divider[axis] = (devices[ahp_gt_get_current_device()].dividers >> (1+axis*4)) & 0xf;
    devices[ahp_gt_get_current_device()].address_value = (devices[ahp_gt_get_current_device()].dividers >> 9) & 0x7f;
    devices[ahp_gt_get_current_device()].rs232_polarity = devices[ahp_gt_get_current_device()].dividers & 1;
    optimize_values(axis);
}

int ahp_gt_connect_fd(int fd)
{
    if(ahp_gt_is_connected())
        return 0;
    if(fd != -1) {
        ahp_serial_SetFD(fd, 9600);
        ahp_gt_connected = 1;
        if(!ahp_gt_detect_device(ahp_gt_get_current_device())) {
            ahp_gt_get_mc_version();
            if(devices[ahp_gt_get_current_device()].version > 0) {
                fprintf(stderr, "MC Version: %02X\n", devices[ahp_gt_get_current_device()].version);
                return 0;
            }
        }
    }
    return 1;
}

int ahp_gt_get_fd()
{
    if(!ahp_gt_is_connected())
        return -1;
    return ahp_serial_GetFD();
}

int ahp_gt_connect(const char* port)
{
    if(ahp_gt_is_connected())
        return 0;
    if(!ahp_serial_OpenComport(port)) {
        if(!ahp_serial_SetupPort(9600, "8N1", 0)) {
            ahp_gt_connected = 1;
            if(!ahp_gt_detect_device()) {
                ahp_gt_get_mc_version();
                if(devices[ahp_gt_get_current_device()].version > 0) {
                    fprintf(stderr, "MC Version: %02X\n", devices[ahp_gt_get_current_device()].version);
                    return 0;
                }
            }
        }
        ahp_serial_CloseComport();
    }
    return 1;
}

void ahp_gt_disconnect()
{
    if(ahp_gt_is_connected()) {
        ahp_serial_CloseComport();
        if(mutexes_initialized) {
            pthread_mutex_unlock(&mutex);
            pthread_mutex_destroy(&mutex);
            pthread_mutexattr_destroy(&mutex_attr);
            mutexes_initialized = 0;
        }
        ahp_gt_connected = 0;
    }
}

unsigned int ahp_gt_is_connected()
{
    return ahp_gt_connected;
}

unsigned int ahp_gt_is_detected(int index)
{
    return ahp_gt_detected[index];
}

int ahp_gt_get_mc_version()
{
    int v = dispatch_command(InquireMotorBoardVersion, 0, -1);
    v >>= 8;
    v &= 0xff;
    if (v < 0x24 || v == 0xff)
        v = -1;
    devices[ahp_gt_get_current_device()].version = v;
    return devices[ahp_gt_get_current_device()].version;
}

MountType ahp_gt_get_mount_type()
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0;
    return devices[ahp_gt_get_current_device()].type;
}

GT1Feature ahp_gt_get_feature(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0;
    return devices[ahp_gt_get_current_device()].gt1feature[axis];
}

SkywatcherFeature ahp_gt_get_features(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0;
    return (SkywatcherFeature)devices[ahp_gt_get_current_device()].features[axis];
}

double ahp_gt_get_motor_steps(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0.0;
    return devices[ahp_gt_get_current_device()].steps[axis];
}

double ahp_gt_get_motor_teeth(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0.0;
    return devices[ahp_gt_get_current_device()].motor[axis];
}

double ahp_gt_get_worm_teeth(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0.0;
    return devices[ahp_gt_get_current_device()].worm[axis];
}

double ahp_gt_get_crown_teeth(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0.0;
    return devices[ahp_gt_get_current_device()].crown[axis];
}

double ahp_gt_get_multiplier(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0.0;
    return devices[ahp_gt_get_current_device()].multiplier[axis];
}

double ahp_gt_get_divider(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0.0;
    return devices[ahp_gt_get_current_device()].divider[axis];
}

int ahp_gt_get_totalsteps(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0;
    return devices[ahp_gt_get_current_device()].totalsteps[axis];
}

int ahp_gt_get_wormsteps(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0;
    return devices[ahp_gt_get_current_device()].wormsteps[axis];
}

double ahp_gt_get_guide_steps(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0.0;
    return devices[ahp_gt_get_current_device()].guide[axis];
}

double ahp_gt_get_acceleration_steps(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0.0;
    return devices[ahp_gt_get_current_device()].accelsteps[axis];
}

double ahp_gt_get_acceleration_angle(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0.0;
    return devices[ahp_gt_get_current_device()].acceleration[axis];
}

int ahp_gt_get_rs232_polarity()
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0;
    return devices[ahp_gt_get_current_device()].rs232_polarity;
}

int ahp_gt_get_pwm_frequency()
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0;
    return devices[ahp_gt_get_current_device()].pwmfreq;
}

int ahp_gt_get_direction_invert(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0;
    return devices[ahp_gt_get_current_device()].direction_invert[axis];
}

GT1Flags ahp_gt_get_mount_flags()
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0;
    return devices[ahp_gt_get_current_device()].mount_flags;
}

GT1SteppingConfiguration ahp_gt_get_stepping_conf(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0;
    return (GT1SteppingConfiguration)devices[ahp_gt_get_current_device()].stepping_conf[axis];
}

GT1SteppingMode ahp_gt_get_stepping_mode(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0;
    return (GT1SteppingMode)devices[ahp_gt_get_current_device()].stepping_mode[axis];
}

double ahp_gt_get_max_speed(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0.0;
    return devices[ahp_gt_get_current_device()].maxspeed[axis];
}

double ahp_gt_get_speed_limit(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0.0;
    return devices[ahp_gt_get_current_device()].speed_limit[axis];
}

void ahp_gt_set_mount_type(MountType value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].type = value;
}

void ahp_gt_set_features(int axis, SkywatcherFeature value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].features[axis] &= ~value;
    devices[ahp_gt_get_current_device()].features[axis] |= value;
}

void ahp_gt_set_feature(int axis, GT1Feature value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].gt1feature[axis] = value & 7;
    optimize_values(axis);
}

void ahp_gt_set_motor_steps(int axis, double value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].steps[axis] = value;
    optimize_values(axis);
}

void ahp_gt_set_motor_teeth(int axis, double value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].motor[axis] = value;
    optimize_values(axis);
}

void ahp_gt_set_worm_teeth(int axis, double value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].worm[axis] = value;
    optimize_values(axis);
}

void ahp_gt_set_crown_teeth(int axis, double value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].crown[axis] = value;
    optimize_values(axis);
}

void ahp_gt_set_guide_steps(int axis, double value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].guide[axis] = value;
    optimize_values(axis);
}

void ahp_gt_set_pwm_frequency(int value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    value = fmin(0xb, fmax(0, value));
    devices[ahp_gt_get_current_device()].pwmfreq = value;
}

void ahp_gt_set_acceleration_angle(int axis, double value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].acceleration[axis] = value;
    optimize_values(axis);
}

void ahp_gt_set_rs232_polarity(int value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].rs232_polarity = value&1;
    optimize_values(0);
}

void ahp_gt_set_direction_invert(int axis, int value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].direction_invert[axis] = value&1;
}

void ahp_gt_set_mount_flags(GT1Flags value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].mount_flags = value;
}

void ahp_gt_set_stepping_conf(int axis, GT1SteppingConfiguration value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].stepping_conf[axis] = (int)value;
}

void ahp_gt_set_stepping_mode(int axis, GT1SteppingMode value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].stepping_mode[axis] = (int)value;
    optimize_values(axis);
}

void ahp_gt_set_max_speed(int axis, double value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].maxspeed[axis] = fabs(value);
    optimize_values(axis);
}

void ahp_gt_set_divider(int axis, int value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].divider[axis] = abs(value);
    optimize_values(axis);
}

void ahp_gt_set_multiplier(int axis, int value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].multiplier[axis] = abs(value);
}

void ahp_gt_set_totalsteps(int axis, int value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].totalsteps[axis] = abs(value);
}

void ahp_gt_set_wormsteps(int axis, int value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].wormsteps[axis] = abs(value);
}

int ahp_gt_get_current_device() {
    return ahp_gt_current_device&0x7f;
}

int ahp_gt_detect_device() {
    if(!ahp_gt_is_connected())
        return -1;
    ahp_gt_detected[ahp_gt_get_current_device()] = 0;
    dispatch_command(SetAddress, 0, ahp_gt_current_device);
    if(ahp_gt_get_mc_version() > 0) {
        ahp_gt_read_values(Ra);
        ahp_gt_read_values(Dec);
        ahp_gt_detected[ahp_gt_get_current_device()] = 1;
        return 0;
    }
    return -1;
}

void ahp_gt_select_device(int address) {
    ahp_gt_current_device = address&0x7f;
    dispatch_command(SetAddress, 0, ahp_gt_current_device);
}

void ahp_gt_set_address(int address)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].address_value = address;
}

int ahp_gt_get_address()
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0;
    return devices[ahp_gt_get_current_device()].address_value;
}

SkywatcherAxisStatus ahp_gt_get_status(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return (SkywatcherAxisStatus){ 0, 0, 0, 0, 0 };
    SkywatcherAxisStatus status;
    int response = dispatch_command(GetAxisStatus, axis, -1);

    status.Initialized = (response & 0x100);
    status.Running     = (response & 0x1);
    if (response & 0x10)
        status.Mode = MODE_SLEW;
    else
        status.Mode = MODE_GOTO;
    if (response & 0x20)
        status.Direction = DIRECTION_BACKWARD;
    else
        status.Direction = DIRECTION_FORWARD;
    if (response & 0x40)
        status.Speed = SPEED_HIGH;
    else
        status.Speed = SPEED_LOW;
    devices[ahp_gt_get_current_device()].axisstatus[axis] = status;
    return status;
}

void ahp_gt_set_position(int axis, double value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    dispatch_command(SetAxisPositionCmd, axis, (int)(value*devices[ahp_gt_get_current_device()].totalsteps[axis]/M_PI/2.0)+0x800000);
}

double ahp_gt_get_position(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0;
    int steps = dispatch_command(GetAxisPosition, axis, -1)-8388608;
    return (double)steps*M_PI*2.0/(double)devices[ahp_gt_get_current_device()].totalsteps[axis];
}

int ahp_gt_is_axis_moving(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0;
    return devices[ahp_gt_get_current_device()].axisstatus[axis].Running;
}

void ahp_gt_goto_absolute(int axis, double target, double speed) {
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    double position = ahp_gt_get_position(axis);
    speed = fabs(speed);
    speed *= (target-position < 0 ? -1 : 1);
    double max = devices[ahp_gt_get_current_device()].totalsteps[axis];
    target /= M_PI*2;
    target *= max;
    target += (double)0x800000;
    double maxperiod = SIDEREAL_DAY * devices[ahp_gt_get_current_device()].wormsteps[axis] / devices[ahp_gt_get_current_device()].totalsteps[axis];
    int period = maxperiod * devices[ahp_gt_get_current_device()].multiplier[axis];
    SkywatcherMotionMode mode = MODE_GOTO_HISPEED;
    if(fabs(speed) < 128.0) {
        mode = MODE_GOTO_LOSPEED;
        period /= devices[ahp_gt_get_current_device()].multiplier[axis];
    }
    mode |= (speed < 0 ? 1 : 0);
    period /= fabs(speed);
    period = fmax(devices[ahp_gt_get_current_device()].minperiod[axis], period);
    ahp_gt_stop_motion(axis, 1);
    devices[ahp_gt_get_current_device()].motionmode[axis] = mode;
    dispatch_command (Initialize, axis, -1);
    dispatch_command (ActivateMotor, axis, -1);
    dispatch_command (SetGotoTarget, axis, target);
    dispatch_command (SetStepPeriod, axis, period);
    dispatch_command (SetMotionMode, axis, mode);
    dispatch_command (StartMotion, axis, -1);
}

void ahp_gt_goto_relative(int axis, double increment, double speed) {
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    speed = fabs(speed);
    speed *= (increment < 0 ? -1 : 1);
    double max = devices[ahp_gt_get_current_device()].totalsteps[axis];
    increment /= M_PI*2;
    increment *= max;
    double maxperiod = SIDEREAL_DAY * devices[ahp_gt_get_current_device()].wormsteps[axis] / devices[ahp_gt_get_current_device()].totalsteps[axis];
    int period = maxperiod * devices[ahp_gt_get_current_device()].multiplier[axis];
    SkywatcherMotionMode mode = MODE_GOTO_HISPEED;
    if(fabs(speed) < 128.0) {
        mode = MODE_GOTO_LOSPEED;
        period /= devices[ahp_gt_get_current_device()].multiplier[axis];
    }
    mode |= (speed < 0 ? 1 : 0);
    period /= fabs(speed);
    ahp_gt_stop_motion(axis, 1);
    devices[ahp_gt_get_current_device()].motionmode[axis] = mode;
    dispatch_command (Initialize, axis, -1);
    dispatch_command (ActivateMotor, axis, -1);
    dispatch_command (SetGotoTargetIncrement, axis, (int)fabs(increment));
    dispatch_command (SetStepPeriod, axis, period);
    dispatch_command (SetMotionMode, axis, mode);
    dispatch_command (StartMotion, axis, -1);
}

void ahp_gt_start_motion(int axis, double speed) {
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    double period = SIDEREAL_DAY * devices[ahp_gt_get_current_device()].multiplier[axis] * devices[ahp_gt_get_current_device()].wormsteps[axis] / devices[ahp_gt_get_current_device()].totalsteps[axis];
    SkywatcherMotionMode mode = MODE_SLEW_HISPEED;
    if(fabs(speed) < 128.0) {
        mode = MODE_SLEW_LOSPEED;
        period /= devices[ahp_gt_get_current_device()].multiplier[axis];
    }
    mode |= (speed < 0 ? 1 : 0);
    period /= fabs(speed);
    ahp_gt_stop_motion(axis, 1);
    devices[ahp_gt_get_current_device()].motionmode[axis] = mode;
    dispatch_command (Initialize, axis, -1);
    dispatch_command (ActivateMotor, axis, -1);
    dispatch_command (SetStepPeriod, axis, period);
    dispatch_command (SetMotionMode, axis, mode);
    dispatch_command (StartMotion, axis, -1);
}

void ahp_gt_stop_motion(int axis, int wait) {
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    dispatch_command(InstantAxisStop, axis, -1);
    if(wait) {
        while (ahp_gt_is_axis_moving(axis))
            usleep(100);
    }
}

void ahp_gt_start_tracking(int axis) {
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    ahp_gt_stop_motion(axis, 1);
    double period = SIDEREAL_DAY * devices[ahp_gt_get_current_device()].wormsteps[axis] / devices[ahp_gt_get_current_device()].totalsteps[axis];
    dispatch_command (SetStepPeriod, axis, period);
    dispatch_command (Initialize, axis, -1);
    dispatch_command (ActivateMotor, axis, -1);
    dispatch_command (SetMotionMode, axis, 0x10);
    dispatch_command (StartMotion, axis, -1);
}
