#include "ahp_gt.h"
#include "rs232.h"

#define HEX(c) (unsigned int)(((c) < 'A') ? ((c) - '0') : ((c) - 'A') + 10)

typedef enum {
    Ra = 0,
    Dec = 1,
    num_axes = 2,
} SkywatcherAxis;

static char command[32];
static char response[32];
static int dispatch_command(SkywatcherCommand cmd, int axis, int command_arg);

MountType type = isEQ8;
const double SIDEREAL_DAY = 86164.0916000;
const double SOLAR_DAY = 86400.0;
int totalsteps[num_axes] = { 200 * 64 * 40 / 10 * 180, 200 * 64 * 40 / 10 * 180 };
int wormsteps[num_axes] = { 200 * 64 * 40 / 10, 200 * 64 * 40 / 10 };
double maxperiod[num_axes] = { 64, 64 };
double speed_limit[num_axes] = { 1000, 1000 };
double acceleration_min[num_axes] = { 1, 1 };
double acceleration[num_axes] = { 1, 1 };
double acceleration_value[num_axes] = { 1, 1 };
double microsteps[num_axes] = { 64, 64 };
double crown[num_axes] = { 180, 180 };
double steps[num_axes] = { 200, 200 };
double motor[num_axes] = { 10, 10 };
double worm[num_axes] = { 40, 40 };
double guide[num_axes] = { 1, 1 };
int direction_invert[num_axes] = { 0, 0 };
int stepping[num_axes] = { 0, 0 };
int version = 0;
double maxspeed[num_axes] = { 1000, 1000 };
double maxspeed_value[num_axes] = { 500, 500 };
int features[num_axes] = { hasPPEC, hasPPEC };
GT1Feature gt1feature[num_axes] = { GpioUnused, GpioUnused };
double accelsteps[num_axes]  = { 1, 1 };


static int Revu24str2long(char *s)
{
    unsigned int res = 0;
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
    unsigned int res = 0;
    res = HEX(s[0]);
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

    // Clear string
    response[0] = '\0';
    char c = 0;
    while(c != '\r' && err_code < 10) {
        if(1 == RS232_PollComport((unsigned char*)&c, 1))
            response[nbytes_read++] = c;
        else
            err_code++;
    }
    if (err_code == 10)
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
    for (unsigned char i = 0; i < 10; i++)
    {
        // Clear string
        command[0] = '\0';
        char command_arg[32];
        unsigned long n;
        if (arg < 0) {
            snprintf(command, 32, ":%c%c\r", cmd, (char)(axis+'1'));
            n = 4;
        } else {
            long2Revu24str((unsigned int)arg, command_arg);
            snprintf(command, 32, ":%c%c%s\r", (char)cmd, (char)(axis+'1'), command_arg);
            n = 10;
        }
        fprintf(stderr, "%s\n", command);

        RS232_flushRXTX();
        if ((RS232_SendBuf((unsigned char*)command, n)) < n)
        {
            if (i == 9)
            {
                return -1;
            }
            else
            {
                usleep(100000);
                continue;
            }
        }
        usleep(100000);

        command[n-1] = '\0';

        return read_eqmod();
    }

    return -1;
}

void optimize_values(int axis)
{
    double baseclock = 375000;
    double steps0 = steps [axis] * worm [axis] / motor [axis];
    int maxsteps = 0xffffff;
    wormsteps [axis] = (double)maxsteps / crown [axis];
    microsteps [axis] = wormsteps [axis] / steps0;
    microsteps [axis] = microsteps[axis] < 8 ? 8 : (microsteps[axis] < 63 ? microsteps[axis] : 63);
    maxperiod [axis] = (SIDEREAL_DAY / crown [axis]);
    maxperiod [axis] /= 50;
    maxperiod [axis] = floor(maxperiod [axis]);
    maxperiod [axis] *= 50;
    speed_limit [axis] = (maxperiod [axis] / 25);
    wormsteps [axis] = (microsteps [axis] * steps0);
    totalsteps [axis] = crown [axis] * wormsteps [axis];
    double speedx = (maxspeed_value [axis] * maxperiod [axis]) / speed_limit [axis];
    speedx = speedx < 1 ? 2 : speedx * 2;
    maxspeed [axis] = (maxperiod [axis] * microsteps [axis] / speedx);
    maxspeed [axis] ++;
    guide [axis] = (SIDEREAL_DAY * baseclock / totalsteps[axis]);
    guide [axis] = (int)(SIDEREAL_DAY * baseclock / totalsteps[axis]);
    double degrees = (((64-acceleration_value [axis]) * totalsteps[axis] / 63) / microsteps[axis] / 180.0);
    for(acceleration [axis] = 0; acceleration [axis] < 63 && degrees > 0; acceleration [axis]++, degrees -= acceleration [axis]);
    accelsteps[axis] = 0;
    if(acceleration[axis] > 0) {
        accelsteps[axis] = (guide [axis] / acceleration [axis]);
        accelsteps[axis] = accelsteps[axis] < 1 ? 1 : accelsteps[axis];
        accelsteps[axis] = accelsteps[axis] > 255 ? 255 : accelsteps[axis];
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
    int offset = axis * 8;
    *finished = 0;
    if (!WriteAndCheck (axis, offset + 0, totalsteps [axis])) {
        *finished = -1;
        return;
    }
    *percent += 100 / 8 / num_axes;
    if (!WriteAndCheck (axis, offset + 1, wormsteps [axis])) {
        *finished = -1;
        return;
    }
    *percent += 100 / 8 / num_axes;
    if (!WriteAndCheck (axis, offset + 2, 0xffffff < maxspeed [axis] ? 0xffffff : maxspeed [axis])) {
        *finished = -1;
        return;
    }
    *percent += 100 / 8 / num_axes;
    if (!WriteAndCheck (axis, offset + 3, guide [axis])) {
        *finished = -1;
        return;
    }
    *percent += 100 / 8 / num_axes;
    if (!WriteAndCheck (axis, offset + 4, wormsteps [axis])) {
        *finished = -1;
        return;
    }
    *percent += 100 / 8 / num_axes;
    if (!WriteAndCheck (axis, offset + 5, (((unsigned short)acceleration [axis] < 0x3f ? (unsigned short)acceleration [axis] : 0x3f) << 18) | (((unsigned short)accelsteps [axis] < 0xff ? (unsigned short)accelsteps [axis] : 0xff) << 10) | (((unsigned short)microsteps [axis] & 0x7f) << 3) | ((stepping [axis] & 0x03) << 1) | (direction_invert[axis] & 1))) {
        *finished = -1;
        return;
    }
    *percent += 100 / 8 / num_axes;
    if (!WriteAndCheck (axis, offset + 6, (int)features [axis])) {
        *finished = -1;
        return;
    }
    *percent += 100 / 8 / num_axes;
    if (!WriteAndCheck (axis, offset + 7, (ushort)gt1feature[axis] | ((unsigned char)type)<<16)) {
        *finished = -1;
        return;
    }
    *percent += 100 / 8 / num_axes;
    dispatch_command (ReloadVars, axis, -1);
}

void ahp_gt_read_values(int axis)
{
    int offset = axis * 8;
    totalsteps [axis] = dispatch_command(GetVars, offset + 0, -1);
    wormsteps [axis] = dispatch_command(GetVars, offset + 1, -1);
    maxspeed [axis] = dispatch_command(GetVars, offset + 2, -1);
    guide [axis] = dispatch_command(GetVars, offset + 3, -1);
    int tmp = dispatch_command(GetVars, offset + 5, -1);
    acceleration_value [axis] = ((tmp >> 18) & 0x3f)*64;
    accelsteps [axis] =  (tmp >> 18) & 0xff;
    microsteps [axis] = (tmp >> 3) & 0x7f;
    direction_invert [axis] = (tmp >> 2) & 0x1;
    stepping [axis] = tmp & 0x03;
    features [axis] = dispatch_command(GetVars, offset + 6, -1);
    gt1feature[axis] = dispatch_command(GetVars, offset + 7, -1) & 0xff;
    type = (dispatch_command(GetVars, offset + 7, -1) >> 16) & 0xff;
    optimize_values(axis);
}

int ahp_gt_connect(const char* port)
{
    if(!RS232_OpenComport(port)) {
        if(!RS232_SetupPort(9600, "8N1", 0)) {
            version = dispatch_command(InquireMotorBoardVersion, 0, -1);
            fprintf(stderr, "MC Version: %02X\n", ahp_gt_get_mc_version());
            if(ahp_gt_get_mc_version()>0x24) {
                ahp_gt_read_values(0);
                ahp_gt_read_values(1);
                return 1;
            }
        }
        RS232_CloseComport();
    }
    return 0;
}

int ahp_gt_get_mc_version()
{
    return (version>>8)&0xff;
}

MountType ahp_gt_get_mount_type()
{
    return type;
}

GT1Feature ahp_gt_get_feature(int axis)
{
    return gt1feature[axis];
}

SkywatcherFeature ahp_gt_get_features(int axis)
{
    return features[axis];
}

double ahp_gt_get_motor_steps(int axis)
{
    return steps[axis];
}

double ahp_gt_get_motor_teeth(int axis)
{
    return motor[axis];
}

double ahp_gt_get_worm_teeth(int axis)
{
    return worm[axis];
}

double ahp_gt_get_crown_teeth(int axis)
{
    return crown[axis];
}

double ahp_gt_get_microsteps(int axis)
{
    return microsteps[axis];
}

double ahp_gt_get_guide_steps(int axis)
{
    return guide[axis];
}

double ahp_gt_get_acceleration_steps(int axis)
{
    return accelsteps[axis];
}

double ahp_gt_get_acceleration(int axis)
{
    return acceleration_value[axis];
}

int ahp_gt_get_direction_invert(int axis)
{
    return direction_invert[axis];
}

GT1Stepping ahp_gt_get_stepping_conf(int axis)
{
    return (GT1Stepping)stepping[axis];
}

double ahp_gt_get_max_speed(int axis)
{
    return maxspeed_value[axis];
}

double ahp_gt_get_speed_limit(int axis)
{
    return speed_limit[axis];
}

void ahp_gt_set_mount_type(MountType value)
{
    type = value;
}

void ahp_gt_set_features(int axis, SkywatcherFeature value)
{
    features[axis] &= ~value;
    features[axis] |= value;
}

void ahp_gt_set_motor_steps(int axis, double value)
{
    steps[axis] = value;
    optimize_values(axis);
}

void ahp_gt_set_motor_teeth(int axis, double value)
{
    motor[axis] = value;
    optimize_values(axis);
}

void ahp_gt_set_worm_teeth(int axis, double value)
{
    worm[axis] = value;
    optimize_values(axis);
}

void ahp_gt_set_crown_teeth(int axis, double value)
{
    crown[axis] = value;
    optimize_values(axis);
}

void ahp_gt_set_guide_steps(int axis, double value)
{
    guide[axis] = value;
}

void ahp_gt_set_acceleration_steps(int axis, double value)
{
    accelsteps[axis] = value;
    optimize_values(axis);
}

void ahp_gt_set_acceleration(int axis, double value)
{
    acceleration_value[axis] = value;
    optimize_values(axis);
}

void ahp_gt_set_direction_invert(int axis, int value)
{
    direction_invert[axis] = value&1;
}

void ahp_gt_set_stepping_conf(int axis, GT1Stepping value)
{
    stepping[axis] = (int)value;
}

void ahp_gt_set_max_speed(int axis, double value)
{
    maxspeed_value[axis] = value < 1 ? 1 : value;
    maxspeed_value[axis] = maxspeed_value[axis] < speed_limit[axis] ? maxspeed_value[axis] : speed_limit[axis];
    optimize_values(axis);
}
