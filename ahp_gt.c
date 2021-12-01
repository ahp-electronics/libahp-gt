#include "ahp_gt.h"
#include "rs232.h"
#include <pthread.h>

#define HEX(c) (int)(((c) < 'A') ? ((c) - '0') : ((c) - 'A') + 10)

typedef enum {
    Ra = 0,
    Dec = 1,
    num_axes = 2,
} SkywatcherAxis;

static int mutexes_initialized = 0;
static pthread_mutexattr_t mutex_attr;
static pthread_mutex_t mutex;
static char command[32];
static char response[32];
static int dispatch_command(SkywatcherCommand cmd, int axis, int command_arg);

static MountType type = isEQ8;
static int rs232_polarity = 0;
static const double SIDEREAL_DAY = 86164.0916000;
static int totalsteps[num_axes] = { 200 * 64 * 40 / 10 * 180, 200 * 64 * 40 / 10 * 180 };
static int wormsteps[num_axes] = { 200 * 64 * 40 / 10, 200 * 64 * 40 / 10 };
static double maxperiod[num_axes] = { 64, 64 };
static double minperiod[num_axes] = { 1, 1 };
static double speed_limit[num_axes] = { 800, 800 };
static double acceleration[num_axes] = { 20.0, 20.0 };
static double acceleration_value[num_axes] = { 20.0, 20.0 };
static double divider[num_axes] = { 1, 1 };
static int multiplier[num_axes] = { 0, 0 };
static int address_value = 0;
static int dividers = 0;
static int mount_flags = 0;
static double crown[num_axes] = { 180, 180 };
static double steps[num_axes] = { 200, 200 };
static double motor[num_axes] = { 10, 10 };
static double worm[num_axes] = { 40, 40 };
static double guide[num_axes] = { 1, 1 };
static int direction_invert[num_axes] = { 0, 0 };
static int stepping_conf[num_axes] = { 0, 0 };
static int stepping_mode[num_axes] = { 0, 0 };
static int version = 0;
static double maxspeed[num_axes] = { 1000, 1000 };
static double maxspeed_value[num_axes] = { 500, 500 };
static int features[num_axes] = { hasPPEC, hasPPEC };
static SkywatcherMotionMode motionmode[num_axes] = {0, 0};
static SkywatcherAxisStatus axisstatus[2] = {0, 0};
static GT1Feature gt1feature[num_axes] = { GpioUnused, GpioUnused };
static double accelsteps[num_axes]  = { 1, 1 };
static unsigned char pwmfreq = 0;
static unsigned int ahp_gt_connected = 0;

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

    // Clear string
    response[0] = '\0';
    char c = 0;
    while(c != '\r' && err_code < 10) {
        if(1 == RS232_RecvBuf(&c, 1))
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
    int ret = -1;
    if(!mutexes_initialized) {
        pthread_mutexattr_init(&mutex_attr);
        pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_ERRORCHECK);
        pthread_mutex_init(&mutex, &mutex_attr);
        mutexes_initialized = 1;
    }
    while(pthread_mutex_trylock(&mutex))
        usleep(10000);
    for (unsigned char i = 0; i < 10; i++)
    {
        // Clear string
        command[0] = '\0';
        char command_arg[32];
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

        RS232_flushRXTX();
        if ((RS232_SendBuf((unsigned char*)command, n)) < n)
        {
            if (i == 9)
            {
                ret = -1;
                break;
            }
            else
            {
                usleep(100000);
                continue;
            }
        }
        usleep(100000);

        command[n-1] = '\0';

        ret = read_eqmod();
        break;
    }
    pthread_mutex_unlock(&mutex);
    return ret;
}

static void optimize_values(int axis)
{
    double sidereal_period = SIDEREAL_DAY / crown[axis];
    double baseclock = 375000;
    int maxsteps = 0xffffff;
    if(stepping_mode[axis] != HalfStep)
        maxsteps >>= 8;
    wormsteps [axis] = (int)(steps [axis] * worm [axis] / motor [axis]);
    totalsteps [axis] = (int)(crown [axis] * wormsteps [axis]);
    double d = 1.0;
    if (ahp_gt_get_mc_version() > 0x30)
        d += fmin(14.0, (double)totalsteps [axis] / maxsteps);
    divider [axis] = floor(d);
    multiplier [axis] = 1;
    if(stepping_mode[axis] != HalfStep)
        multiplier [axis] += (int)(62-(d-divider [axis])*62);
    wormsteps [axis] = (int)((double)wormsteps [axis] * (double)multiplier [axis] / (double)divider [axis]);
    totalsteps [axis] = (int)((double)wormsteps [axis] * (double)crown [axis]);

    maxperiod [axis] = (int)sidereal_period;
    speed_limit [axis] = (int)(800);
    maxspeed [axis] = fmin(speed_limit [axis], maxspeed [axis]);
    maxspeed_value [axis] = (int)fmax(minperiod [axis], (maxperiod [axis] * multiplier [axis] / maxspeed [axis]));
    guide [axis] = (int)(SIDEREAL_DAY * baseclock / totalsteps [axis]);

    double accel = acceleration [axis] / (M_PI * 2.0);
    int degrees = (int)(accel * (double)totalsteps [axis] / (double)multiplier [axis]);
    for (acceleration_value [axis] = 0; acceleration_value [axis] < 63 && degrees > 0; acceleration_value [axis]++, degrees -= acceleration_value [axis])
        ;
    accelsteps [axis] = 0;
    if (acceleration_value [axis] > 0) {
        accelsteps [axis] = (int)fmax (1, fmin (0xff, guide [axis] / acceleration_value [axis]));
    }
    if (ahp_gt_get_mc_version() > 0x30) {
        dividers = rs232_polarity | ((unsigned char)divider [0] << 1) | ((unsigned char)divider [1] << 5) | (address_value << 9);
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
    *percent = axis * 50 + 6.25;
    if (!WriteAndCheck (axis, offset + 1, wormsteps [axis])) {
        *finished = -1;
        return;
    }
    *percent = axis * 50 + 12.5;
    if (!WriteAndCheck (axis, offset + 2, maxspeed_value [axis])) {
        *finished = -1;
        return;
    }
    *percent = axis * 50 + 18.75;
    if (!WriteAndCheck (axis, offset + 3, guide [axis])) {
        *finished = -1;
        return;
    }
    *percent = axis * 50 + 25;
    if (!WriteAndCheck (axis, offset + 4, wormsteps [axis])) {
        *finished = -1;
        return;
    }
    *percent = axis * 50 + 32.25;
    if (!WriteAndCheck (axis, offset + 5, ((int)acceleration_value [axis] << 18) | ((int)accelsteps [axis] << 10) | (((int)multiplier [axis] & 0x7f) << 3) | ((stepping_conf[axis] & 0x03) << 1) | (direction_invert[axis] & 1))) {
        *finished = -1;
        return;
    }
    *percent = axis * 50 + 38.5;
    if (!WriteAndCheck (axis, offset + 6, (int)features [axis])) {
        *finished = -1;
        return;
    }
    *percent = axis * 50 + 44.75;
    if (!WriteAndCheck (axis, offset + 7, ((((0xf-pwmfreq) << 4) >> (2 * axis)) & 0x30) | ((int)stepping_mode[axis] << 6) | (((mount_flags >> axis)&1) << 3) | ((int)gt1feature[axis] & 7) | (axis == 0?(((unsigned char)type)<<16):((mount_flags&0x3fc)<<14)) | (int)(((dividers>>(8*axis))&0xff)<<8))) {
        *finished = -1;
        return;
    }
    *percent = axis * 50 + 50;
    dispatch_command (ReloadVars, axis, -1);
    *finished = 1;
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
    multiplier [axis] = (tmp >> 3) & 0x7f;
    direction_invert [axis] = (tmp >> 2) & 0x1;
    stepping_conf [axis] = (tmp & 0x06)>>1;
    features [axis] = dispatch_command(GetVars, offset + 6, -1);
    gt1feature[axis] = dispatch_command(GetVars, offset + 7, -1) & 0x7;
    stepping_mode[axis] = (dispatch_command(GetVars, offset + 7, -1) >> 6) & 0x03;
    pwmfreq = (dispatch_command(GetVars, 7, -1) >> 4) & 0x3;
    pwmfreq |= (dispatch_command(GetVars, 15, -1) >> 2) & 0xc;
    pwmfreq = 15-pwmfreq;
    type = (dispatch_command(GetVars, offset + 7, -1) >> 16) & 0xff;
    mount_flags = (dispatch_command(GetVars, 7, -1) & 0x8) >> 3;
    mount_flags |= (dispatch_command(GetVars, 15, -1) & 0x8) >> 2;
    dividers = (dispatch_command(GetVars, 7, -1) >> 8) & 0xff;
    dividers |= dispatch_command(GetVars, 15, -1) & 0xff00;
    divider[axis] = (dividers >> (1+axis*4)) & 0xf;
    address_value = (dividers >> 9) & 0x7f;
    rs232_polarity = dividers & 1;
    optimize_values(axis);
}

int ahp_gt_connect_fd(int fd)
{
    if(fd != -1) {
        RS232_SetFD(fd, 9600);
        version = dispatch_command(InquireMotorBoardVersion, 0, -1);
        fprintf(stderr, "MC Version: %02X\n", ahp_gt_get_mc_version());
        if(ahp_gt_get_mc_version()>0x24) {
            ahp_gt_read_values(0);
            ahp_gt_read_values(1);
            ahp_gt_connected = 1;
            return 0;
        }
    }
    return 1;
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
                ahp_gt_connected = 1;
                return 0;
            }
        }
        RS232_CloseComport();
    }
    return 1;
}

void ahp_gt_disconnect()
{
    RS232_CloseComport();
    if(mutexes_initialized) {
        pthread_mutex_unlock(&mutex);
        pthread_mutex_destroy(&mutex);
        pthread_mutexattr_destroy(&mutex_attr);
        mutexes_initialized = 0;
    }
    ahp_gt_connected = 0;
}

unsigned int ahp_gt_is_connected()
{
    return ahp_gt_connected;
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
    return (SkywatcherFeature)features[axis];
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

double ahp_gt_get_multiplier(int axis)
{
    return multiplier[axis];
}

double ahp_gt_get_divider(int axis)
{
    return divider[axis];
}

int ahp_gt_get_totalsteps(int axis)
{
    return totalsteps[axis];
}

int ahp_gt_get_wormsteps(int axis)
{
    return wormsteps[axis];
}

double ahp_gt_get_guide_steps(int axis)
{
    return guide[axis];
}

double ahp_gt_get_acceleration_steps(int axis)
{
    return accelsteps[axis];
}

double ahp_gt_get_acceleration_angle(int axis)
{
    return acceleration[axis];
}

int ahp_gt_get_rs232_polarity()
{
    return rs232_polarity;
}

int ahp_gt_get_pwm_frequency()
{
    return pwmfreq;
}

int ahp_gt_get_direction_invert(int axis)
{
    return direction_invert[axis];
}

int ahp_gt_get_mount_flags()
{
    return mount_flags;
}

GT1SteppingConfiguration ahp_gt_get_stepping_conf(int axis)
{
    return (GT1SteppingConfiguration)stepping_conf[axis];
}

GT1SteppingMode ahp_gt_get_stepping_mode(int axis)
{
    return (GT1SteppingMode)stepping_mode[axis];
}

double ahp_gt_get_max_speed(int axis)
{
    return maxspeed[axis];
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

void ahp_gt_set_feature(int axis, GT1Feature value)
{
    gt1feature[axis] = value & 7;
    optimize_values(axis);
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
    optimize_values(axis);
}

void ahp_gt_set_pwm_frequency(int value)
{
    value = fmin(0xb, fmax(0, value));
    pwmfreq = value;
}

void ahp_gt_set_acceleration_angle(int axis, double value)
{
    acceleration [axis] = value;
    optimize_values(axis);
}

void ahp_gt_set_rs232_polarity(int value)
{
    rs232_polarity = value;
    dividers = rs232_polarity | ((unsigned char)divider [0] << 1) | ((unsigned char)divider [1] << 5) | (address_value << 9);
}

void ahp_gt_set_direction_invert(int axis, int value)
{
    direction_invert[axis] = value&1;
}

void ahp_gt_set_mount_flags(int value)
{
    mount_flags = value;
}

void ahp_gt_set_stepping_conf(int axis, GT1SteppingConfiguration value)
{
    stepping_conf[axis] = (int)value;
}

void ahp_gt_set_stepping_mode(int axis, GT1SteppingMode value)
{
    stepping_mode[axis] = (int)value;
    optimize_values(axis);
}

void ahp_gt_set_max_speed(int axis, double value)
{
    maxspeed[axis] = fabs(value);
    optimize_values(axis);
}

void ahp_gt_set_divider(int axis, int value)
{
    divider[axis] = abs(value);
    dividers = rs232_polarity | ((unsigned char)divider [0] << 1) | ((unsigned char)divider [1] << 5) | (address_value << 9);
}

void ahp_gt_set_multiplier(int axis, int value)
{
    multiplier[axis] = abs(value);
}

void ahp_gt_set_totalsteps(int axis, int value)
{
    totalsteps[axis] = abs(value);
}

void ahp_gt_set_wormsteps(int axis, int value)
{
    wormsteps[axis] = abs(value);
}

int ahp_gt_select_device(int address) {
    address &= 0x7f;
    int device_present = dispatch_command(SetAddress, 0, (address | 0x80));
    if(!device_present) {
        ahp_gt_read_values(Ra);
        ahp_gt_read_values(Dec);
    }
    return device_present;
}

void ahp_gt_set_address(int address)
{
    address_value = address;
    optimize_values(0);
}

int ahp_gt_get_address()
{
    return address_value;
}

SkywatcherAxisStatus ahp_gt_get_status(int axis)
{
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
    return status;
}

void ahp_gt_set_position(int axis, double value)
{
    dispatch_command(SetAxisPositionCmd, axis, (int)(value*totalsteps[axis]/M_PI/2.0)+0x800000);
}

double ahp_gt_get_position(int axis)
{
    int steps = dispatch_command(GetAxisPosition, axis, -1);
    steps -= 0x800000;
    return (double)steps*M_PI*2.0/(double)totalsteps[axis];
}

int ahp_gt_is_axis_moving(int axis)
{
    return axisstatus[axis].Running;
}

void ahp_gt_goto_absolute(int axis, double target, double speed) {
    double position = ahp_gt_get_position(axis);
    speed = fabs(speed);
    speed *= (target-position < 0 ? -1 : 1);
    double max = totalsteps[axis];
    target /= M_PI*2;
    target *= max;
    target += (double)0x800000;
    double maxperiod = SIDEREAL_DAY * wormsteps[axis] / totalsteps[axis];
    int period = maxperiod * multiplier[axis];
    SkywatcherMotionMode mode = MODE_SLEW_HISPEED;
    if(fabs(speed) < 128.0) {
        mode = MODE_SLEW_LOSPEED;
        period /= multiplier[axis];
    }
    mode |= (speed < 0 ? 1 : 0);
    period /= fabs(speed);
    ahp_gt_stop_motion(axis, 1);
    motionmode[axis] = mode;
    dispatch_command (Initialize, axis, -1);
    dispatch_command (ActivateMotor, axis, -1);
    dispatch_command(SetGotoTarget, axis, target);
    dispatch_command (SetStepPeriod, axis, period);
    dispatch_command (SetMotionMode, axis, mode);
    dispatch_command (StartMotion, axis, -1);
}

void ahp_gt_goto_relative(int axis, double increment, double speed) {
    speed = fabs(speed);
    speed *= (increment < 0 ? -1 : 1);
    double max = totalsteps[axis];
    increment /= M_PI*2;
    increment *= max;
    double maxperiod = SIDEREAL_DAY * wormsteps[axis] / totalsteps[axis];
    int period = maxperiod * multiplier[axis];
    SkywatcherMotionMode mode = MODE_SLEW_HISPEED;
    if(fabs(speed) < 128.0) {
        mode = MODE_SLEW_LOSPEED;
        period /= multiplier[axis];
    }
    mode |= (speed < 0 ? 1 : 0);
    period /= fabs(speed);
    ahp_gt_stop_motion(axis, 1);
    motionmode[axis] = mode;
    dispatch_command (Initialize, axis, -1);
    dispatch_command (ActivateMotor, axis, -1);
    dispatch_command(SetGotoTargetIncrement, axis, (int)fabs(increment));
    dispatch_command (SetStepPeriod, axis, period);
    dispatch_command (SetMotionMode, axis, mode);
    dispatch_command (StartMotion, axis, -1);
}

void ahp_gt_start_motion(int axis, double speed) {
    double period = SIDEREAL_DAY * multiplier[axis] * wormsteps[axis] / totalsteps[axis];
    SkywatcherMotionMode mode = MODE_SLEW_HISPEED;
    if(fabs(speed) < 128.0) {
        mode = MODE_SLEW_LOSPEED;
        period /= multiplier[axis];
    }
    mode |= (speed < 0 ? 1 : 0);
    period /= fabs(speed);
    ahp_gt_stop_motion(axis, 1);
    motionmode[axis] = mode;
    dispatch_command (Initialize, axis, -1);
    dispatch_command (ActivateMotor, axis, -1);
    dispatch_command (SetStepPeriod, axis, period);
    dispatch_command (SetMotionMode, axis, mode);
    dispatch_command (StartMotion, axis, -1);
}

void ahp_gt_stop_motion(int axis, int wait) {
    dispatch_command(InstantAxisStop, axis, -1);
    if(wait) {
        while (ahp_gt_is_axis_moving(axis))
            usleep(100000);
    }
}

void ahp_gt_start_tracking(int axis) {
    ahp_gt_stop_motion(axis, 1);
    double period = SIDEREAL_DAY * wormsteps[axis] / totalsteps[axis];
    dispatch_command (SetStepPeriod, axis, period);
    dispatch_command (Initialize, axis, -1);
    dispatch_command (ActivateMotor, axis, -1);
    dispatch_command (SetMotionMode, axis, 0x10);
    dispatch_command (StartMotion, axis, -1);
}
