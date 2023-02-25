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
#include <pthread.h>
#include <time.h>
#include <fcntl.h>
#include <sys/time.h>


#include "rs232.c"

#define HEX(c) (int)(((c) < 'A') ? ((c) - '0') : ((c) - 'A') + 10)
#define MAX_STEP_FREQ 1000

#ifndef GAMMAJ2000
///Right ascension of the meridian at J2000 zero at Greenwich
#define GAMMAJ2000 18.6971378528
#endif
#ifndef SIDEREAL_DAY
#define SIDEREAL_DAY 86164.098903691
#endif
#ifndef SOLAR_DAY
#define SOLAR_DAY 86400
#endif
#ifndef SIDEREAL_24
#define SIDEREAL_24 SIDEREAL_DAY * 24.0 / SOLAR_DAY
#endif

typedef enum {
    Ra = 0,
    Dec = 1,
    num_axes = 2,
} SkywatcherAxis;

typedef struct {
    int rs232_polarity;
    int totalsteps[num_axes];
    int wormsteps[num_axes];
    int accel_steps[num_axes];
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
    double max_step_frequency[num_axes];
    double acceleration[num_axes];
    double crown[num_axes];
    double steps[num_axes];
    double motor[num_axes];
    double worm[num_axes];
    double guide[num_axes];
    double one_second[num_axes];
    double maxspeed[num_axes];
    double maxspeed_value[num_axes];
    double accel_increment[num_axes];
    SkywatcherMotionMode motionmode[num_axes];
    SkywatcherAxisStatus axisstatus[num_axes];
    GT1Feature gt1feature[num_axes];
    MountType type;
    GT1Flags mount_flags;
    double lat;
    double lon;
    double el;
    time_t time_offset;
    int time_zone;
    int isdst;
    int is_aligned;
    int connfd;
    int flipped;
    int will_flip;
    int tracking_mode;
    int threads_running;
    int baud_rate;
    pthread_t tracking_thread;
} gt1_info;

const double rates[9] = { 1, 8, 16, 32, 64, 128, 400, 600, 800 };
static int mutexes_initialized = 0;
static pthread_mutexattr_t mutex_attr;
static pthread_mutex_t mutex;
static char command[32];
static char response[32];
static int dispatch_command(SkywatcherCommand cmd, int axis, int command_arg);
static unsigned int ahp_gt_current_device = 0;
static unsigned int ahp_gt_connected = 0;
static unsigned int ahp_gt_detected[128] = { 0 };
static gt1_info devices[128] = { 0 };
static int sockfd;

static double get_timestamp()
{
    struct timezone tz;
    tz.tz_minuteswest = 0;
    tz.tz_dsttime = 0;
    struct timeval now;
    gettimeofday(&now, &tz);
    return (double) now.tv_sec + now.tv_usec / 1000000.0;
}

static double time_to_J2000time(time_t tp)
{
    struct tm t_tm;
    time_t j2000;
    t_tm.tm_sec = 0;
    t_tm.tm_min = 0;
    t_tm.tm_hour = 12;
    t_tm.tm_mday = 1;
    t_tm.tm_mon = 0;
    t_tm.tm_year = 100;
    j2000 = mktime(&t_tm);
    return (double)(tp - j2000);
}

static double J2000time_to_lst(double secs_since_J2000, double Long)
{
    Long *= SIDEREAL_24 / 360.0;
    return fmod(SIDEREAL_24 * secs_since_J2000 / SIDEREAL_DAY + Long + GAMMAJ2000, SIDEREAL_24);
}

static double get_lst()
{
    double lat, lon, el;
    ahp_gt_get_location(&lat, &lon, &el);
    double lst = fmod(J2000time_to_lst(time_to_J2000time(ahp_gt_get_time()), lon), 24.0);
    return lst;
}

static double range_ha(double ha)
{
    if (ha < -12)
        ha += 24.0;
    if (ha >= 12.0)
        ha -= 24.0;
    return ha;
}

static double range_24(double ra)
{
    if (ra < 0)
        ra += 24.0;
    if (ra >= 24.0)
        ra -= 24.0;
    return ra;
}

static double range_360(double deg)
{
    if ((deg >= 360.0))
        return (deg - 360.0);
    if (deg < 0.0)
        return (deg + 360.0);
    return deg;
}

double range_dec(double dec)
{
    if ((dec >= 270.0) && (dec <= 360.0))
        return (dec - 360.0);
    if ((dec >= 180.0) && (dec < 270.0))
        return (180.0 - dec);
    if ((dec >= 90.0) && (dec < 180.0))
        return (180.0 - dec);
    return dec;
}

static double calc_flipped_ha(double *ha, double *dec)
{
    int flipped = 0;
    double lat, lon, el;
    ahp_gt_get_location(&lat, &lon, &el);
    *ha = range_ha(*ha);
    *dec -= 90.0;
    if (lat >= 0.0) {
        if(*ha > 6.0) {
            flipped = 1;
            *ha = *ha-12.0;
        } else {
            *dec = -*dec;
        }
    }
    if (lat < 0.0) {
        if(*ha < 6.0) {
            flipped = 1;
            *ha = *ha+12.0;
        } else {
            *dec = -*dec;
        }
    }
    *ha = range_ha(*ha);
    return flipped;
}

static double get_local_hour_angle(double Ra)
{
    double ha = (get_lst() - Ra);
    return range_ha(ha);
}

double ahp_gt_tracking_sine(double Alt, double Az, double Lat)
{
    Alt *= M_PI / 180.0;
    Az *= M_PI / 180.0;
    Lat *= M_PI / 180.0;
    return cos(Lat) * cos(Az) / cos(Alt);
}

double ahp_gt_tracking_cosine(double Alt, double Az, double Lat)
{
    Alt *= M_PI / 180.0;
    Az *= M_PI / 180.0;
    Lat *= M_PI / 180.0;
    return cos(Lat) * sin(Az) / cos(Alt);
}

void ahp_gt_get_alt_az_coordinates(double Ra, double Dec, double* Alt, double *Az)
{
    double alt, az;
    double lat, lon, el;
    ahp_gt_get_location(&lat, &lon, &el);
    double ha = get_local_hour_angle(Ra);
    ha *= M_PI / 12.0;
    Dec *= M_PI / 180.0;
    lat *= M_PI / 180.0;
    alt = asin(sin(Dec) * sin(lat) + cos(Dec) * cos(lat) * cos(ha));
    az = acos((sin(Dec) - sin(alt) * sin(lat)) / (cos(alt) * cos(lat)));
    alt *= 180.0 / M_PI;
    az *= 180.0 / M_PI;
    if (sin(ha) > 0.0)
        az = 360 - az;
    *Alt = alt;
    *Az = az;
}

void ahp_gt_get_ra_dec_coordinates(double Alt, double Az, double *Ra, double *Dec)
{
    double ra, dec, ha;
    double lat, lon, el;
    ahp_gt_get_location(&lat, &lon, &el);
    Alt *= M_PI / 180.0;
    Az *= M_PI / 180.0;
    lat *= M_PI / 180.0;
    dec = asin(cos(Az) * (cos(Alt) * cos(lat)) + sin(Alt) * sin(lat));
    ha = acos((sin(Alt) - sin(dec) * sin(lat)) / (cos(dec) * cos(lat)));
    dec *= 180.0 / M_PI;
    ha *= 24.0 / M_PI;
    if (sin(ha) > 0.0)
        ha = 24.0 - ha;
    ra = range_24(get_local_hour_angle(ha));
    *Ra = ra;
    *Dec = dec;
}

double ahp_gt_get_ha()
{
    double lat, lon, el;
    double ha = ahp_gt_get_position(0, NULL) * 12.0 / M_PI;
    ahp_gt_get_location(&lat, &lon, &el);
    if (lat >= 0.0)
        ha = range_ha(ha);
    else
        ha = range_ha(24.0 - ha);
    return ha;
}

double ahp_gt_get_ra()
{
    double lat, lon, el;
    ahp_gt_get_location(&lat, &lon, &el);
    double ra = get_lst() - range_24(ahp_gt_get_ha());
    if(devices[ahp_gt_get_current_device()].flipped) {
        if (lat >= 0.0) {
            ra = get_lst() - range_24(ahp_gt_get_ha()) - 12.0;
        } else {
            ra = get_lst() - range_24(ahp_gt_get_ha()) + 12.0;
        }
    }
    return range_24(ra);
}

double ahp_gt_get_dec()
{
    double dec = ahp_gt_get_position(1, NULL) * 180.0 / M_PI;
    double lat, lon, el;
    ahp_gt_get_location(&lat, &lon, &el);
    if(lat >= 0.0) {
        if(dec > 90.0 && dec < 270.0)
            devices[ahp_gt_get_current_device()].flipped = 1;
        else
            devices[ahp_gt_get_current_device()].flipped = 0;
    } else {
        if(dec < 90.0 || dec >= 270.0)
            devices[ahp_gt_get_current_device()].flipped = 1;
        else
            devices[ahp_gt_get_current_device()].flipped = 0;
    }
    return dec;
}

static int readcmd(char *cmd, int len)
{
    int n = read(devices[ahp_gt_get_current_device()].connfd, cmd, len);
    cmd[(n < 0 ? 0 : n)] = 0;
    return n;
}

static void *track(void* arg) {
    gt1_info *info = arg;
    while(info->threads_running) {
        double alt, az;
        switch(info->tracking_mode) {
        case 0:
            break;
        case 1:
            ahp_gt_start_motion(0, 1.0);
            break;
        case 2:
            ahp_gt_get_alt_az_coordinates(ahp_gt_get_ra(), ahp_gt_get_dec(), &alt, &az);
            ahp_gt_start_motion(0, ahp_gt_tracking_cosine(alt, az, info->lat));
            ahp_gt_start_motion(1, ahp_gt_tracking_sine(alt, az, info->lat));
            break;
        default:
            break;
        }
        usleep(500000);
    }
    return NULL;
}

static int synscan_poll()
{
    char msg[32];
    char cmd[32];
    time_t ts;
    struct tm *now;
    int in_goto;
    double ha = 0.0, alt = 0.0, az = 0.0, lat = 0.0, lon = 0.0, el = 0.0, ra = 0.0, dec = 0.0;
    unsigned int alpha, delta;
    int d, m, s, sign;
    if(ahp_gt_is_connected()) {
        if(ahp_gt_is_detected(ahp_gt_get_current_device())) {
            memset(cmd, 0, 32);
            int n = readcmd(cmd, 1);
            if(n <= 0)
                goto err_end;
            switch(cmd[0]) {
            case GetRaDec:
                delta = ahp_gt_get_dec() * 0x8000 / 180.0;
                alpha = ahp_gt_get_ra() * 0x8000 / 12.0;
                sprintf(msg, "%04X,%04X#", alpha, delta);
                write(devices[ahp_gt_get_current_device()].connfd, msg, 10);
                break;
            case GetPreciseRaDec:
                delta = ahp_gt_get_dec() * 0x80000000 / 180.0;
                alpha = ahp_gt_get_ra() * 0x80000000 / 12.0;
                sprintf(msg, "%08X,%08X#", alpha, delta);
                write(devices[ahp_gt_get_current_device()].connfd, msg, 18);
                break;
            case GetAzAlt:
                ahp_gt_get_alt_az_coordinates(ahp_gt_get_ra(), ahp_gt_get_dec(), &alt, &az);
                alpha = az * 0x8000 / 12.0;
                delta = alt * 0x8000 / 180.0;
                sprintf(msg, "%04X,%04X#", alpha, delta);
                write(devices[ahp_gt_get_current_device()].connfd, msg, 10);
                break;
            case GetPreciseAzAlt:
                ahp_gt_get_alt_az_coordinates(ahp_gt_get_ra(), ahp_gt_get_dec(), &alt, &az);
                alpha = az * 0x80000000 / 12.0;
                delta = alt * 0x80000000 / 180.0;
                sprintf(msg, "%08X,%08X#", alpha, delta);
                write(devices[ahp_gt_get_current_device()].connfd, msg, 18);
                break;
            case SyncRaDec:
                if(readcmd(cmd, 4) < 0)
                    goto err_end;
                ra = range_24((double)strtol(cmd, NULL, 16) * 12.0 / 0x8000);
                ha = get_local_hour_angle(ra);
                ahp_gt_set_position(0, ha * M_PI / 12.0);
                if(readcmd(cmd, 1) < 0)
                    goto err_end;
                if(readcmd(cmd, 4) < 0)
                    goto err_end;
                dec = range_dec((double)strtol(cmd, NULL, 16) * 180.0 / 0x8000);
                ahp_gt_set_position(1, dec * M_PI / 180.0);
                sprintf(msg, "#");
                write(devices[ahp_gt_get_current_device()].connfd, msg, 1);
                break;
            case SyncPreciseRaDec:
                if(readcmd(cmd, 8) < 0)
                    goto err_end;
                ra = range_24((double)strtol(cmd, NULL, 16) * 12.0 / 0x80000000);
                ha = get_local_hour_angle(ra);
                ahp_gt_set_position(0, ha * M_PI / 12.0);
                if(readcmd(cmd, 1) < 0)
                    goto err_end;
                if(readcmd(cmd, 8) < 0)
                    goto err_end;
                dec = range_dec((double)strtol(cmd, NULL, 16) * 180.0 / 0x80000000);
                ahp_gt_set_position(1, dec * M_PI / 180.0);
                sprintf(msg, "#");
                write(devices[ahp_gt_get_current_device()].connfd, msg, 1);
                break;
            case GotoRaDec:
                if(readcmd(cmd, 4) < 0)
                    goto err_end;
                ra = range_24((double)strtol(cmd, NULL, 16) * 12.0 / 0x8000);
                if(readcmd(cmd, 1) < 0)
                    goto err_end;
                if(readcmd(cmd, 4) < 0)
                    goto err_end;
                dec = range_dec((double)strtol(cmd, NULL, 16) * 180.0 / 0x8000);
                ahp_gt_goto_radec(ra, dec);
                sprintf(msg, "#");
                write(devices[ahp_gt_get_current_device()].connfd, msg, 1);
                break;
            case GotoPreciseRaDec:
                if(readcmd(cmd, 8) < 0)
                    goto err_end;
                ra = range_24((double)strtol(cmd, NULL, 16) * 12.0 / 0x80000000);
                if(readcmd(cmd, 1) < 0)
                    goto err_end;
                if(readcmd(cmd, 8) < 0)
                    goto err_end;
                dec = range_dec((double)strtol(cmd, NULL, 16) * 180.0 / 0x80000000);
                ahp_gt_goto_radec(ra, dec);
                sprintf(msg, "#");
                write(devices[ahp_gt_get_current_device()].connfd, msg, 1);
                break;
            case GotoAzAlt:
                if(readcmd(cmd, 4) < 0)
                    goto err_end;
                az = range_360((double)strtol(cmd, NULL, 16) * 180.0 / 0x8000);
                if(readcmd(cmd, 1) < 0)
                    goto err_end;
                if(readcmd(cmd, 4) < 0)
                    goto err_end;
                alt = range_dec((double)strtol(cmd, NULL, 16) * 180.0 / 0x8000);
                ahp_gt_goto_altaz(az, alt);
                sprintf(msg, "#");
                write(devices[ahp_gt_get_current_device()].connfd, msg, 1);
                break;
            case GotoPreciseAzAlt:
                if(readcmd(cmd, 8) < 0)
                    goto err_end;
                az = range_360((double)strtol(cmd, NULL, 16) * 180.0 / 0x80000000);
                if(readcmd(cmd, 1) < 0)
                    goto err_end;
                if(readcmd(cmd, 8) < 0)
                    goto err_end;
                alt = range_dec((double)strtol(cmd, NULL, 16) * 180.0 / 0x80000000);
                ahp_gt_goto_altaz(az, alt);
                sprintf(msg, "#");
                write(devices[ahp_gt_get_current_device()].connfd, msg, 1);
                break;
            case CancelGOTO:
                ahp_gt_stop_motion(0, 0);
                ahp_gt_stop_motion(1, 0);
                break;
            case GetTrackingMode:
                sprintf(msg, "%c#", ahp_gt_is_axis_moving(0) * 2);
                write(devices[ahp_gt_get_current_device()].connfd, msg, 2);
                break;
            case SetTrackingMode:
                if(readcmd(cmd, 1) < 0)
                    goto err_end;
                devices[ahp_gt_get_current_device()].tracking_mode = cmd[0];
                sprintf(msg, "#");
                write(devices[ahp_gt_get_current_device()].connfd, msg, 1);
                break;
            case Slew:
                if(readcmd(cmd, 7) < 0)
                    goto err_end;
                switch(cmd[0]) {
                case 1:
                    sprintf(msg, "%c%c#", 0, ahp_gt_get_mc_version());
                    write(devices[ahp_gt_get_current_device()].connfd, msg, 3);
                case 2:
                    if(cmd[3] == 0)
                        ahp_gt_stop_motion((cmd[1] == 16 ? 0 : 1), 0);
                    else
                        ahp_gt_start_motion((cmd[1] == 16 ? 0 : 1), (cmd[2] == 36 ? 1 : -1)*rates[cmd[3]-1]);
                    sprintf(msg, "#");
                    write(devices[ahp_gt_get_current_device()].connfd, msg, 1);
                    break;
                case 3:
                    if(strtol(&cmd[3], NULL, 16) == 0)
                        ahp_gt_stop_motion((cmd[1] == 16 ? 0 : 1), 0);
                    else
                        ahp_gt_start_motion((cmd[1] == 16 ? 0 : 1), (cmd[2] == 6 ? 1 : -1)*(double)strtol(&cmd[3], NULL, 16)/15.0);
                    sprintf(msg, "#");
                    write(devices[ahp_gt_get_current_device()].connfd, msg, 1);
                    break;
                }
                break;
            case SetLocation:
                if(readcmd(cmd, 8) < 0)
                    goto err_end;
                lat = 0.0;
                lat += (double)cmd[0];
                lat += (double)cmd[1] / 60.0;
                lat += (double)cmd[2] / 3600.0;
                lat *= cmd[3] ? -1 : 1;
                lon = 0.0;
                lon += (double)cmd[4];
                lon += (double)cmd[5] / 60.0;
                lon += (double)cmd[6] / 3600.0;
                lon *= cmd[7] ? -1 : 1;
                ahp_gt_set_location(lat, lon, 0);
                sprintf(msg, "#");
                write(devices[ahp_gt_get_current_device()].connfd, msg, 1);
                break;
            case GetLocation:
                ahp_gt_get_location(&lat, &lon, &el);
                sign = lat > 0 ? 0 : 1;
                lat = fabs(lat);
                d = floor(lat);
                lat -= d;
                m = lat * 60;
                lat -= m;
                s = lat * 60;
                sprintf(msg, "%c%c%c%c", d, m, s, sign);
                sign = lon > 0 ? 0 : 1;
                lon = fabs(lon);
                d = floor(lon);
                lon -= d;
                m = lon * 60;
                lon -= m;
                s = lon * 60;
                sprintf(msg, "%c%c%c%c", d, m, s, sign);
                sprintf(msg, "#");
                write(devices[ahp_gt_get_current_device()].connfd, msg, 1);
                break;
            case SetTime:
                if(readcmd(cmd, 8) < 0)
                    goto err_end;
                now = malloc(sizeof(struct tm));
                memset(now, 0, sizeof(struct tm));
                now->tm_hour = cmd[0];
                now->tm_min = cmd[1];
                now->tm_sec = cmd[2];
                now->tm_mon = cmd[3]-1;
                now->tm_mday = cmd[4];
                now->tm_year = cmd[5]+100;
                devices[ahp_gt_get_current_device()].isdst = cmd[7];
                devices[ahp_gt_get_current_device()].time_zone = cmd[6];
                ts = mktime(now);
                ts -= devices[ahp_gt_get_current_device()].time_zone * 3600;
                if(devices[ahp_gt_get_current_device()].isdst) ts -= 3600;
                ahp_gt_set_time(ts);
                free(now);
                sprintf(msg, "#");
                write(devices[ahp_gt_get_current_device()].connfd, msg, 1);
                break;
            case GetTime:
                ts = ahp_gt_get_time();
                now = gmtime(&ts);
                now += devices[ahp_gt_get_current_device()].time_zone * 3600;
                if(now->tm_isdst) now += 3600;
                d = devices[ahp_gt_get_current_device()].time_zone;
                d += (d < 0 ? 256 : 0);
                sprintf(msg, "%c%c%c%c%c%c%c%c#", now->tm_hour, now->tm_min, now->tm_sec, now->tm_mon, now->tm_mday, now->tm_year, d, now->tm_isdst);
            case GetSynScanVersion:
                sprintf(msg, "042507#");
                write(devices[ahp_gt_get_current_device()].connfd, msg, 7);
                break;
            case GetModel:
                sprintf(msg, "%c#", ahp_gt_get_mount_type());
                write(devices[ahp_gt_get_current_device()].connfd, msg, 2);
                break;
            case Echo:
                if(readcmd(cmd, 1) < 0)
                    goto err_end;
                sprintf(msg, "%c#", cmd[0]);
                write(devices[ahp_gt_get_current_device()].connfd, msg, 2);
                break;
            case AlignmentComplete:
                sprintf(msg, "%c#", devices[ahp_gt_get_current_device()].is_aligned);
                write(devices[ahp_gt_get_current_device()].connfd, msg, 2);
                break;
            case GOTOinProgress:
                in_goto = 0;
                in_goto |= ahp_gt_is_axis_moving(0);
                in_goto |= ahp_gt_is_axis_moving(1);
                sprintf(msg, "%c#", in_goto + '0');
                write(devices[ahp_gt_get_current_device()].connfd, msg, 2);
                break;
            case GetMountPointingState:
                sprintf(msg, "%c#", devices[ahp_gt_get_current_device()].flipped ? 'W' : 'E');
                write(devices[ahp_gt_get_current_device()].connfd, msg, 2);
                break;
            default:
                sprintf(msg, "#");
                write(devices[ahp_gt_get_current_device()].connfd, msg, 1);
                break;
            }
        }
    }
    return 0;
    err_end:
    return -1;
}

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
    char * reply;
    int err_code = 0, nbytes_read = 0;
    int max_err = 30;
    // Clear string
    memset(response, '\0', 32);
    unsigned char c = 0;
    while(c != '\r' && err_code < max_err) {
        if(1 == ahp_serial_RecvBuf(&c, 1) && c != 0) {
            response[nbytes_read++] = c;
        } else {
            err_code++;
            usleep(1000);
        }
    }
    if (err_code == max_err)
    {
        return 0;
    }
    if(!strncmp(command, response, strlen(command))) {
        reply = &response[strlen(command)];
        nbytes_read -= strlen(command);
    } else reply = response;

    // Remove CR
    reply[nbytes_read - 1] = '\0';

    pwarn("%s\n", reply);

    switch (reply[0])
    {
        case '=':
            if(nbytes_read > 2) {
                if(nbytes_read > 5)
                    return Revu24str2long(reply+1);
                else
                    return Highstr2long(reply+1);
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
    int maxtries = 15;
    int i;
    int c;
    memset(response, '0', 32);
    if(!mutexes_initialized) {
        pthread_mutexattr_init(&mutex_attr);
        pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_ERRORCHECK);
        pthread_mutex_init(&mutex, &mutex_attr);
        mutexes_initialized = 1;
    }
    while(pthread_mutex_trylock(&mutex))
        usleep(100);
    i = maxtries;
retry:
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
    pgarb("%s\n", command);

    ahp_serial_flushRXTX();
    for(c = 0; c < n; c++) {
        if ((ahp_serial_SendByte((unsigned char)command[c])) < 0)
            goto retry;
    }
    if (--i == 0)
        goto ret_err;

    ret = read_eqmod();
    pthread_mutex_unlock(&mutex);
    return ret;
ret_err:
    pthread_mutex_unlock(&mutex);
    return -1;
}

static void optimize_values(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    double baseclock = 375000;
    double usteps = 62.0;
    double maxdiv = 14.0;
    devices[ahp_gt_get_current_device()].wormsteps [axis] = (int)(devices[ahp_gt_get_current_device()].steps [axis] * devices[ahp_gt_get_current_device()].worm [axis] / devices[ahp_gt_get_current_device()].motor [axis]);
    devices[ahp_gt_get_current_device()].totalsteps [axis] = (int)(devices[ahp_gt_get_current_device()].crown [axis] * devices[ahp_gt_get_current_device()].wormsteps [axis]);
    int maxsteps = 0xffffff;
    if(devices[ahp_gt_get_current_device()].stepping_mode[axis] != HalfStep) {
        maxsteps >>= 6;
    }
    double d = 1.0;
    d += fmin(maxdiv, (double)devices[ahp_gt_get_current_device()].totalsteps [axis] / maxsteps);
    devices[ahp_gt_get_current_device()].divider [axis] = floor(d);
    while(devices[ahp_gt_get_current_device()].divider [axis] > 1 && usteps > 14) {
        devices[ahp_gt_get_current_device()].divider [axis] --;
        usteps /= 2;
        usteps --;
    }
    devices[ahp_gt_get_current_device()].multiplier [axis] = 1;
    switch(devices[ahp_gt_get_current_device()].stepping_mode[axis]) {
    case HalfStep:
        devices[ahp_gt_get_current_device()].speed_limit [axis] = MAX_STEP_FREQ*256;
        break;
    case Microstep:
        devices[ahp_gt_get_current_device()].multiplier [axis] += (int)usteps;
        devices[ahp_gt_get_current_device()].speed_limit [axis] = MAX_STEP_FREQ/4;
        break;
    case Mixed:
        devices[ahp_gt_get_current_device()].multiplier [axis] += (int)usteps;
        devices[ahp_gt_get_current_device()].speed_limit [axis] = MAX_STEP_FREQ;
        break;
    default:
        break;
    }
    devices[ahp_gt_get_current_device()].one_second[axis] = (1500000.0);
    devices[ahp_gt_get_current_device()].wormsteps [axis] *= (double)devices[ahp_gt_get_current_device()].multiplier [axis] / (double)devices[ahp_gt_get_current_device()].divider [axis];
    devices[ahp_gt_get_current_device()].totalsteps [axis] = (int)(devices[ahp_gt_get_current_device()].crown [axis] * devices[ahp_gt_get_current_device()].wormsteps [axis]);

    double sidereal_period = SIDEREAL_DAY * devices[ahp_gt_get_current_device()].multiplier[axis] * devices[ahp_gt_get_current_device()].wormsteps[axis] / devices[ahp_gt_get_current_device()].totalsteps[axis];
    devices[ahp_gt_get_current_device()].maxperiod [axis] = (int)sidereal_period;
    devices[ahp_gt_get_current_device()].speed_limit [axis] *= (SIDEREAL_DAY * (double)devices[ahp_gt_get_current_device()].multiplier[axis] / devices[ahp_gt_get_current_device()].totalsteps [axis]);
    devices[ahp_gt_get_current_device()].minperiod [axis] = 1;
    devices[ahp_gt_get_current_device()].maxspeed [axis] = fmin(devices[ahp_gt_get_current_device()].speed_limit [axis], devices[ahp_gt_get_current_device()].maxspeed [axis]);
    devices[ahp_gt_get_current_device()].maxspeed_value [axis] = (int)fmax(devices[ahp_gt_get_current_device()].minperiod [axis], (devices[ahp_gt_get_current_device()].maxperiod [axis] / devices[ahp_gt_get_current_device()].maxspeed [axis]));
    devices[ahp_gt_get_current_device()].guide [axis] = (int)(SIDEREAL_DAY * baseclock / devices[ahp_gt_get_current_device()].totalsteps [axis]);

    double degrees = devices[ahp_gt_get_current_device()].acceleration [axis] * (double)devices[ahp_gt_get_current_device()].totalsteps [axis] / devices[ahp_gt_get_current_device()].multiplier [axis] / (M_PI * 2.0);
    devices[ahp_gt_get_current_device()].accel_steps [axis] = floor(fmin(63, pow(degrees * 2, 0.4) + 1));
    devices[ahp_gt_get_current_device()].accel_increment [axis] = (int)fmin (0xff, devices[ahp_gt_get_current_device()].guide [axis] / devices[ahp_gt_get_current_device()].accel_steps [axis]);
    devices[ahp_gt_get_current_device()].address_value &= 0x7f;
    devices[ahp_gt_get_current_device()].rs232_polarity &= 0x1;
    devices[ahp_gt_get_current_device()].dividers = devices[ahp_gt_get_current_device()].rs232_polarity | ((unsigned char)devices[ahp_gt_get_current_device()].divider [0] << 1) | (((unsigned char)devices[ahp_gt_get_current_device()].divider [1]) << 5) | (devices[ahp_gt_get_current_device()].address_value << 9);
}

static int Check(int pos, int val)
{
    int ret = -1;
    int ntries = 10;
    while (ntries-- > 0)
    {
        ret = dispatch_command(GetVars, pos, -1);
        if(ret == val) {
            return 1;
        }
    }
    return 0;
}

static int WriteAndCheck(int axis, int pos, int val)
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
                     if (Check(pos, val)) {
                         nchecks = 0;
                         return 1;
                     }
                 }
             }
         }
     }
     return 0;
}

int ahp_gt_start_synscan_server(int port, int *interrupt)
{
    struct sockaddr_in servaddr;

    if(*interrupt) {
        perr("interrupted call");
        return -1;
    }
    if ( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
        perr("socket creation failed");
        return -1;
    }

    memset(&servaddr, 0, sizeof(servaddr));

    servaddr.sin_family    = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(port);

    if ( bind(sockfd, (const struct sockaddr *)&servaddr,
            sizeof(servaddr)) < 0 )
    {
        perr("bind failed");
        close(sockfd);
        return -1;
    }

    if ((listen(sockfd,  SOMAXCONN)) != 0) {
        perr("Listen failed...\n");
        close(sockfd);
        return -1;
    }

    ahp_gt_set_position(0, M_PI / 2.0);
    ahp_gt_set_position(1, M_PI / 2.0);

    struct timeval tv;
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(sockfd, &rfds);

    while(!(*interrupt)) {
        devices[ahp_gt_get_current_device()].connfd = -1;
        struct sockaddr client;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        socklen_t len = sizeof(client);
        devices[ahp_gt_get_current_device()].connfd = accept(sockfd, &client, &len);
        if(devices[ahp_gt_get_current_device()].connfd > -1) {
            while(!(*interrupt) && !synscan_poll())
                usleep(100000);
            close(devices[ahp_gt_get_current_device()].connfd);
        }
    }
    close(sockfd);
    return 0;
}

void ahp_gt_write_values(int axis, int *percent, int *finished)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    int offset = axis * 8;
    *finished = 0;
    *percent = axis * 50;
    if (!WriteAndCheck (axis, offset + 0, devices[ahp_gt_get_current_device()].totalsteps [axis])) {
        *finished = -1;
        return;
    }
    *percent = *percent + 6.25;
    if (!WriteAndCheck (axis, offset + 1, devices[ahp_gt_get_current_device()].wormsteps [axis])) {
        *finished = -1;
        return;
    }
    *percent = *percent + 6.25;
    if (!WriteAndCheck (axis, offset + 2, devices[ahp_gt_get_current_device()].maxspeed_value [axis])) {
        *finished = -1;
        return;
    }
    *percent = *percent + 6.25;
    if (!WriteAndCheck (axis, offset + 3, devices[ahp_gt_get_current_device()].guide [axis])) {
        *finished = -1;
        return;
    }
    *percent = *percent + 6.25;
    if (!WriteAndCheck (axis, offset + 4, devices[ahp_gt_get_current_device()].one_second [axis])) {
        *finished = -1;
        return;
    }
    *percent = *percent + 6.25;
    if (!WriteAndCheck (axis, offset + 5, ((int)devices[ahp_gt_get_current_device()].accel_steps [axis] << 18) | ((int)devices[ahp_gt_get_current_device()].accel_increment [axis] << 10) | (((int)devices[ahp_gt_get_current_device()].multiplier [axis] & 0x7f) << 3) | ((devices[ahp_gt_get_current_device()].stepping_conf[axis] & 0x03) << 1) | (devices[ahp_gt_get_current_device()].direction_invert[axis] & 1))) {
        *finished = -1;
        return;
    }
    *percent = *percent + 6.25;
    if (!WriteAndCheck (axis, offset + 6, (int)devices[ahp_gt_get_current_device()].features [axis])) {
        *finished = -1;
        return;
    }
    *percent = *percent + 6.25;
    if (!WriteAndCheck (axis, offset + 7, ((((0xf-devices[ahp_gt_get_current_device()].pwmfreq) << 4) >> (2 * axis)) & 0x30) | ((int)devices[ahp_gt_get_current_device()].stepping_mode[axis] << 6) | (((devices[ahp_gt_get_current_device()].mount_flags >> axis)&1) << 3) | ((int)devices[ahp_gt_get_current_device()].gt1feature[axis] & 7) | (axis == 0?(((unsigned char)devices[ahp_gt_get_current_device()].type)<<16):((devices[ahp_gt_get_current_device()].mount_flags&0x3fc)<<14)) | (int)(((devices[ahp_gt_get_current_device()].dividers>>(8*axis))&0xff)<<8))) {
        *finished = -1;
        return;
    }
    *percent = *percent + 6.25;
    dispatch_command (ReloadVars, axis, -1);
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
    devices[ahp_gt_get_current_device()].maxspeed_value [axis] = dispatch_command(GetVars, offset + 2, -1);
    devices[ahp_gt_get_current_device()].guide [axis] = dispatch_command(GetVars, offset + 3, -1);
    devices[ahp_gt_get_current_device()].one_second [axis] = dispatch_command(GetVars, offset + 4, -1);
    int tmp = dispatch_command(GetVars, offset + 5, -1);
    devices[ahp_gt_get_current_device()].accel_steps [axis] = ((tmp >> 18) & 0x3f);
    devices[ahp_gt_get_current_device()].accel_increment [axis] =  (tmp >> 10) & 0xff;
    devices[ahp_gt_get_current_device()].multiplier [axis] = (tmp >> 3) & 0x7f;
    devices[ahp_gt_get_current_device()].direction_invert [axis] = tmp & 0x1;
    devices[ahp_gt_get_current_device()].stepping_conf [axis] = (tmp & 0x06)>>1;
    devices[ahp_gt_get_current_device()].features [axis] = dispatch_command(GetVars, offset + 6, -1);
    devices[ahp_gt_get_current_device()].gt1feature[axis] = dispatch_command(GetVars, offset + 7, -1) & 0x7;
    devices[ahp_gt_get_current_device()].stepping_mode[axis] = (dispatch_command(GetVars, offset + 7, -1) >> 6) & 0x03;
    int pwmfreq = (dispatch_command(GetVars, 7, -1) >> 4) & 0x3;
    pwmfreq |= (dispatch_command(GetVars, 15, -1) >> 2) & 0xc;
    devices[ahp_gt_get_current_device()].pwmfreq = 0xf - pwmfreq;
    devices[ahp_gt_get_current_device()].type = (dispatch_command(GetVars, offset + 7, -1) >> 16) & 0xff;
    devices[ahp_gt_get_current_device()].mount_flags = (dispatch_command(GetVars, 7, -1) & 0x8) >> 3;
    devices[ahp_gt_get_current_device()].mount_flags |= (dispatch_command(GetVars, 15, -1) & 0x8) >> 2;
    devices[ahp_gt_get_current_device()].mount_flags |= (dispatch_command(GetVars, 15, -1) & 0xff0000) >> 14;
    devices[ahp_gt_get_current_device()].mount_flags &= 0x3ff;
    devices[ahp_gt_get_current_device()].dividers = (dispatch_command(GetVars, 7, -1) >> 8) & 0xff;
    devices[ahp_gt_get_current_device()].dividers |= dispatch_command(GetVars, 15, -1) & 0xff00;
    devices[ahp_gt_get_current_device()].divider[axis] = (devices[ahp_gt_get_current_device()].dividers >> (1+axis*4)) & 0xf;
    devices[ahp_gt_get_current_device()].address_value = (devices[ahp_gt_get_current_device()].dividers >> 9) & 0x7f;
    devices[ahp_gt_get_current_device()].rs232_polarity = devices[ahp_gt_get_current_device()].dividers & 1;

    devices[ahp_gt_get_current_device()].motor [axis] = 1;
    if (devices[ahp_gt_get_current_device()].steps [axis] == 0)
        devices[ahp_gt_get_current_device()].steps [axis] = 1;
    if (devices[ahp_gt_get_current_device()].wormsteps [axis] == 0)
        devices[ahp_gt_get_current_device()].wormsteps [axis] = 1;
    if (devices[ahp_gt_get_current_device()].totalsteps [axis] == 0)
        devices[ahp_gt_get_current_device()].totalsteps [axis] = 1;
    if (devices[ahp_gt_get_current_device()].multiplier [axis] == 0)
        devices[ahp_gt_get_current_device()].multiplier [axis] = 1;
    if (devices[ahp_gt_get_current_device()].maxspeed_value [axis] == 0)
        devices[ahp_gt_get_current_device()].maxspeed_value [axis] = 1;
    double degrees = pow(devices[ahp_gt_get_current_device()].accel_steps [axis] - 1, 2.5) / 2.0;
    devices[ahp_gt_get_current_device()].acceleration [axis] = degrees / ((double)devices[ahp_gt_get_current_device()].totalsteps [axis] / devices[ahp_gt_get_current_device()].multiplier [axis] / (M_PI * 2.0));
    devices[ahp_gt_get_current_device()].crown [axis] = devices[ahp_gt_get_current_device()].totalsteps [axis] / devices[ahp_gt_get_current_device()].wormsteps [axis];
    devices[ahp_gt_get_current_device()].worm [axis] = devices[ahp_gt_get_current_device()].wormsteps [axis] * devices[ahp_gt_get_current_device()].divider [axis] / devices[ahp_gt_get_current_device()].steps [axis] / devices[ahp_gt_get_current_device()].multiplier [axis];
    double decimals = devices[ahp_gt_get_current_device()].worm [axis] - floor(devices[ahp_gt_get_current_device()].worm [axis]);
    decimals /= floor(devices[ahp_gt_get_current_device()].worm [axis]);
    if(decimals != 0.0) {
        devices[ahp_gt_get_current_device()].motor [axis] /= decimals;
        devices[ahp_gt_get_current_device()].worm [axis] /= decimals;
    }
    devices[ahp_gt_get_current_device()].motor [axis] = floor(devices[ahp_gt_get_current_device()].motor [axis]);
    devices[ahp_gt_get_current_device()].worm [axis] = floor(devices[ahp_gt_get_current_device()].worm [axis]);
    double sidereal_period = SIDEREAL_DAY * devices[ahp_gt_get_current_device()].multiplier[axis] * devices[ahp_gt_get_current_device()].wormsteps[axis] / devices[ahp_gt_get_current_device()].totalsteps[axis];
    devices[ahp_gt_get_current_device()].maxspeed [axis] = sidereal_period / devices[ahp_gt_get_current_device()].maxspeed_value [axis];
}

int ahp_gt_connect_fd(int fd)
{
    if(ahp_gt_is_connected())
        return 0;
    if(fd != -1) {
        ahp_serial_SetFD(fd, 9600);
        ahp_gt_connected = 1;
        memset(devices, 0, sizeof(gt1_info)*128);
        memset(ahp_gt_detected, 0, sizeof(unsigned int)*128);
        if(!ahp_gt_detect_device(ahp_gt_get_current_device())) {
            ahp_gt_get_mc_version();
            if(devices[ahp_gt_get_current_device()].version > 0) {
                pgarb("MC Version: %02X\n", devices[ahp_gt_get_current_device()].version);
                return 0;
            }
        }
    }
    return 1;
}

void ahp_gt_set_fd(int fd)
{
    if(!ahp_gt_is_connected())
        return;
    return ahp_serial_SetFD(fd, 9600);
}

int ahp_gt_get_fd()
{
    if(!ahp_gt_is_connected())
        return -1;
    return ahp_serial_GetFD();
}

int ahp_gt_connect_udp(const char *address, int port)
{
    int fd = -1;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd >= 0 ) {
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = inet_addr(address);
        if(!connect(fd, (const struct sockaddr *)&addr, sizeof(addr)))
            return ahp_gt_connect_fd(fd);
    }

    return 1;
}
int ahp_gt_connect(const char* port)
{
    if(ahp_gt_is_connected())
        return 0;
    if(!ahp_serial_OpenComport(port)) {
        devices[ahp_gt_get_current_device()].baud_rate = 9600;
        int highspeed = 0;
retry:
        if(!ahp_serial_SetupPort(devices[ahp_gt_get_current_device()].baud_rate, "8N1", 0)) {
            ahp_gt_connected = 1;
            memset(devices, 0, sizeof(gt1_info)*128);
            memset(ahp_gt_detected, 0, sizeof(unsigned int)*128);
            if(!ahp_gt_detect_device()) {
                ahp_gt_get_mc_version();
                if(devices[ahp_gt_get_current_device()].version > 0) {
                    pgarb("MC Version: %02X\n", devices[ahp_gt_get_current_device()].version);
                    return 0;
                }
            } else if(!highspeed) {
                devices[ahp_gt_get_current_device()].baud_rate = 115200;
                highspeed = 1;
                goto retry;
            }
        }
        ahp_serial_CloseComport();
    }
    return 1;
}

void ahp_gt_disconnect()
{
    int addr = 0;
    if(ahp_gt_is_connected()) {
        for(addr = 0; addr < 128; addr++) {
            if(ahp_gt_detected[addr]) {
                ahp_gt_select_device(addr);
                devices[ahp_gt_get_current_device()].threads_running = 0;
                pthread_join(devices[ahp_gt_get_current_device()].tracking_thread, NULL);
            }
        }
        ahp_serial_CloseComport();
        if(mutexes_initialized) {
            pthread_mutex_unlock(&mutex);
            pthread_mutex_destroy(&mutex);
            pthread_mutexattr_destroy(&mutex_attr);
            mutexes_initialized = 0;
        }
        memset(devices, 0, sizeof(gt1_info)*128);
        memset(ahp_gt_detected, 0, sizeof(unsigned int)*128);
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
    v &= 0xffff;
    if (v == 0xffff)
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
    return devices[ahp_gt_get_current_device()].accel_increment[axis];
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
    return devices[ahp_gt_get_current_device()].maxspeed [axis];
}

double ahp_gt_get_max_step_frequency(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0.0;
    return devices[ahp_gt_get_current_device()].max_step_frequency[axis];
}

double ahp_gt_get_speed_limit(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0.0;
    return devices[ahp_gt_get_current_device()].speed_limit[axis];
}

double ahp_gt_get_timing(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0.0;
    return devices[ahp_gt_get_current_device()].one_second [axis];
}

void ahp_gt_set_timing(int axis, int value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].one_second [axis] = value;
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
    devices[ahp_gt_get_current_device()].features[axis] = value;
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
    value = fmin(0xf, fmax(0, value));
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
    devices[ahp_gt_get_current_device()].maxspeed[axis] = value;
    optimize_values(axis);
}

void ahp_gt_set_max_step_frequency(int axis, double value)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].max_step_frequency[axis] = value;
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
    int a = 0;
    ahp_gt_detected[ahp_gt_get_current_device()] = 0;
    devices[ahp_gt_get_current_device()].baud_rate = 9600;
    dispatch_command(SetAddress, 0, ahp_gt_current_device);
    if(ahp_gt_get_mc_version() > 0) {
        for (a = 0; a < num_axes; a++) {
            devices[ahp_gt_get_current_device()].steps[a] = 200;
        }
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
    optimize_values(0);
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
        return (SkywatcherAxisStatus){ 0, 0, 0, 0, 0, 0, 0 };
    SkywatcherAxisStatus status;
    int response = dispatch_command(GetAxisStatus, axis, -1);
    status.position = ahp_gt_get_position(axis, &status.timestamp);

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
    dispatch_command(SetAxisPositionCmd, axis, (int)((value-M_PI_2)*devices[ahp_gt_get_current_device()].totalsteps[axis]/M_PI/2.0)+0x800000);
}

double ahp_gt_get_position(int axis, double *timestamp)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0;
    int steps = dispatch_command(GetAxisPosition, axis, -1)-8388608;
    if(timestamp != NULL)
        *timestamp = get_timestamp() - 0.008333333;
    return (double)steps*M_PI*2.0/(double)devices[ahp_gt_get_current_device()].totalsteps[axis];
}

void ahp_gt_set_time(double tm)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    double t = get_timestamp();
    devices[ahp_gt_get_current_device()].time_offset = tm - t;
}

double ahp_gt_get_time()
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0.0;
    double t = get_timestamp();
    return devices[ahp_gt_get_current_device()].time_offset + t;
}

void ahp_gt_set_location(double latitude, double longitude, double elevation)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].lat = latitude;
    devices[ahp_gt_get_current_device()].lon = longitude;
    devices[ahp_gt_get_current_device()].el = elevation;
}

void ahp_gt_get_location(double *latitude, double *longitude, double *elevation)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    *latitude = devices[ahp_gt_get_current_device()].lat;
    *longitude = devices[ahp_gt_get_current_device()].lon;
    *elevation = devices[ahp_gt_get_current_device()].el;
}

int ahp_gt_is_axis_moving(int axis)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0;
    return devices[ahp_gt_get_current_device()].axisstatus[axis].Running;
}

void ahp_gt_goto_altaz(double alt, double az)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    double ra, dec;
    ahp_gt_get_ra_dec_coordinates(alt, az, &ra, &dec);
    ahp_gt_goto_radec(ra, dec);
}

void ahp_gt_goto_radec(double ra, double dec)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    dec = range_dec(dec);
    ra = range_24(ra);
    double ha = get_local_hour_angle(ra);
    if((ahp_gt_get_mount_flags() & isForkMount) == 0) {
        devices[ahp_gt_get_current_device()].will_flip = calc_flipped_ha(&ha, &dec);
    } else {
        devices[ahp_gt_get_current_device()].will_flip = 0;
    }
    dec *= M_PI / 180.0;
    ha *= M_PI / 12.0;
    ahp_gt_goto_absolute(0, ha, 800.0);
    ahp_gt_goto_absolute(1, dec, 800.0);
}

void ahp_gt_sync_radec(double ra, double dec)
{
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    double ha = get_local_hour_angle(ra);
    if((ahp_gt_get_mount_flags() & isForkMount) == 0) {
        devices[ahp_gt_get_current_device()].will_flip = calc_flipped_ha(&ha, &dec);
    } else {
        devices[ahp_gt_get_current_device()].will_flip = 0;
    }
    dec *= M_PI / 180.0;
    ha *= M_PI / 12.0;
    ahp_gt_set_position(0, ha);
    ahp_gt_set_position(1, dec);
}

void ahp_gt_goto_absolute(int axis, double target, double speed) {
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    double position = ahp_gt_get_position(axis, NULL);
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

void ahp_gt_start_tracking_thread() {
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].threads_running = 1;
    pthread_create(&devices[ahp_gt_get_current_device()].tracking_thread, NULL, (void*)&devices[ahp_gt_get_current_device()], track);

}

void ahp_gt_set_tracking_mode(int mode) {
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].tracking_mode = mode;
}

void ahp_gt_start_tracking(int axis) {
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    double period = SIDEREAL_DAY * devices[ahp_gt_get_current_device()].wormsteps[axis] / devices[ahp_gt_get_current_device()].totalsteps[axis];
    dispatch_command (SetStepPeriod, axis, period);
    dispatch_command (Initialize, axis, -1);
    dispatch_command (ActivateMotor, axis, -1);
    dispatch_command (SetMotionMode, axis, 0x10);
    dispatch_command (StartMotion, axis, -1);
}

void ahp_gt_correct_tracking(int axis, double target_period, int *interrupt) {
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    double target_steps = ahp_gt_get_wormsteps(axis) / target_period;
    double one_second = 0;
    double start_time = 0;
    double current_time = 0;
    double time_passed = 0;
    double polltime = 10.0 * SIDEREAL_DAY / ahp_gt_get_totalsteps(axis);
    double initial_second = ahp_gt_get_timing(axis);
    dispatch_command (SetStepPeriod, axis, target_period);
    dispatch_command (Initialize, axis, -1);
    dispatch_command (ActivateMotor, axis, -1);
    dispatch_command (SetMotionMode, axis, 0x10);
    dispatch_command (StartMotion, axis, -1);
    *interrupt = 0;
    double start_steps = ahp_gt_get_position(axis, &start_time) * ahp_gt_get_totalsteps(axis) / M_PI / 2.0;
    while(!*interrupt && time_passed < target_period) {
        double current_steps = ahp_gt_get_position(axis, &current_time) * ahp_gt_get_totalsteps(axis) / M_PI / 2.0 - start_steps;
        current_time = fmax(0, current_time - start_time);
        usleep(fmax(1, fmod(polltime+time_passed-current_time, polltime)*1000000));
        time_passed = current_time;
        one_second = (current_steps / time_passed / target_steps);
    }
    *interrupt = 1;
    ahp_gt_stop_motion(axis, 0);
    one_second *= initial_second;
    ahp_gt_set_timing(axis, one_second);
    WriteAndCheck (axis, axis * 8 + 4, ahp_gt_get_timing(axis));
    dispatch_command (ReloadVars, axis, -1);
}

void ahp_gt_set_aligned(int aligned) {
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return;
    devices[ahp_gt_get_current_device()].is_aligned = aligned;
}

int ahp_gt_is_aligned() {
    if(!ahp_gt_is_detected(ahp_gt_get_current_device()))
        return 0;
    return devices[ahp_gt_get_current_device()].is_aligned;
}
