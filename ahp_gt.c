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
*    SOFTWARE._
*/

#include "ahp_gt.h"
#include <pthread.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>

#include "serial.h"
#define AXES_LIMIT 127

#define HEX(c) (int)(((c) < 'A') ? ((c) - '0') : ((c) - 'A') + 10)
#define MAX_STEP_FREQ 1000
#define DUMP(str, var) sprintf(str, "%s = %x", #var, var);
#ifndef GAMMAJ2000
///Right ascension of the sun at J2000 zero from Greenwich
#define GAMMAJ2000 18.6971378528
#endif
#ifndef SIDEREAL_DAY
#define SIDEREAL_DAY 86164.098903691
#endif
#ifndef SIDEREAL_NOON
#define SIDEREAL_NOON (SIDEREAL_DAY / 2)
#endif
#ifndef SOLAR_DAY
#define SOLAR_DAY 86400
#endif
#ifndef SIDEREAL_24
#define SIDEREAL_24 SIDEREAL_DAY * 24.0 / SOLAR_DAY
#endif

///Axis Indexes
static const char axes[NumAxes][32] = {
"Ra",
"Dec",
"Focus",
"Filter",
"Rotator",
"Iris",
"Shutter",
"Dome",
"Instrument",
"TipX",
"TipY",
"TipZ",
"TiltX",
"TiltY",
"TiltZ",
"InstrumentX",
"InstrumentY",
"InstrumentZ",
"InstrumentRotationX",
"InstrumentRotationY",
"InstrumentRotationZ",
"PhasePrimaryX",
"PhasePrimaryY",
"PhasePrimaryZ",
"PhaseSecondaryX",
"PhaseSecondaryY",
"PhaseSecondaryZ",
"PhaseTertiaryX",
"PhaseTertiaryY",
"PhaseTertiaryZ",
"FrequencyPrimaryX",
"FrequencyPrimaryY",
"FrequencyPrimaryZ",
"FrequencySecondaryX",
"FrequencySecondaryY",
"FrequencySecondaryZ",
"FrequencyTertiaryX",
"FrequencyTertiaryY",
"FrequencyTertiaryZ",
"PCMPrimaryX",
"PCMPrimaryY",
"PCMPrimaryZ",
"PCMSecondaryX",
"PCMSecondaryY",
"PCMSecondaryZ",
"PCMTertiaryX",
"PCMTertiaryY",
"PCMTertiaryZ",
"PlaneX",
"PlaneY",
"PlaneZ",
"RailX",
"RailY",
"RailZ",
///Guide, following indexes add motion compensation to the previous axes, starting  from Ra
"Guide = 64",
};

typedef struct {
    GT_Model model;
    int index;
    int last_step;
    int last_write;
    int last_read;
    int totalsteps;
    int wormsteps;
    int accel_steps;
    int divider;
    int intensity_limited;
    double intensity;
    int multiplier;
    int direction_invert;
    int stepping_conf;
    int stepping_mode;
    int version;
    int features;
    int dividers;
    double maxperiod;
    double minperiod;
    double max_step_frequency;
    double acceleration;
    double crown;
    double steps;
    double motor;
    double worm;
    double guide;
    double one_second;
    double maxspeed;
    double maxspeed_value;
    double accel_increment;
    unsigned char pwmfreq;
    double voltage;
    gt_deviator *deviators;
    int deviators_n;
    SkywatcherMotionMode motionmode;
    SkywatcherAxisStatus axisstatus;
    GTFeature gtfeature;
    int detected;
} gt_axis;

typedef struct {
    int index;
    int num_axes;
    gt_axis axis[AXES_LIMIT];
    char comport[128];
    int rs232_polarity;
    int baud_rate;
    MountType type;
    int flipped;
    int will_flip;
    int tracking_mode;
    GTFlags mount_flags;
    int is_aligned;
    double lat;
    double lon;
    double el;
    time_t time_offset;
    int time_zone;
    int isdst;
    int connfd;
    int threads_running;
    int detected;
} gt_info;

const double rates[9] = { 1, 8, 16, 32, 64, 128, 400, 600, 800 };
static int mutexes_initialized = 0;
static pthread_mutexattr_t mutex_attr;
static pthread_mutex_t mutex;
static pthread_mutex_t mutex_rw;
static pthread_t tracking_thread;
static char command[32];
static char response[32];
static unsigned int ahp_gt_current_device = 0;
static unsigned int ahp_gt_connected = 0;
gt_info devices[128] = { 0 };
int sockfd;

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
    return fmod(SIDEREAL_24 * secs_since_J2000 / SIDEREAL_DAY + Long - GAMMAJ2000, SIDEREAL_24);
}

static double get_lst()
{
    double lat = 0.0, lon = 0.0, el = 0.0;
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

static double range_180(double deg)
{
    if (deg >= 180.0)
        return (360.0 - deg);
    if (deg < -180.0)
        return (deg + 360.0);
    return deg;
}

static double range_360(double deg)
{
    if (deg >= 360.0)
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

static double get_flipped_ha(double *ha, double *dec)
{
    int flipped = 0;
    double lat = 0.0, lon = 0.0, el = 0.0;
    ahp_gt_get_location(&lat, &lon, &el);
    *ha = range_24(*ha);
    if (lat >= 0.0) {
        if(*dec > 0.0) {
            *ha = *ha-12.0;
            flipped = 1;
        }
    }
    if (lat < 0.0) {
        if(*dec < 0.0) {
            *ha = *ha-12.0;
            flipped = 1;
        }
    }
    *ha = range_24(*ha);
    return flipped;
}

static double calc_flipped_ha(double *ha, double *dec)
{
    int flipped = 0;
    double lat = 0.0, lon = 0.0, el = 0.0;
    ahp_gt_get_location(&lat, &lon, &el);
    if (lat >= 0.0) {
        if(*ha > 0.0) {
            *ha = *ha-6.0;
        } else {
            *ha = *ha+6.0;
            *dec = -*dec;
        }
    }
    if (lat < 0.0) {
        if(*ha < 0.0) {
            *ha = *ha-6.0;
        } else {
            *ha = *ha+6.0;
            *dec = -*dec;
        }
    }
    return flipped;
}

static double get_local_hour_angle(double Ra)
{
    double ha = (get_lst() - Ra);
    return range_ha(ha);
}

void radec2rad(double ra, double dec, double *alpha, double *delta, int *flipped)
{
    dec = range_dec(dec);
    ra = range_24(ra);
    double ha = get_local_hour_angle(ra);
    dec -= 90.0;
    if((ahp_gt_get_mount_flags() & isForkMount) == 0) {
        *flipped = calc_flipped_ha(&ha, &dec);
    } else {
        ha -= 6.0;
        *flipped = 0;
    }
    ha = range_ha(ha);
    *alpha = ha * M_PI / 12.0;
    *delta = dec * M_PI / 180.0;
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
    if (sin(ha) > 0.0)
        ha = M_PI * 2.0 - ha;
    ha *= 24.0 / M_PI;
    ra = range_24(get_local_hour_angle(ha));
    *Ra = ra;
    *Dec = dec;
}

double ahp_gt_get_ha()
{
    double lat = 0.0, lon = 0.0, el = 0.0;
    double ha = ahp_gt_get_position(0, NULL) * 12.0 / M_PI + 6.0;
    ahp_gt_get_location(&lat, &lon, &el);
    if (lat >= 0.0)
        ha = range_ha(ha);
    else
        ha = range_ha(24.0 - ha);
    return ha;
}

double ahp_gt_get_ra()
{
    double ha = ahp_gt_get_ha();
    double dec = ahp_gt_get_position(1, NULL) * 180.0 / M_PI;
    if((ahp_gt_get_mount_flags() & isForkMount) == 0) {
        devices[ahp_gt_get_current_device()].flipped = get_flipped_ha(&ha, &dec);
    } else {
        devices[ahp_gt_get_current_device()].flipped = 0;
    }
    return range_24(get_lst() - ha);
}

double ahp_gt_get_dec()
{
    double ha = ahp_gt_get_ha();
    double dec = ahp_gt_get_position(1, NULL) * 180.0 / M_PI;
    if((ahp_gt_get_mount_flags() & isForkMount) == 0) {
        double delta = dec;
        devices[ahp_gt_get_current_device()].flipped = get_flipped_ha(&ha, &delta);
    } else {
        devices[ahp_gt_get_current_device()].flipped = 0;
    }
    dec += 90.0;
    dec = range_dec(dec);
    return dec;
}

static int readcmd(char *cmd, int len)
{
    int n = read(devices[ahp_gt_get_current_device()].connfd, cmd, len);
    cmd[(n < 0 ? 0 : n)] = 0;
    return n;
}

static void *track(void* arg) {
    gt_info *info = arg;
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
        if(ahp_gt_is_detected()) {
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
                sprintf(msg, "%c#", ahp_gt_get_tracking_mode());
                write(devices[ahp_gt_get_current_device()].connfd, msg, 2);
                break;
            case SetTrackingMode:
                if(readcmd(cmd, 1) < 0)
                    goto err_end;
                ahp_gt_set_tracking_mode(cmd[0]);
                sprintf(msg, "#");
                write(devices[ahp_gt_get_current_device()].connfd, msg, 1);
                break;
            case Slew:
                if(readcmd(cmd, 7) < 0)
                    goto err_end;
                switch(cmd[0]) {
                case 1:
                    sprintf(msg, "%c%c#", 0, ahp_gt_get_mc_version(0));
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

static int dispatch_command(SkywatcherCommand cmd, int axis, int arg)
{
    errno = 0;
    int ret = -1;
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

    if(serial_write(command, n) < 0)
        goto ret_err;
    serial_flush();
    switch(cmd) {
        case Flash:
        case ReloadVars:
        case FlashEnable:
        case SetAddress:
            ret = 0;
            break;
        case GetAxisStatus:
            serial_read(response, 10);
            if(response[0] != '!' && response[5] == '\r')
                ret = Highstr2long(response+1);
            else
                ret = -1;
            break;
        case InquireMotorBoardVersion:
        case InquireGridPerRevolution:
        case InquireTimerInterruptFreq:
        case InquireHighSpeedRatio:
        case InquirePECPeriod:
        case GetAxisPosition:
        case GetStepPeriod:
        case GetFeatureCmd:
        case InquireAuxEncoder:
        case GetVars:
            serial_read(response, 10);
            if(response[0] != '!' && response[7] == '\r')
                ret = Revu24str2long(response+1);
            else
                ret = -1;
            break;
        default:
            ret = 0;
        break;
    }
    pthread_mutex_unlock(&mutex);
    return ret;
ret_err:
    pthread_mutex_unlock(&mutex);
    errno = ERANGE;
    return -1;
}

static void optimize_values(int axis)
{
    if(!ahp_gt_is_detected())
        return;
    double baseclock = 375000;
    double usteps = 62.0;
    double maxdiv = 14.0;
    devices[ahp_gt_get_current_device()].axis [axis].wormsteps = (int)(devices[ahp_gt_get_current_device()].axis [axis].steps * devices[ahp_gt_get_current_device()].axis [axis].worm / devices[ahp_gt_get_current_device()].axis [axis].motor);
    devices[ahp_gt_get_current_device()].axis [axis].totalsteps = (int)(devices[ahp_gt_get_current_device()].axis [axis].crown * devices[ahp_gt_get_current_device()].axis [axis].wormsteps);
    int maxsteps = 0xffffff;
    if(devices[ahp_gt_get_current_device()].axis[axis].stepping_mode != HalfStep) {
        maxsteps >>= 6;
    }
    double d = 1.0;
    d += fmin(maxdiv, (double)devices[ahp_gt_get_current_device()].axis [axis].totalsteps / maxsteps);
    devices[ahp_gt_get_current_device()].axis [axis].divider = floor(d);
    while(devices[ahp_gt_get_current_device()].axis [axis].divider > 1 && usteps > 14) {
        devices[ahp_gt_get_current_device()].axis [axis].divider --;
        usteps /= 2;
        usteps --;
    }
    devices[ahp_gt_get_current_device()].axis [axis].multiplier = 1;
    switch(devices[ahp_gt_get_current_device()].axis[axis].stepping_mode) {
    case HalfStep:
        devices[ahp_gt_get_current_device()].axis [axis].multiplier = 1;
        break;
    case Microstep:
        devices[ahp_gt_get_current_device()].axis [axis].multiplier += (int)usteps;
        break;
    case Mixed:
        devices[ahp_gt_get_current_device()].axis [axis].multiplier += (int)usteps;
        break;
    default:
        break;
    }
    devices[ahp_gt_get_current_device()].axis [axis].wormsteps *= (double)devices[ahp_gt_get_current_device()].axis [axis].multiplier / (double)devices[ahp_gt_get_current_device()].axis [axis].divider;
    devices[ahp_gt_get_current_device()].axis [axis].totalsteps = (int)(devices[ahp_gt_get_current_device()].axis [axis].crown * devices[ahp_gt_get_current_device()].axis [axis].wormsteps);

    double totalsteps = (double)ahp_gt_get_totalsteps(axis) * ahp_gt_get_divider(axis) / ahp_gt_get_multiplier(axis);
    double steps_s = totalsteps / SIDEREAL_DAY;
    double sidereal_period = SIDEREAL_DAY * devices[ahp_gt_get_current_device()].axis [axis].multiplier * devices[ahp_gt_get_current_device()].axis [axis].wormsteps / devices[ahp_gt_get_current_device()].axis [axis].totalsteps;
    devices[ahp_gt_get_current_device()].axis [axis].maxperiod = (int)sidereal_period;
    devices[ahp_gt_get_current_device()].axis [axis].minperiod = 1;
    devices[ahp_gt_get_current_device()].axis [axis].maxspeed_value = (int)fmax(devices[ahp_gt_get_current_device()].axis [axis].minperiod, (devices[ahp_gt_get_current_device()].axis [axis].maxperiod / devices[ahp_gt_get_current_device()].axis [axis].maxspeed));
    devices[ahp_gt_get_current_device()].axis [axis].guide = (int)(SIDEREAL_DAY * baseclock / devices[ahp_gt_get_current_device()].axis [axis].totalsteps);

    double degrees = devices[ahp_gt_get_current_device()].axis [axis].acceleration * (double)devices[ahp_gt_get_current_device()].axis [axis].totalsteps / devices[ahp_gt_get_current_device()].axis [axis].multiplier / (M_PI * 2.0);
    devices[ahp_gt_get_current_device()].axis [axis].accel_steps = floor(fmin(63, pow(degrees * 2, 0.4) + 1));
    devices[ahp_gt_get_current_device()].axis [axis].accel_increment = (int)fmin (0xff, devices[ahp_gt_get_current_device()].axis [axis].guide / devices[ahp_gt_get_current_device()].axis [axis].accel_increment);
    devices[ahp_gt_get_current_device()].index &= 0x7f;
    devices[ahp_gt_get_current_device()].rs232_polarity &= 0x1;
    int dividers;
    dividers = devices[ahp_gt_get_current_device()].rs232_polarity | (devices[ahp_gt_get_current_device()].index << 9);
    dividers |= (((unsigned char)(devices[ahp_gt_get_current_device()].axis [0].divider) & 0xf) << 1) | ((((unsigned char)(devices[ahp_gt_get_current_device()].axis[1].divider)) & 0xf) << 5);
    if(devices[ahp_gt_get_current_device()].axis[axis].model == GT1 || devices[ahp_gt_get_current_device()].axis[axis].model == GT2) {
        devices[ahp_gt_get_current_device()].axis [0].dividers = dividers;
        devices[ahp_gt_get_current_device()].axis [1].dividers = dividers;
    } else
        devices[ahp_gt_get_current_device()].axis [axis].dividers = dividers;
    devices[ahp_gt_get_current_device()].axis [axis].detected = 1;
}
int ahp_gt_reset(int axis)
{
    int ret = 0;
    int ntries = 10;
    while (!ret && ntries-- > 0)
    {
        ret = dispatch_command(FlashEnable, axis, -1);
        if (ret>-1)
        {
            ret = dispatch_command(Flash, axis, -1);
            if (ret>-1)
            {
                ntries = 0;
                return 1;
            }
            ret = 0;
        }
        ret = 0;
    }
    return -1;
}

int ahp_gt_reload(int axis)
{
    int ret = 0;
    int ntries = 10;
    while (!ret && ntries-- > 0)
    {
        ret = dispatch_command(FlashEnable, axis, -1);
        if (ret>-1)
        {
            ret = dispatch_command(ReloadVars, axis, -1);
            if (ret>-1)
            {
                ntries = 0;
                return 1;
            }
            ret = 0;
        }
        ret = 0;
    }
    return -1;
}

int ahp_gt_read(int axis, int pos)
{
    int ret = 0;
    int ntries = 50;
    while (!ret && ntries-- > 0)
    {
        usleep(10000);
        ret = dispatch_command(FlashEnable, axis, -1);
        if (ret>-1)
        {
            usleep(10000);
            ret = dispatch_command(GetVars, pos, -1);
            if (ret>-1)
            {
                ntries = 0;
                return ret;
            }
            ret = 0;
        }
    }
    return -1;
}

int ahp_gt_write_and_verify(int axis, int pos, int val)
{
    int ret = 0;
    int ntries = 10;
    while (!ret && ntries-- > 0)
    {
        usleep(10000);
        ret = dispatch_command(FlashEnable, axis, -1);
        if (ret>-1)
        {
            usleep(10000);
            ret = dispatch_command(SetVars, pos, val);
            if (ret>-1)
            {
                usleep(10000);
                if (ahp_gt_read(axis, pos) == val) {
                    ntries = 0;
                    return 1;
                }
                ret = 0;
            }
            ret = 0;
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
    if(!ahp_gt_is_detected())
        return;
    optimize_values(axis);
    int offset = 0;
    if(devices[ahp_gt_get_current_device()].axis [axis].model == GT1) offset = axis * 8;
    int _percent, _finished;
    if(percent == NULL)
        percent = &_percent;
    if(finished == NULL)
        finished = &_finished;
    *percent = 0;
    *finished = 0;
    int dividers = devices[ahp_gt_get_current_device()].axis [axis].dividers;
    int mount_flags = devices[ahp_gt_get_current_device()].mount_flags & ~0x1;
    if((ahp_gt_get_mount_flags() & isForkMount) && ((devices[ahp_gt_get_current_device()].axis [axis].version & 0xf) == 1)) {
        mount_flags |= 1;
    }
    int values[] = {
    devices[ahp_gt_get_current_device()].axis [axis].totalsteps,
    devices[ahp_gt_get_current_device()].axis [axis].wormsteps,
    devices[ahp_gt_get_current_device()].axis [axis].maxspeed_value,
    devices[ahp_gt_get_current_device()].axis [axis].guide,
    devices[ahp_gt_get_current_device()].axis [axis].one_second,
    (((int)(devices[ahp_gt_get_current_device()].axis [axis].accel_increment) & 0xff) << 10) | (((int)devices[ahp_gt_get_current_device()].axis [axis].accel_steps & 0x3f) << 18) | (((int)devices[ahp_gt_get_current_device()].axis [axis].multiplier & 0x7f) << 3) | ((devices[ahp_gt_get_current_device()].axis[axis].stepping_conf & 0x03) << 1) | (devices[ahp_gt_get_current_device()].axis[axis].direction_invert & 1),
    (int)devices[ahp_gt_get_current_device()].axis [axis].features,
    (((int)(devices[ahp_gt_get_current_device()].axis[axis].pwmfreq << 4)) & 0x30) | (((int)devices[ahp_gt_get_current_device()].axis[0].stepping_mode << 6) & 0xc0) | ((mount_flags << 3) & 0x8) | ((int)devices[ahp_gt_get_current_device()].axis[0].gtfeature & 0x7) | ((((unsigned char)devices[ahp_gt_get_current_device()].type)<<16)&0xff0000) | (int)((dividers<<8)&0xff00),
    (((int)(devices[ahp_gt_get_current_device()].axis[axis].pwmfreq << 2)) & 0x30) | (((int)devices[ahp_gt_get_current_device()].axis[1].stepping_mode << 6) & 0xc0) | ((mount_flags << 2) & 0x8) | ((int)devices[ahp_gt_get_current_device()].axis[1].gtfeature & 0x7) | (((mount_flags)<<14)&0xff0000) | (int)((dividers)&0xff00),
    (((int)(devices[ahp_gt_get_current_device()].axis[axis].pwmfreq << 4)) & 0x30) | (((int)devices[ahp_gt_get_current_device()].axis[axis].stepping_mode << 6) & 0xc0) | ((mount_flags << 3) & 0x8) | ((int)devices[ahp_gt_get_current_device()].axis[axis].gtfeature & 0x7) | ((((unsigned char)devices[ahp_gt_get_current_device()].type)<<16)&0xff0000) | (int)((dividers<<8)&0xff00),
    (((int)(devices[ahp_gt_get_current_device()].axis[axis].pwmfreq << 2)) & 0x30) | (((int)devices[ahp_gt_get_current_device()].axis[axis].stepping_mode << 6) & 0xc0) | ((mount_flags << 2) & 0x8) | ((int)devices[ahp_gt_get_current_device()].axis[axis].gtfeature & 0x7) | (((mount_flags)<<14)&0xff0000) | (int)((dividers)&0xff00),
    (((int)(devices[ahp_gt_get_current_device()].axis [axis].index << 12) & 0x7f000) | ((int)devices[ahp_gt_get_current_device()].axis[axis].intensity & 0x3ff) | (devices[ahp_gt_get_current_device()].axis[axis].intensity_limited ? 0x400 : 0)),
    };
    int idx = 0;
    if (!ahp_gt_write_and_verify (axis, offset + 0, values[idx++])) {
        *finished = -1;
        return;
    }
    *percent = *percent + 10;
    if (!ahp_gt_write_and_verify (axis, offset + 1, values[idx++])) {
        *finished = -1;
        return;
    }
    *percent = *percent + 10;
    if (!ahp_gt_write_and_verify (axis, offset + 2, values[idx++])) {
        *finished = -1;
        return;
    }
    *percent = *percent + 10;
    if (!ahp_gt_write_and_verify (axis, offset + 3, values[idx++])) {
        *finished = -1;
        return;
    }
    *percent = *percent + 10;
    if (!ahp_gt_write_and_verify (axis, offset + 4, values[idx++])) {
        *finished = -1;
        return;
    }
    *percent = *percent + 10;
    if (!ahp_gt_write_and_verify (axis, offset + 5, values[idx++])) {
        *finished = -1;
        return;
    }
    *percent = *percent + 10;
    if (!ahp_gt_write_and_verify (axis, offset + 6, values[idx++])) {
        *finished = -1;
        return;
    }
    *percent = *percent + 10;
    if(devices[ahp_gt_get_current_device()].axis [axis].model == GT5) {
        idx++;
        idx++;
    }
    if (!ahp_gt_write_and_verify (axis, 7, values[idx++])) {
        *finished = -1;
        return;
    }
    *percent = *percent + 10;
    if (!ahp_gt_write_and_verify (axis, 15, values[idx++])) {
        *finished = -1;
        return;
    }
    *percent = *percent + 10;
    if(devices[ahp_gt_get_current_device()].axis [axis].model == GT5 || devices[ahp_gt_get_current_device()].axis [axis].model == GT2) {
        if (!ahp_gt_write_and_verify (axis, 8, (devices[ahp_gt_get_current_device()].axis [axis].index<<12)|((ahp_gt_is_intensity_limited(axis)<<10)|(int)ahp_gt_get_intensity_limit(axis)))) {
            *finished = -1;
            return;
        }
    }
    *percent = *percent + 10;
    devices[ahp_gt_get_current_device()].axis [axis].last_write = time(NULL);
    *finished = 1;
}

void ahp_gt_read_values(int axis)
{
    if(!ahp_gt_is_connected())
        return;
    if(!ahp_gt_axis_is_detected(axis))
        return;
    if(devices[ahp_gt_get_current_device()].axis [axis].last_read > devices[ahp_gt_get_current_device()].axis [axis].last_write)
        return;
    int offset = 0;
    double steps, totalsteps, wormsteps, multiplier;
    steps = 200;
    long value = dispatch_command(InquireGridPerRevolution, axis, -1);
    if(value > 0)
        totalsteps = value;
    else
        totalsteps = 9216000;
    value = dispatch_command(InquireTimerInterruptFreq, axis, -1);
    if(value > 0)
        wormsteps = value;
    else
        wormsteps = 51200;
    value = dispatch_command(InquireHighSpeedRatio, axis, -1);
    if(value > 0)
        multiplier = value;
    else
        multiplier = 64;
    double crown = (double)totalsteps / wormsteps;
    double worm = (double)wormsteps / steps / multiplier;
    double motor = 1;
    double decimals = fabs(worm - round(worm));
    if(decimals == 0) decimals++;
    motor = 1.0 / decimals;
    worm /= decimals;
    devices[ahp_gt_get_current_device()].axis [axis].steps = fabs(round(steps));
    devices[ahp_gt_get_current_device()].axis [axis].crown = fabs(round(crown));
    devices[ahp_gt_get_current_device()].axis [axis].motor = fabs(round(motor));
    devices[ahp_gt_get_current_device()].axis [axis].worm = fabs(round(worm));
    devices[ahp_gt_get_current_device()].axis [axis].totalsteps = fabs(round(totalsteps));
    devices[ahp_gt_get_current_device()].axis [axis].wormsteps = fabs(round(wormsteps));
    devices[ahp_gt_get_current_device()].axis [axis].multiplier = fabs(round(multiplier));
    devices[ahp_gt_get_current_device()].axis [axis].maxspeed_value = 50;
    devices[ahp_gt_get_current_device()].axis [axis].guide = 12709;
    devices[ahp_gt_get_current_device()].axis [axis].one_second = 1500000;
    devices[ahp_gt_get_current_device()].axis [axis].accel_increment = 43;
    devices[ahp_gt_get_current_device()].axis [axis].accel_steps =  63;
    devices[ahp_gt_get_current_device()].axis [axis].direction_invert = 0;
    devices[ahp_gt_get_current_device()].axis [axis].stepping_conf = 0;
    devices[ahp_gt_get_current_device()].axis [axis].features = 16384;
    devices[ahp_gt_get_current_device()].axis[axis].gtfeature = 0;
    devices[ahp_gt_get_current_device()].axis[axis].stepping_mode = 0;
    devices[ahp_gt_get_current_device()].axis[axis].divider = 1;
    devices[ahp_gt_get_current_device()].axis [axis].pwmfreq = 5;
    devices[ahp_gt_get_current_device()].type = 0;
    devices[ahp_gt_get_current_device()].mount_flags = 0;
    devices[ahp_gt_get_current_device()].axis [axis].dividers = 0;
    devices[ahp_gt_get_current_device()].index = 1;
    devices[ahp_gt_get_current_device()].rs232_polarity = 0;
    if((devices[ahp_gt_get_current_device()].axis [axis].model != GT1) && (devices[ahp_gt_get_current_device()].axis [axis].model != GT2) && (devices[ahp_gt_get_current_device()].axis [axis].model != GT5)) {
        return;
    }

    if((devices[ahp_gt_get_current_device()].axis [axis].version & 0xf) == 1)
        offset = axis * 8;
    value = ahp_gt_read(axis, offset + 2);
    if(value > 0)
        devices[ahp_gt_get_current_device()].axis [axis].maxspeed_value = value;
    value = ahp_gt_read(axis, offset + 3);
    if(value > 0)
    devices[ahp_gt_get_current_device()].axis [axis].guide = value;
    value = ahp_gt_read(axis, offset + 4);
    if(value > 0)
    devices[ahp_gt_get_current_device()].axis [axis].one_second = value;
    value = ahp_gt_read(axis, offset + 5);
    if(value > 0) {
        devices[ahp_gt_get_current_device()].axis [axis].accel_steps = ((value >> 18) & 0x3f);
        devices[ahp_gt_get_current_device()].axis [axis].accel_increment =  (value >> 10) & 0xff;
        devices[ahp_gt_get_current_device()].axis [axis].direction_invert = value & 0x1;
        devices[ahp_gt_get_current_device()].axis [axis].stepping_conf = (value & 0x06)>>1;
        double degrees = pow(devices[ahp_gt_get_current_device()].axis [axis].accel_steps - 1, 2.5) / 2.0;
        devices[ahp_gt_get_current_device()].axis [axis].acceleration = degrees / ((double)devices[ahp_gt_get_current_device()].axis [axis].totalsteps / devices[ahp_gt_get_current_device()].axis [axis].multiplier / (M_PI * 2.0));
    }
    value = ahp_gt_read(axis, offset + 6);
    if(value > 0) {
        devices[ahp_gt_get_current_device()].axis [axis].features = ahp_gt_read(axis, offset + 6);
    }
    value = ahp_gt_read(axis, offset + 7);
    if(value > 0) {
        devices[ahp_gt_get_current_device()].axis[axis].pwmfreq = (value & 0x30) >> 4;
        devices[ahp_gt_get_current_device()].axis[axis].gtfeature = value & 0x7;
        devices[ahp_gt_get_current_device()].axis[axis].stepping_mode = (value >> 6) & 0x03;
        devices[ahp_gt_get_current_device()].axis [axis].dividers = (value >> 8) & 0xff;
        devices[ahp_gt_get_current_device()].mount_flags = (value & 0x8) >> 3;
        devices[ahp_gt_get_current_device()].type = (value >> 16) & 0xff;
    }
    value = ahp_gt_read(axis, offset + 15);
    if(value > 0) {
        devices[ahp_gt_get_current_device()].axis[axis].pwmfreq |= (value & 0x30) >> 2;
        devices[ahp_gt_get_current_device()].axis[axis].gtfeature = value & 0x7;
        devices[ahp_gt_get_current_device()].axis[axis].stepping_mode = (value >> 6) & 0x03;
        devices[ahp_gt_get_current_device()].axis [axis].dividers |= value & 0xff00;
        devices[ahp_gt_get_current_device()].mount_flags |= (value & 0x8) >> 2;
        devices[ahp_gt_get_current_device()].mount_flags |= (value & 0xff0000) >> 14;
        devices[ahp_gt_get_current_device()].mount_flags &= 0x3ff;
        devices[ahp_gt_get_current_device()].mount_flags &= ~0x1;
        if((ahp_gt_get_mount_flags() & isForkMount) != 0) {
            devices[ahp_gt_get_current_device()].mount_flags |= 1;
        }
    }
    if(devices[ahp_gt_get_current_device()].axis [axis].model == GT1 || devices[ahp_gt_get_current_device()].axis [axis].model == GT2)
        devices[ahp_gt_get_current_device()].axis[axis].divider = (devices[ahp_gt_get_current_device()].axis [axis].dividers >> (1+axis*4)) & 0xf;
    else if(devices[ahp_gt_get_current_device()].axis [axis].model == GT5)
        devices[ahp_gt_get_current_device()].axis[axis].divider = (devices[ahp_gt_get_current_device()].axis [axis].dividers >> 1) & 0xf;
    if(devices[ahp_gt_get_current_device()].axis [axis].model == GT2 || devices[ahp_gt_get_current_device()].axis [axis].model == GT5) {
        value = ahp_gt_read(axis, 8);
        if(value > 0) {
            devices[ahp_gt_get_current_device()].axis[axis].intensity = value & 0x3ff;
            devices[ahp_gt_get_current_device()].axis[axis].intensity_limited = (value & 0x400) != 0;
        }
    }
    worm = (double)wormsteps / steps / multiplier / devices[ahp_gt_get_current_device()].axis[axis].divider;
    motor = 1;
    decimals = fabs(worm - round(worm));
    if(decimals == 0) decimals++;
    motor = 1.0 / decimals;
    worm /= decimals;
    devices[ahp_gt_get_current_device()].index = (devices[ahp_gt_get_current_device()].axis [axis].dividers >> 9) & 0x7f;
    devices[ahp_gt_get_current_device()].rs232_polarity = devices[ahp_gt_get_current_device()].axis [axis].dividers & 1;
    double sidereal_period = SIDEREAL_DAY * devices[ahp_gt_get_current_device()].axis [axis].multiplier * devices[ahp_gt_get_current_device()].axis [axis].wormsteps / devices[ahp_gt_get_current_device()].axis [axis].totalsteps;
    devices[ahp_gt_get_current_device()].axis [axis].maxspeed = sidereal_period / devices[ahp_gt_get_current_device()].axis [axis].maxspeed_value;
    optimize_values(axis);
    devices[ahp_gt_get_current_device()].axis [axis].last_read = time(NULL);
}

void ahp_gt_clear()
{
    int d;
    memset(devices, 0, sizeof(gt_info)*128);
    for(d = 0; d < 128; d++) {
        memset(devices[d].axis, 0, sizeof(gt_axis)*AXES_LIMIT);
        devices[d].baud_rate = 9600;
    }
}

int ahp_gt_connect_fd(int fd)
{
    if(ahp_gt_is_connected())
        return 0;
    if(fd != -1) {
        serial_set_fd(fd, 9600);
        ahp_gt_connected = 1;
        return 0;
    }
    return 1;
}

void ahp_gt_set_fd(int fd)
{
    if(!ahp_gt_is_connected())
        return;
    serial_set_fd(fd, 9600);
}

int ahp_gt_get_fd()
{
    if(!ahp_gt_is_connected())
        return -1;
    return serial_get_fd();
}

int ahp_gt_connect_udp(const char *address, int port)
{
    if(ahp_gt_is_connected())
        return 0;
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
    serial_connect(port, 9600, "8N1");
    if(!serial_is_open()) {
        ahp_gt_connected = 0;
        serial_close();
        return 1;
    }
    strcpy(devices[ahp_gt_get_current_device()].comport, port);
    ahp_gt_connected = 1;
    return 0;
}

void ahp_gt_set_high_rate(int value)
{
    if(ahp_gt_is_connected())
        return;
    serial_close();
    serial_connect(devices[ahp_gt_get_current_device()].comport, value?115200:9600, "8N1");
}

void ahp_gt_disconnect()
{
    int addr = 0;
    if(ahp_gt_is_connected()) {
        ahp_gt_select_device(0);
        serial_close();
        if(mutexes_initialized) {
            pthread_mutex_unlock(&mutex);
            pthread_mutex_destroy(&mutex);
            pthread_mutexattr_destroy(&mutex_attr);
            mutexes_initialized = 0;
        }
        memset(devices, 0, sizeof(gt_info)*128);
        ahp_gt_connected = 0;
    }
}

unsigned int ahp_gt_is_connected()
{
    return ahp_gt_connected;
}

unsigned int ahp_gt_is_detected()
{
    return devices[ahp_gt_get_current_device()].detected;
}

unsigned int ahp_gt_axis_is_detected(int axis)
{
    return devices[ahp_gt_get_current_device()].axis[axis].detected;
}

int ahp_gt_get_mc_version(int axis)
{
    int v = dispatch_command(InquireMotorBoardVersion, axis, -1);
    v &= 0xffff;
    if (v == 0xffff)
        v = -1;
    switch (v&0xf) {
        case 1:
            devices[ahp_gt_get_current_device()].axis[axis].model = GT1;
            break;
        case 2:
        case 3:
        case 7:
        case 8:
            devices[ahp_gt_get_current_device()].axis[axis].model = GT2;
            break;
        case 4:
        case 5:
            devices[ahp_gt_get_current_device()].axis[axis].model = GT5;
            break;
        default:
            devices[ahp_gt_get_current_device()].axis[axis].model = GT1;
            break;
    }
    return v;
}

MountType ahp_gt_get_mount_type()
{
    if(!ahp_gt_is_detected())
        return 0;
    return devices[ahp_gt_get_current_device()].type;
}

GTFeature ahp_gt_get_feature(int axis)
{
    if(!ahp_gt_is_detected())
        return 0;
    return devices[ahp_gt_get_current_device()].axis[axis].gtfeature;
}

SkywatcherFeature ahp_gt_get_features(int axis)
{
    if(!ahp_gt_is_detected())
        return 0;
    return (SkywatcherFeature)devices[ahp_gt_get_current_device()].axis[axis].features;
}

double ahp_gt_get_motor_steps(int axis)
{
    if(!ahp_gt_is_detected())
        return 0.0;
    return devices[ahp_gt_get_current_device()].axis[axis].steps;
}

double ahp_gt_get_motor_teeth(int axis)
{
    if(!ahp_gt_is_detected())
        return 0.0;
    return devices[ahp_gt_get_current_device()].axis[axis].motor;
}

double ahp_gt_get_worm_teeth(int axis)
{
    if(!ahp_gt_is_detected())
        return 0.0;
    return devices[ahp_gt_get_current_device()].axis[axis].worm;
}

double ahp_gt_get_crown_teeth(int axis)
{
    if(!ahp_gt_is_detected())
        return 0.0;
    return devices[ahp_gt_get_current_device()].axis[axis].crown;
}

double ahp_gt_get_multiplier(int axis)
{
    if(!ahp_gt_is_detected())
        return 0.0;
    return devices[ahp_gt_get_current_device()].axis[axis].multiplier;
}

double ahp_gt_get_divider(int axis)
{
    if(!ahp_gt_is_detected())
        return 0.0;
    return devices[ahp_gt_get_current_device()].axis[axis].divider;
}

int ahp_gt_get_totalsteps(int axis)
{
    if(!ahp_gt_is_detected())
        return 0;
    return devices[ahp_gt_get_current_device()].axis[axis].totalsteps;
}

int ahp_gt_get_wormsteps(int axis)
{
    if(!ahp_gt_is_detected())
        return 0;
    return devices[ahp_gt_get_current_device()].axis[axis].wormsteps;
}

double ahp_gt_get_guide_steps(int axis)
{
    if(!ahp_gt_is_detected())
        return 0.0;
    return devices[ahp_gt_get_current_device()].axis[axis].guide * M_PI * 2 / SIDEREAL_DAY;
}

double ahp_gt_get_acceleration_steps(int axis)
{
    if(!ahp_gt_is_detected())
        return 0.0;
    return devices[ahp_gt_get_current_device()].axis[axis].accel_increment;
}

double ahp_gt_get_acceleration_angle(int axis)
{
    if(!ahp_gt_is_detected())
        return 0.0;
    return devices[ahp_gt_get_current_device()].axis[axis].acceleration;
}

int ahp_gt_get_rs232_polarity()
{
    if(!ahp_gt_is_detected())
        return 0;
    return devices[ahp_gt_get_current_device()].rs232_polarity;
}

void ahp_gt_limit_intensity(int axis, int value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis[axis].intensity_limited = value;
    int oldvalue = dispatch_command(GetVars, 9, -1);
    oldvalue = (oldvalue >> 16) | (oldvalue << 16) | oldvalue & 0xff00;
    oldvalue &= ~0x0400;
    oldvalue |= value ? 0x0400 : 0;
    oldvalue = (oldvalue >> 16) | (oldvalue << 16) | oldvalue & 0xff00;
    dispatch_command(FlashEnable, axis, -1);
    dispatch_command(SetVars, 9, oldvalue);
    dispatch_command(ReloadVars, axis, -1);
}

int ahp_gt_is_intensity_limited(int axis)
{
    if(!ahp_gt_is_detected())
        return 0;
    return (devices[ahp_gt_get_current_device()].axis[axis].intensity_limited);
}

void ahp_gt_set_intensity_limit(int axis, double value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis[axis].intensity = value;
    dispatch_command(FlashEnable, axis, -1);
    int oldvalue = dispatch_command(GetVars, 9, -1);
    oldvalue = (oldvalue >> 16) | (oldvalue << 16) | oldvalue & 0xff00;
    oldvalue &= ~0x03ff;
    oldvalue |= (int)value & 0x03ff;
    oldvalue = (oldvalue >> 16) | (oldvalue << 16) | oldvalue & 0xff00;
    dispatch_command(FlashEnable, axis, -1);
    dispatch_command(SetVars, 9, oldvalue);
    dispatch_command(ReloadVars, axis, -1);

}

double ahp_gt_get_intensity_limit(int axis)
{
    if(!ahp_gt_is_detected())
        return 0;
    return devices[ahp_gt_get_current_device()].axis[axis].intensity;
}

int ahp_gt_get_pwm_frequency(int axis)
{
    if(!ahp_gt_is_detected())
        return 0;
    return 0xf-devices[ahp_gt_get_current_device()].axis [axis].pwmfreq;
}

int ahp_gt_get_direction_invert(int axis)
{
    if(!ahp_gt_is_detected())
        return 0;
    return devices[ahp_gt_get_current_device()].axis[axis].direction_invert;
}

GTFlags ahp_gt_get_mount_flags()
{
    if(!ahp_gt_is_detected())
        return 0;
    return devices[ahp_gt_get_current_device()].mount_flags;
}

GTSteppingConfiguration ahp_gt_get_stepping_conf(int axis)
{
    if(!ahp_gt_is_detected())
        return 0;
    return (GTSteppingConfiguration)devices[ahp_gt_get_current_device()].axis[axis].stepping_conf;
}

GTSteppingMode ahp_gt_get_stepping_mode(int axis)
{
    if(!ahp_gt_is_detected())
        return 0;
    return (GTSteppingMode)devices[ahp_gt_get_current_device()].axis[axis].stepping_mode;
}

double ahp_gt_get_max_speed(int axis)
{
    if(!ahp_gt_is_detected())
        return 0.0;
    return devices[ahp_gt_get_current_device()].axis [axis].maxspeed * M_PI * 2 / SIDEREAL_DAY;
}

double ahp_gt_get_max_step_frequency(int axis)
{
    if(!ahp_gt_is_detected())
        return 0.0;
    return devices[ahp_gt_get_current_device()].axis[axis].max_step_frequency;
}

double ahp_gt_get_timing(int axis)
{
    if(!ahp_gt_is_detected())
        return 0.0;
    return devices[ahp_gt_get_current_device()].axis [axis].one_second;
}

void ahp_gt_set_timing(int axis, int value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis [axis].one_second = value;
}

void ahp_gt_set_mount_type(MountType value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].type = value;
}

void ahp_gt_delete_axis(int axis)
{
    if(!ahp_gt_is_detected())
        return;
    memset(&devices[ahp_gt_get_current_device()].axis [axis], 0, sizeof(gt_axis));
}

void ahp_gt_copy_axis(int axis, int value)
{
    if(!ahp_gt_is_detected())
        return;
    memcpy(&devices[ahp_gt_get_current_device()].axis [value], &devices[ahp_gt_get_current_device()].axis [axis], sizeof(gt_axis));
    dispatch_command(FlashEnable, axis, -1);
    int oldvalue = dispatch_command(GetVars, 9, -1);
    oldvalue = (oldvalue >> 16) | (oldvalue << 16) | oldvalue & 0xff00;
    oldvalue &= ~0xff000;
    oldvalue |= value << 12;
    oldvalue = (oldvalue >> 16) | (oldvalue << 16) | oldvalue & 0xff00;
    dispatch_command(FlashEnable, axis, -1);
    dispatch_command(SetVars, 9, oldvalue);
    dispatch_command(ReloadVars, axis, -1);
}

void ahp_gt_move_axis(int axis, int value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis [axis].index = value;
}

void ahp_gt_set_axes_limit(int value)
{
    devices[ahp_gt_get_current_device()].num_axes = value;
}

int ahp_gt_get_axes_limit()
{
    return devices[ahp_gt_get_current_device()].num_axes;
}

void ahp_gt_set_features(int axis, SkywatcherFeature value)
{
    if(!ahp_gt_is_detected())
        return;
    if(devices[ahp_gt_get_current_device()].axis[axis].model == GT1 || devices[ahp_gt_get_current_device()].axis[axis].model == GT2) {
        devices[ahp_gt_get_current_device()].axis[0].features = value;
        devices[ahp_gt_get_current_device()].axis[1].features = value;
    } else
        devices[ahp_gt_get_current_device()].axis[axis].features = value;
}

void ahp_gt_set_feature(int axis, GTFeature value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis[axis].gtfeature = value & 7;
    optimize_values(axis);
}

void ahp_gt_set_motor_steps(int axis, double value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis[axis].steps = value;
    optimize_values(axis);
}

void ahp_gt_set_motor_teeth(int axis, double value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis[axis].motor = value;
    optimize_values(axis);
}

void ahp_gt_set_worm_teeth(int axis, double value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis[axis].worm = value;
    optimize_values(axis);
}

void ahp_gt_set_crown_teeth(int axis, double value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis[axis].crown = value;
    optimize_values(axis);
}

void ahp_gt_set_guide_steps(int axis, double value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis[axis].guide = value * SIDEREAL_DAY / M_PI / 2;
    optimize_values(axis);
}

void ahp_gt_set_pwm_frequency(int axis, int value)
{
    if(!ahp_gt_is_detected())
        return;
    value = 0xf-value;
    if(devices[ahp_gt_get_current_device()].axis[axis].model == GT1 || devices[ahp_gt_get_current_device()].axis[axis].model == GT2) {
        devices[ahp_gt_get_current_device()].axis [0].pwmfreq = value;
        devices[ahp_gt_get_current_device()].axis [1].pwmfreq = value;
    }
    else
        devices[ahp_gt_get_current_device()].axis [axis].pwmfreq = value;
    optimize_values(axis);
}

void ahp_gt_set_acceleration_angle(int axis, double value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis[axis].acceleration = value;
    optimize_values(axis);
}

void ahp_gt_set_rs232_polarity(int value)
{
    int a;
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].rs232_polarity = value&1;
    for(a = 0; a < ahp_gt_get_axes_limit(); a++)
        optimize_values(a);
}

void ahp_gt_set_direction_invert(int axis, int value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis[axis].direction_invert = value&1;
    optimize_values(axis);
}

void ahp_gt_set_mount_flags(GTFlags value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].mount_flags = value;
}

void ahp_gt_set_stepping_conf(int axis, GTSteppingConfiguration value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis[axis].stepping_conf = (int)value;
    optimize_values(axis);
}

void ahp_gt_set_stepping_mode(int axis, GTSteppingMode value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis[axis].stepping_mode = (int)value;
    optimize_values(axis);
}

void ahp_gt_set_max_speed(int axis, double value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis[axis].maxspeed = value * SIDEREAL_DAY / M_PI / 2;
    optimize_values(axis);
}

void ahp_gt_set_max_step_frequency(int axis, double value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis[axis].max_step_frequency = value;
    optimize_values(axis);
}

void ahp_gt_set_divider(int axis, int value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis[axis].divider = abs(value);
    dispatch_command(FlashEnable, axis, -1);
    int value1 = dispatch_command(GetVars, 7, -1);
    value1 = (value1 >> 16) | (value1 << 16) | value1 & 0xff00;
    int dividers = (value1 & 0xff00) >> 8;
    value1 &= ~0xff00;
    dispatch_command(FlashEnable, axis, -1);
    int value2 = dispatch_command(GetVars, 7, -1);
    value2 = (value2 >> 16) | (value2 << 16) | value2 & 0xff00;
    dividers |= value2 & 0xff00;
    value2 &= ~0xff00;
    if(devices[ahp_gt_get_current_device()].axis[axis].model == GT1) {
        dividers &= ~(0x1e << (axis * 4));
        dividers |= value << (axis * 4);
    } else {
        dividers &= ~0x1fe;
        dividers |= (value << 1) | (value << 5);
    }
    value1 |= (dividers << 8) & 0xff00;
    value2 |= (dividers) & 0xff00;
    value1 = (value1 >> 16) | (value1 << 16) | value1 & 0xff00;
    value2 = (value2 >> 16) | (value2 << 16) | value2 & 0xff00;
    dispatch_command(FlashEnable, axis, -1);
    dispatch_command(SetVars, 7, value1);
    dispatch_command(ReloadVars, axis, -1);
    dispatch_command(FlashEnable, axis, -1);
    dispatch_command(SetVars, 15, value2);
    dispatch_command(ReloadVars, axis, -1);
}

void ahp_gt_set_multiplier(int axis, int value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis[axis].multiplier = abs(value);
}

void ahp_gt_set_totalsteps(int axis, int value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis[axis].totalsteps = abs(value);
}

void ahp_gt_set_wormsteps(int axis, int value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis[axis].wormsteps = abs(value);
}

void ahp_gt_set_voltage(int axis, double value) {
    devices[ahp_gt_get_current_device()].axis[axis].voltage = value;
}

void ahp_gt_set_intensity(int axis, int value)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].axis[axis].intensity = value;
    optimize_values(axis);
}

void ahp_gt_add_intensity_deviator(int axis, gt_deviator deviator) {
    devices[ahp_gt_get_current_device()].axis[axis].deviators_n++;
    devices[ahp_gt_get_current_device()].axis[axis].deviators = (gt_deviator*)realloc(devices[ahp_gt_get_current_device()].axis[axis].deviators, devices[ahp_gt_get_current_device()].axis[axis].deviators_n * sizeof(gt_deviator));
    memcpy(&devices[ahp_gt_get_current_device()].axis[axis].deviators[devices[ahp_gt_get_current_device()].axis[axis].deviators_n-1], &deviator, sizeof(gt_deviator));
}

double ahp_gt_get_intensity_deviation(int axis, double freq) {
    int x;
    double y = 0;
    for (x = 0; x < devices[ahp_gt_get_current_device()].axis[axis].deviators_n; x ++) {
        y += devices[ahp_gt_get_current_device()].axis[axis].deviators[x].constant;
        y += pow(devices[ahp_gt_get_current_device()].axis[axis].deviators[x].variable, 2);
    }
    return devices[ahp_gt_get_current_device()].axis[axis].voltage/pow(y, 0.5);
}

int ahp_gt_get_current_device() {
    return ahp_gt_current_device;
}

int ahp_gt_detect_device(int *percent) {
    if(!ahp_gt_is_connected())
        return -1;
    int a = 0;
    if(percent == NULL)
        percent = (int*)malloc(sizeof(int));
    memset(devices, 0, 128*sizeof(gt_info));
    devices[ahp_gt_get_current_device()].detected = 0;
    devices[ahp_gt_get_current_device()].baud_rate = 9600;
    ahp_gt_set_axes_limit(NumAxes);
    int num_axes = ahp_gt_get_axes_limit();
    for (a = 0; a < num_axes; a++)
        memset(&devices[a].axis, 0, 128*sizeof(gt_axis));
    double progress = 0;
    *percent = 0;
    for (a = 0; a < num_axes; a++) {
        devices[ahp_gt_get_current_device()].axis [a].index = a;
        devices[ahp_gt_get_current_device()].axis[a].detected = 0;
        devices[ahp_gt_get_current_device()].axis[a].version = ahp_gt_get_mc_version(a);
        if(devices[ahp_gt_get_current_device()].axis[a].version > 0) {
            pgarb("MC Axis %d Version: %06X\n", a, devices[ahp_gt_get_current_device()].axis[a].version);
            devices[ahp_gt_get_current_device()].axis[a].index = a;
            devices[ahp_gt_get_current_device()].axis[a].detected = 1;
            devices[ahp_gt_get_current_device()].detected = 1;
            ahp_gt_read_values(a);
        }
        *percent = a * 100.0 / num_axes;
    }
    *percent = 0;
    if(devices[ahp_gt_get_current_device()].detected) {
        return 0;
    }
    return -1;
}

int ahp_gt_select_device(int address) {
    if(!ahp_gt_is_connected())
        return -1;
    address = fmax(0, fmin(address, 128));
    if(!dispatch_command(SetAddress, 0, address)) {
        ahp_gt_current_device = address;
        return 0;
    }
    return -1;
}

void ahp_gt_delete_device(int index)
{
    if(!ahp_gt_is_detected())
        return;
    memset(&devices[index], 0, sizeof(gt_info));
}

void ahp_gt_copy_device(int from, int to)
{
    if(!ahp_gt_is_detected())
        return;
    devices[from].index = to;
    memcpy(&devices[to], &devices[from], sizeof(gt_info));
    memcpy(&devices[to].axis, &devices[from].axis, sizeof(gt_axis)*NumAxes);
    int axis;
    for(axis = 0; axis < NumAxes; axis++)
        if(devices[to].axis[axis].detected) break;
    dispatch_command(FlashEnable, axis, -1);
    int oldvalue = dispatch_command(GetVars, 15, -1);
    oldvalue = (oldvalue >> 16) | (oldvalue << 16) | oldvalue & 0xff00;
    oldvalue &= ~0xfe00;
    oldvalue |= to << 9;
    oldvalue = (oldvalue >> 16) | (oldvalue << 16) | oldvalue & 0xff00;
    dispatch_command(FlashEnable, axis, -1);
    dispatch_command(SetVars, 15, oldvalue);
    dispatch_command(ReloadVars, axis, -1);
}

SkywatcherAxisStatus ahp_gt_get_status(int axis)
{
    if(!ahp_gt_is_detected())
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
    devices[ahp_gt_get_current_device()].axis[axis].axisstatus = status;
    return status;
}

void ahp_gt_set_position(int axis, double value)
{
    if(!ahp_gt_is_detected())
        return;
    dispatch_command(SetAxisPositionCmd, axis, (int)((value-M_PI_2)*devices[ahp_gt_get_current_device()].axis[axis].totalsteps/M_PI/2.0)+0x800000);
}

double ahp_gt_get_position(int axis, double *timestamp)
{
    if(!ahp_gt_is_detected())
        return 0;
    int steps = dispatch_command(GetAxisPosition, axis, -1) - 0x800000;
    if(errno == 0) devices[ahp_gt_get_current_device()].axis[axis].last_step = steps;
    if(timestamp != NULL)
        *timestamp = get_timestamp() - 0.008333333;
    return (double)devices[ahp_gt_get_current_device()].axis[axis].last_step*M_PI*2.0/(double)devices[ahp_gt_get_current_device()].axis[axis].totalsteps;
}

void ahp_gt_set_time(double tm)
{
    if(!ahp_gt_is_detected())
        return;
    double t = get_timestamp();
    devices[ahp_gt_get_current_device()].time_offset = tm - t;
}

double ahp_gt_get_time()
{
    if(!ahp_gt_is_detected())
        return 0.0;
    double t = get_timestamp();
    return devices[ahp_gt_get_current_device()].time_offset + t;
}

void ahp_gt_set_time_offset(double offset)
{
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].time_offset = offset;
}

double ahp_gt_get_time_offset()
{
    if(!ahp_gt_is_detected())
        return 0.0;
    return devices[ahp_gt_get_current_device()].time_offset;
}

void ahp_gt_set_location(double latitude, double longitude, double elevation)
{
    if(!ahp_gt_is_detected())
        return;
    longitude = range_360(longitude);
    latitude = range_dec(latitude);
    devices[ahp_gt_get_current_device()].lat = latitude;
    devices[ahp_gt_get_current_device()].lon = longitude;
    devices[ahp_gt_get_current_device()].el = elevation;
}

void ahp_gt_get_location(double *latitude, double *longitude, double *elevation)
{
    if(!ahp_gt_is_detected())
        return;
    *latitude = devices[ahp_gt_get_current_device()].lat;
    *longitude = devices[ahp_gt_get_current_device()].lon;
    *elevation = devices[ahp_gt_get_current_device()].el;
}

int ahp_gt_is_axis_moving(int axis)
{
    if(!ahp_gt_is_detected())
        return 0;
    return devices[ahp_gt_get_current_device()].axis[axis].axisstatus.Running;
}

void ahp_gt_goto_altaz(double alt, double az)
{
    if(!ahp_gt_is_detected())
        return;
    if(ahp_gt_is_axis_moving(0))
        return;
    if(ahp_gt_is_axis_moving(1))
        return;
    double ra, dec;
    ahp_gt_get_ra_dec_coordinates(alt, az, &ra, &dec);
    ahp_gt_goto_radec(ra, dec);
}

void ahp_gt_goto_radec(double ra, double dec)
{
    if(!ahp_gt_is_detected())
        return;
    if(ahp_gt_is_axis_moving(0))
        return;
    if(ahp_gt_is_axis_moving(1))
        return;
    double alpha, delta;
    radec2rad(ra, dec, &alpha, &delta, &devices[ahp_gt_get_current_device()].will_flip);
    ahp_gt_goto_absolute(0, alpha, 800.0);
    ahp_gt_goto_absolute(1, delta, 800.0);
}

void ahp_gt_sync_radec(double ra, double dec)
{
    if(!ahp_gt_is_detected())
        return;
    double alpha, delta;
    radec2rad(ra, dec, &alpha, &delta, &devices[ahp_gt_get_current_device()].flipped);
    ahp_gt_set_position(0, alpha);
    ahp_gt_set_position(1, delta);
}

void ahp_gt_goto_absolute(int axis, double target, double speed) {
    if(!ahp_gt_is_detected())
        return;
    if(ahp_gt_is_axis_moving(axis))
        return;
    double position = ahp_gt_get_position(axis, NULL);
    speed *= SIDEREAL_DAY / M_PI / 2;
    speed = fabs(speed);
    speed *= (target-position < 0 ? -1 : 1);
    double max = devices[ahp_gt_get_current_device()].axis[axis].totalsteps;
    target *= max;
    target /= M_PI*2;
    target += (double)0x800000;
    double maxperiod = SIDEREAL_DAY * devices[ahp_gt_get_current_device()].axis[axis].wormsteps / devices[ahp_gt_get_current_device()].axis[axis].totalsteps;
    int period = maxperiod * devices[ahp_gt_get_current_device()].axis[axis].multiplier;
    SkywatcherMotionMode mode = MODE_GOTO_HISPEED;
    if(fabs(speed) < 128.0) {
        mode = MODE_GOTO_LOSPEED;
        period /= devices[ahp_gt_get_current_device()].axis[axis].multiplier;
    }
    mode |= (speed < 0 ? 1 : 0);
    period /= fabs(speed);
    period = fmax(devices[ahp_gt_get_current_device()].axis[axis].minperiod, period);
    devices[ahp_gt_get_current_device()].axis[axis].motionmode = mode;
    dispatch_command (Initialize, axis, -1);
    dispatch_command (ActivateMotor, axis, -1);
    dispatch_command (SetGotoTarget, axis, target);
    dispatch_command (SetStepPeriod, axis, period);
    dispatch_command (SetMotionMode, axis, mode);
    dispatch_command (StartMotion, axis, -1);
}

void ahp_gt_goto_relative(int axis, double increment, double speed) {
    if(!ahp_gt_is_detected())
        return;
    if(ahp_gt_is_axis_moving(axis))
        return;
    speed *= SIDEREAL_DAY / M_PI / 2;
    speed = fabs(speed);
    speed *= (increment < 0 ? -1 : 1);
    double max = devices[ahp_gt_get_current_device()].axis[axis].totalsteps;
    increment /= M_PI*2;
    increment *= max;
    double maxperiod = SIDEREAL_DAY * devices[ahp_gt_get_current_device()].axis[axis].wormsteps / devices[ahp_gt_get_current_device()].axis[axis].totalsteps;
    int period = maxperiod * devices[ahp_gt_get_current_device()].axis[axis].multiplier;
    SkywatcherMotionMode mode = MODE_GOTO_HISPEED;
    if(fabs(speed) < 128.0) {
        mode = MODE_GOTO_LOSPEED;
        period /= devices[ahp_gt_get_current_device()].axis[axis].multiplier;
    }
    mode |= (speed < 0 ? 1 : 0);
    period /= fabs(speed);
    devices[ahp_gt_get_current_device()].axis[axis].motionmode = mode;
    dispatch_command (Initialize, axis, -1);
    dispatch_command (ActivateMotor, axis, -1);
    dispatch_command (SetGotoTargetIncrement, axis, (int)fabs(increment));
    dispatch_command (SetStepPeriod, axis, period);
    dispatch_command (SetMotionMode, axis, mode);
    dispatch_command (StartMotion, axis, -1);
}

void ahp_gt_start_motion(int axis, double speed) {
    if(!ahp_gt_is_detected())
        return;
    if(ahp_gt_is_axis_moving(axis))
        return;
    speed *= SIDEREAL_DAY / M_PI / 2;
    double period = SIDEREAL_DAY * devices[ahp_gt_get_current_device()].axis[axis].multiplier * devices[ahp_gt_get_current_device()].axis[axis].wormsteps / devices[ahp_gt_get_current_device()].axis[axis].totalsteps;
    SkywatcherMotionMode mode = MODE_SLEW_HISPEED;
    if(fabs(speed) < 128.0) {
        mode = MODE_SLEW_LOSPEED;
        period /= devices[ahp_gt_get_current_device()].axis[axis].multiplier;
    }
    mode |= (speed < 0 ? 1 : 0);
    period /= fabs(speed);
    devices[ahp_gt_get_current_device()].axis[axis].motionmode = mode;
    dispatch_command (Initialize, axis, -1);
    dispatch_command (ActivateMotor, axis, -1);
    dispatch_command (SetStepPeriod, axis, period);
    dispatch_command (SetMotionMode, axis, mode);
    dispatch_command (StartMotion, axis, -1);
}

void ahp_gt_stop_motion(int axis, int wait) {
    if(!ahp_gt_is_detected())
        return;
    dispatch_command(InstantAxisStop, axis, -1);
    if(wait) {
        while (ahp_gt_is_axis_moving(axis))
            usleep(100);
    }
}

void ahp_gt_start_tracking_thread() {
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].threads_running = 1;
    pthread_create(&tracking_thread, NULL, (void*)&devices[ahp_gt_get_current_device()], track);
}

void ahp_gt_stop_tracking_thread() {
    if(!ahp_gt_is_detected())
        return;
    if(!devices[ahp_gt_get_current_device()].threads_running)
        return;
    devices[ahp_gt_get_current_device()].threads_running = 0;
    pthread_join(tracking_thread, NULL);
}

void ahp_gt_set_tracking_mode(int mode) {
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].tracking_mode = mode;
}

int ahp_gt_get_tracking_mode() {
    if(!ahp_gt_is_detected())
        return 0;
    return devices[ahp_gt_get_current_device()].tracking_mode;
}

void ahp_gt_start_tracking(int axis) {
    if(!ahp_gt_is_detected())
        return;
    double period = SIDEREAL_DAY * devices[ahp_gt_get_current_device()].axis[axis].wormsteps / devices[ahp_gt_get_current_device()].axis[axis].totalsteps;
    dispatch_command (SetStepPeriod, axis, period);
    dispatch_command (Initialize, axis, -1);
    dispatch_command (ActivateMotor, axis, -1);
    dispatch_command (SetMotionMode, axis, 0x10);
    dispatch_command (StartMotion, axis, -1);
}

void ahp_gt_correct_tracking(int axis, double target_period, int *interrupt) {
    if(!ahp_gt_is_detected())
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
    one_second = initial_second - initial_second*(one_second-1.0);
    ahp_gt_set_timing(axis, one_second);
    ahp_gt_write_and_verify (axis, axis * 8 + 4, ahp_gt_get_timing(axis));
    ahp_gt_reload(axis);
}

void ahp_gt_set_aligned(int aligned) {
    if(!ahp_gt_is_detected())
        return;
    devices[ahp_gt_get_current_device()].is_aligned = aligned;
}

int ahp_gt_is_aligned() {
    if(!ahp_gt_is_detected())
        return 0;
    return devices[ahp_gt_get_current_device()].is_aligned;
}

GT_Model ahp_gt_get_axis_model(int axis) {
    return devices[ahp_gt_get_current_device()].axis[axis].model;
}

const char* ahp_gt_get_axis_name(int axis) {
    return axes[axis];
}

const char** ahp_gt_get_axes_names() {
    return (const char**)axes;
}
