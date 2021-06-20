/*
    libahp_xc library to drive the AHP XC correlators
    Copyright (C) 2020  Ilia Platone

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef _AHP_GT_H
#define _AHP_GT_H

#ifdef  __cplusplus
extern "C" {
#endif
#ifdef _WIN32
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT extern
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

///GT1 coil configuration
typedef enum {
    AABB             = 0,
    ABAB             = 1,
    ABBA             = 2,
} GT1Stepping;

///GT1Feature AHP GT default features
typedef enum  {
    GpioUnused             = 0x0000,
    GpioAsST4              = 0x0001,
    LowCurrent             = 0x0100,
    LowCurrentAndGpioAsST4 = 0x0101,
} GT1Feature;

///SkywatcherFeature Skywatcher default features
typedef enum {
    inPPECTraining         = 0x000010,
    inPPEC                 = 0x000020,
    hasEncoder             = 0x000001,
    hasPPEC                = 0x000002,
    hasHomeIndexer         = 0x000004,
    isAZEQ                 = 0x000008,
    hasPolarLed            = 0x001000,
    hasCommonSlewStart     = 0x002000, // supports :J3
    hasHalfCurrentTracking = 0x004000,
    hasWifi                = 0x008000,
} SkywatcherFeature;

///MountType Default Mount type
typedef enum {
    isEQ6 = 0x00,
    isHEQ5 = 0x01,
    isEQ5 = 0x02,
    isEQ3 = 0x03,
    isEQ8 = 0x04,
    isAZEQ6 = 0x05,
    isAZEQ5 = 0x06,
    isGT = 0x80,
    isMF = 0x81,
    is114GT = 0x82,
    isDOB = 0x90,
    isCustom = 0xF0,
} MountType;

///SkywatcherCommand Taken from INDIlib
typedef enum {
    Null                      = '\0',
    Initialize                = 'F',
    InquireMotorBoardVersion  = 'e',
    InquireGridPerRevolution  = 'a',
    InquireTimerInterruptFreq = 'b',
    InquireHighSpeedRatio     = 'g',
    InquirePECPeriod          = 's',
    InstantAxisStop           = 'L',
    NotInstantAxisStop        = 'K',
    SetAxisPositionCmd        = 'E',
    GetAxisPosition           = 'j',
    GetAxisStatus             = 'f',
    SetSwitch                 = 'O',
    SetMotionMode             = 'G',
    SetGotoTargetIncrement    = 'H',
    SetBreakPointIncrement    = 'M',
    SetGotoTarget             = 'S',
    SetBreakStep              = 'U',
    SetStepPeriod             = 'I',
    StartMotion               = 'J',
    GetStepPeriod             = 'D', // See Merlin protocol http://www.papywizard.org/wiki/DevelopGuide
    ActivateMotor             = 'B', // See eq6direct implementation http://pierre.nerzic.free.fr/INDI/
    SetST4GuideRateCmd        = 'P',
    SetFeatureCmd             = 'W', // EQ8/AZEQ6/AZEQ5 only
    GetFeatureCmd             = 'q', // EQ8/AZEQ6/AZEQ5 only
    InquireAuxEncoder         = 'd', // EQ8/AZEQ6/AZEQ5 only
    SetVars                   = '@',
    GetVars                   = '?',
    ReloadVars                = '$',
    Flash                     = '#',
    FlashEnable               = '!',
} SkywatcherCommand;

///AHP_GT_VERSION This library version
#define AHP_GT_VERSION @AHP_GT_VERSION@

/**
* \brief Write values from the GT controller
*/
DLL_EXPORT void ahp_gt_write_values(int axis, int *percent, int *finished);

/**
* \brief Read values from the GT controller
*/
DLL_EXPORT void ahp_gt_read_values(int axis);

/**
* \brief Connect to the GT controller
*/
DLL_EXPORT int ahp_gt_connect(const char* port);

/**
* \brief Get the GT firmware version
*/
DLL_EXPORT int ahp_gt_get_mc_version();

/**
* \brief Get the current GT features
*/
DLL_EXPORT MountType ahp_gt_get_mount_type();

/**
* \brief Get the current GT features
*/
DLL_EXPORT GT1Feature ahp_gt_get_feature(int axis);

/**
* \brief Get the current SkyWatcher features
*/
DLL_EXPORT SkywatcherFeature ahp_gt_get_features(int axis);

/**
* \brief Get the current motor steps number
*/
DLL_EXPORT double ahp_gt_get_motor_steps(int axis);

/**
* \brief Get the current motor gear teeth number
*/
DLL_EXPORT double ahp_gt_get_motor_teeth(int axis);

/**
* \brief Get the current worm gear teeth number
*/
DLL_EXPORT double ahp_gt_get_worm_teeth(int axis);

/**
* \brief Get the current crown gear teeth number
*/
DLL_EXPORT double ahp_gt_get_crown_teeth(int axis);

/**
* \brief Get the microsteps in the current configuration
*/
DLL_EXPORT double ahp_gt_get_microsteps(int axis);

/**
* \brief Get the guiding speed
*/
DLL_EXPORT double ahp_gt_get_guide_steps(int axis);

/**
* \brief Get the acceleration increment steps number
*/
DLL_EXPORT double ahp_gt_get_acceleration_steps(int axis);

/**
* \brief Get the acceleration
*/
DLL_EXPORT double ahp_gt_get_acceleration(int axis);

/**
* \brief Get the forward direction
*/
DLL_EXPORT int ahp_gt_get_direction_invert(int axis);

/**
* \brief Get the stepping configuration
*/
DLL_EXPORT GT1Stepping ahp_gt_get_stepping_conf(int axis);

/**
* \brief Get the maximum speed
*/
DLL_EXPORT double ahp_gt_get_max_speed(int axis);

/**
* \brief Get the speed limit
*/
DLL_EXPORT double ahp_gt_get_speed_limit(int axis);

/**
* \brief Set the mount type
*/
DLL_EXPORT void ahp_gt_set_mount_type(MountType value);

/**
* \brief Get the GT features
*/
DLL_EXPORT void ahp_gt_set_features(int axis, SkywatcherFeature value);

/**
* \brief Set the motor steps number
*/
DLL_EXPORT void ahp_gt_set_motor_steps(int axis, double value);

/**
* \brief Set the motor gear teeth number
*/
DLL_EXPORT void ahp_gt_set_motor_teeth(int axis, double value);

/**
* \brief Set the worm gear teeth number
*/
DLL_EXPORT void ahp_gt_set_worm_teeth(int axis, double value);

/**
* \brief Set the crown gear teeth number
*/
DLL_EXPORT void ahp_gt_set_crown_teeth(int axis, double value);

/**
* \brief Set the guiding speed
*/
DLL_EXPORT void ahp_gt_set_guide_steps(int axis, double value);

/**
* \brief Set the number of acceleration increments
*/
DLL_EXPORT void ahp_gt_set_acceleration_steps(int axis, double value);

/**
* \brief Set the acceleration in high speed mode
*/
DLL_EXPORT void ahp_gt_set_acceleration(int axis, double value);

/**
* \brief Set the forward direction
*/
DLL_EXPORT void ahp_gt_set_direction_invert(int axis, int value);

/**
* \brief Set the stepping configuration
*/
DLL_EXPORT void ahp_gt_set_stepping_conf(int axis, GT1Stepping value);

/**
* \brief Set the maximum goto speed
*/
DLL_EXPORT void ahp_gt_set_max_speed(int axis, double value);

/**
* \brief Start a slew motion
*/
DLL_EXPORT void ahp_gt_start_motion(int axis, double speed);

/**
* \brief Stop an axis motion
*/
DLL_EXPORT void ahp_gt_stop_motion(int axis);

/**
* \brief Set the slew speed
*/
DLL_EXPORT void ahp_gt_set_axis_speed(int axis, double speed);

/**
* \brief Start a test tracking motion
*/
DLL_EXPORT void ahp_gt_start_tracking(int axis);

/**
* \brief Obtain the current libahp-gt version
*/
DLL_EXPORT inline unsigned int ahp_gt_get_version(void) { return AHP_GT_VERSION; }

#ifdef __cplusplus
} // extern "C"
#endif

#endif //_AHP_GT_H
