// -------------------------------------------------------------------------
// CV Drone (= OpenCV + AR.Drone)
// Copyright(C) 2014 puku0x
// https://github.com/puku0x/cvdrone
//
// This source file is part of CV Drone library.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of EITHER:
// (1) The GNU Lesser General Public License as published by the Free
//     Software Foundation; either version 2.1 of the License, or (at
//     your option) any later version. The text of the GNU Lesser
//     General Public License is included with this library in the
//     file cvdrone-license-LGPL.txt.
// (2) The BSD-style license that is included with this library in
//     the file cvdrone-license-BSD.txt.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
// cvdrone-license-LGPL.txt and cvdrone-license-BSD.txt for more details.
//
//! @file   command.cpp
//! @brief  Sending AT commands
//
// -------------------------------------------------------------------------

#include "ardrone.h"

// From Parrot SDK, undocumented
#define DEFAULT_MISC1_VALUE 2
#define DEFAULT_MISC2_VALUE 20
#define DEFAULT_MISC3_VALUE 2000
#define DEFAULT_MISC4_VALUE 3000

float maxEulerAngle = 9.0f * DEG_TO_RAD; //12
float maxYaw = 30.0f * DEG_TO_RAD; //99
int maxAltitude = 1500;
int maxVerticalSpeed = 500;

//typedef union _float_or_int_t {
//  float32_t f;
//  int32_t   i;
//} float_or_int_t;

// --------------------------------------------------------------------------
//! @brief   Initialize AT command.
//! @return  Result of initialization
//! @retval  1 Success
//! @retval  0 Failure
// --------------------------------------------------------------------------
int ARDrone::initCommand(void)
{
    // BC - force locale to ensure proper decimal separator "." instead of "," using sprintf etc
    setlocale(LC_NUMERIC, "POSIX");

    // Open the IP address and port
    if (!sockCommand.open(ip, ARDRONE_AT_PORT)) {
        CVDRONE_ERROR("UDPSocket::open(port=%d) failed. (%s, %d)\n", ARDRONE_AT_PORT, __FILE__, __LINE__);
        return 0;
    }

    // AR.Drone 2.0
    if (version.major == ARDRONE_VERSION_2) {
        // Send undocumented command
        sockCommand.sendf("AT*PMODE=%d,%d\r", ++seq, DEFAULT_MISC1_VALUE);

        // Send undocumented command
        sockCommand.sendf("AT*MISC=%d,%d,%d,%d,%d\r", ++seq, DEFAULT_MISC1_VALUE, DEFAULT_MISC2_VALUE, DEFAULT_MISC3_VALUE, DEFAULT_MISC4_VALUE);

        // Send flat trim
        sockCommand.sendf("AT*FTRIM=%d,\r", ++seq);
        msleep(100);

        // Set the configuration IDs
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", ++seq, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"custom:session_id\",\"%s\"\r", ++seq, ARDRONE_SESSION_ID);
        msleep(100);
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", ++seq, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"custom:profile_id\",\"%s\"\r", ++seq, ARDRONE_PROFILE_ID);
        msleep(100);
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", ++seq, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"custom:application_id\",\"%s\"\r", ++seq, ARDRONE_APPLOCATION_ID);
        msleep(100);

        // Set maximum velocity in Z-axis [mm/s]
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", ++seq, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"control:control_vz_max\",\"%d\"\r", ++seq, maxVerticalSpeed);
        msleep(30);

        // Set maximum yaw [rad/s]
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", ++seq, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"control:control_yaw\",\"%f\"\r", ++seq, maxYaw);
        msleep(30);

        // Set maximum euler angle [rad]
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", ++seq, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"control:euler_angle_max\",\"%f\"\r", ++seq, maxEulerAngle);
        msleep(50);

        // Set maximum altitude [mm]
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", ++seq, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"control:altitude_max\",\"%d\"\r", ++seq, maxAltitude);
        msleep(30);

        // Bitrate control mode
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", ++seq, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"video:bitrate_ctrl_mode\",\"%d\"\r", ++seq, 0);     // VBC_MODE_DISABLED
        //sockCommand.sendf("AT*CONFIG=%d,\"video:bitrate_ctrl_mode\",\"%d\"\r", ++seq, 1);   // VBC_MODE_DYNAMIC
        //sockCommand.sendf("AT*CONFIG=%d,\"video:bitrate_ctrl_mode\",\"%d\"\r", ++seq, 2);   // VBC_MANUAL
        msleep(30);

        // Bitrate
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", ++seq, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"video:bitrate\",\"%d\"\r", ++seq, 1000);
        msleep(30);

        // Max bitrate
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", ++seq, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"video:max_bitrate\",\"%d\"\r", ++seq, 4000);
        msleep(30);

        // Set video codec
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", ++seq, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        //sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", ++seq, 0x81);   // H264_360P_CODEC
        //sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", ++seq, 0x82); // MP4_360P_H264_720P_CODEC
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", ++seq, 0x83); // H264_720P_CODEC
        //sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", ++seq, 0x88); // MP4_360P_H264_360P_CODEC
        msleep(50);

        // Set video channel to default
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", ++seq, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_channel\",\"0\"\r", ++seq);
        msleep(30);

        // Disable USB recording
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", ++seq, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_on_usb\",\"FALSE\"\r", ++seq);
        msleep(30);
    }

    // Disable outdoor mode
    setOutdoorMode(false);

    // Create a mutex
    mutexCommand = new pthread_mutex_t;
    pthread_mutex_init(mutexCommand, NULL);

    // Create a thread
    threadCommand = new pthread_t;
    if (pthread_create(threadCommand, NULL, runCommand, this) != 0) {
        CVDRONE_ERROR("pthread_create() was failed. (%s, %d)\n", __FILE__, __LINE__);
        return 0;
    }

    return 1;
}

// --------------------------------------------------------------------------
//! @brief   Thread function for AT command.
//! @return  None
// --------------------------------------------------------------------------
void ARDrone::loopCommand(void)
{
    unsigned long cnt = 0;
    while (1) {
        if(cnt % 5 == 0){
            // Reset Watch-Dog every 100ms
            if (mutexCommand) pthread_mutex_lock(mutexCommand);
            sockCommand.sendf("AT*COMWDG=%d\r", ++seq);
            if (mutexCommand) pthread_mutex_unlock(mutexCommand);
        }

        //continously move the drone with stored velocity - BC
        move3D(controlVelocity.roll, controlVelocity.pitch, controlVelocity.gaz, controlVelocity.yaw);

        pthread_testcancel();

        cnt++;
        msleep(20); //refresh time from Parrot SDK
    }
}

// --------------------------------------------------------------------------
//! @brief   Take off the AR.Drone.
//! @return  None
// --------------------------------------------------------------------------
void ARDrone::takeoff(void)
{
    // Get the state
    if (mutexCommand) pthread_mutex_lock(mutexNavdata);
    int state = navdata.ardrone_state;
    if (mutexCommand) pthread_mutex_unlock(mutexNavdata);

    // If AR.Drone is in emergency, reset it
    if (state & ARDRONE_EMERGENCY_MASK) emergency();
    else {
        // Send take off
        if (mutexCommand) pthread_mutex_lock(mutexCommand);
        sockCommand.sendf("AT*REF=%d,290718208\r", ++seq);
        if (mutexCommand) pthread_mutex_unlock(mutexCommand);
    }
}

// --------------------------------------------------------------------------
//! @brief   Land the AR.Drone.
//! @return  None
// --------------------------------------------------------------------------
void ARDrone::landing(void)
{
    // Get the state
    if (mutexCommand) pthread_mutex_lock(mutexNavdata);
    int state = navdata.ardrone_state;
    if (mutexCommand) pthread_mutex_unlock(mutexNavdata);

    // If AR.Drone is in emergency, reset it
    if (state & ARDRONE_EMERGENCY_MASK) emergency();
    else {
        // Send langding
        if (mutexCommand) pthread_mutex_lock(mutexCommand);
        sockCommand.sendf("AT*REF=%d,290717696\r", ++seq);
        if (mutexCommand) pthread_mutex_unlock(mutexCommand);
    }
}

// --------------------------------------------------------------------------
//! @brief   Emergency stop.
//! @return  None
// --------------------------------------------------------------------------
void ARDrone::emergency(void)
{
    // Send emergency
    if (mutexCommand) pthread_mutex_lock(mutexCommand);
    sockCommand.sendf("AT*REF=%d,290717952\r", ++seq);
    if (mutexCommand) pthread_mutex_unlock(mutexCommand);
}

void ARDrone::setVelocity(ControlCommand& c) {
    controlVelocity = c;
}

// --------------------------------------------------------------------------
//! @brief   Move the AR.Drone in 2D plane.
//! @param   vx X velocity [m/s]
//! @param   vy Y velocity [m/s]
//! @param   vr Angular velocity [rad/s]
//! @return  None
// --------------------------------------------------------------------------
void ARDrone::move(double vx, double vy, double vr)
{
    move3D(vx, vy, 0.0, vr);
}

// --------------------------------------------------------------------------
//! @brief   Move the AR.Drone in 3D space.
//! @param   vx X velocity [m/s]
//! @param   vy Y velocity [m/s]
//! @param   vz Z velocity [m/s]
//! @param   vr Angular velocity [rad/s]
//! @return  None
// --------------------------------------------------------------------------
#define _EPS 10e-2
void ARDrone::move3D(double vx, double vy, double vz, double vr)
{
    // AR.Drone is flying
    if (!onGround()) {
        // Command velocities
        // roll - pitch - gaz - yaw
        float v[4] = {1.0f * (float)vx, -1.0f * (float)vy, 1.0f * (float)vz, 1.0f * (float)vr};
        //int mode = (fabs(v[0]) > 0.0 || fabs(v[1]) > 0.0);

        // These lines are for testing, they should be moved to configurations
        // Bit 0 of control_flag == 0: should we hover?
        // Bit 1 of control_flag == 1: should we use combined yaw mode?

        int32_t control_flag = 0x00;
        int32_t combined_yaw = 0x00;

        // Auto hover detection based on ~0 values for 4DOF cmd_vel
        int32_t hover = (int32_t)
                (
                (fabs(v[0]) < _EPS) &&
                (fabs(v[1]) < _EPS) &&
                (fabs(v[2]) < _EPS) &&
                (fabs(v[3]) < _EPS)
                );

        control_flag |= ((1 - hover) << 0);
        control_flag |= (combined_yaw << 1);
        //int mode = 1;

        // Nomarization (-1.0 to +1.0)
        for (int i = 0; i < 4; i++) {
            if (fabs(v[i]) > 1.0) v[i] /= fabs(v[i]);
        }

        // Send a command
        if (mutexCommand) pthread_mutex_lock(mutexCommand);
        sockCommand.sendf("AT*PCMD_MAG=%d,%d,%d,%d,%d,%d,%d,%d\r", ++seq, control_flag, *(int*)(&v[0]), *(int*)(&v[1]), *(int*)(&v[2]), *(int*)(&v[3]), 0, 0);
        if (mutexCommand) pthread_mutex_unlock(mutexCommand);
    }
}

// --------------------------------------------------------------------------
//! @brief   Change the camera channel.
//! @param   channel Camera channel 
//! @note    AR.Drone 1.0 supports [0, 1, 2, 3]. 
//!          AR.Drone 2.0 supports [0, 1].
//! @return  None
// --------------------------------------------------------------------------
void ARDrone::setCamera(int channel)
{
    // Enable mutex lock
    if (mutexCommand) pthread_mutex_lock(mutexCommand);

    // AR.Drone 2.0
    if (version.major == ARDRONE_VERSION_2) {
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", ++seq, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_channel\",\"%d\"\r", ++seq, channel % 2);
    }
    // AR.Drone 1.0
    else {
        sockCommand.sendf("AT*CONFIG=%d,\"video:video_channel\",\"%d\"\r", ++seq, channel % 4);
    }

    // Disable mutex lock
    if (mutexCommand) pthread_mutex_unlock(mutexCommand);

    msleep(100);
}

// --------------------------------------------------------------------------
//! @brief   Set a reference of the horizontal plane.
//! @return  None
// --------------------------------------------------------------------------
void ARDrone::setFlatTrim(void)
{
    if (onGround()) {
        // Send flat trim command
        if (mutexCommand) pthread_mutex_lock(mutexCommand);
        sockCommand.sendf("AT*FTRIM=%d\r", ++seq);
        if (mutexCommand) pthread_mutex_unlock(mutexCommand);
    }
}

// --------------------------------------------------------------------------
//! @brief   Calibrate AR.Drone's magnetometer.
//! @param   device Device ID
//! @return  None
// --------------------------------------------------------------------------
void ARDrone::setCalibration(int device)
{
    if (!onGround()) {
        // Send calibration command
        if (mutexCommand) pthread_mutex_lock(mutexCommand);
        sockCommand.sendf("AT*CALIB=%d,%d\r", ++seq, device);
        if (mutexCommand) pthread_mutex_unlock(mutexCommand);
    }
}

// --------------------------------------------------------------------------
//! @brief   Run specified flight animation.
//! @param   id Flight animation ID
//! @param   timeout Timeout [ms]
//! @return  None
// --------------------------------------------------------------------------
void ARDrone::setAnimation(int id, int timeout)
{
    // ID
    if (version.major == ARDRONE_VERSION_2) id = abs(id % ARDRONE_NB_ANIM_MAYDAY);
    else                                    id = abs(id % ARDRONE_ANIM_FLIP_AHEAD);

    // Default timeout
    if (timeout < 1) {
        const int default_timeout[ARDRONE_NB_ANIM_MAYDAY] = {1000, 1000, 1000, 1000, 1000, 1000, 5000, 5000, 2000, 5000, 5000, 5000, 5000, 5000, 5000, 5000, 15, 15, 15, 15};
        timeout = default_timeout[id];
    }

    // Send a command
    if (mutexCommand) pthread_mutex_lock(mutexCommand);
    sockCommand.sendf("AT*ANIM=%d,%d,%d\r", ++seq, id, timeout);
    if (mutexCommand) pthread_mutex_unlock(mutexCommand);
}

// --------------------------------------------------------------------------
//! @brief   Run specified LED animation.
//! @param   id LED animation ID
//! @param   freq Frequency [Hz],
//! @param   duration Duration [s]
//! @return  None
// --------------------------------------------------------------------------
void ARDrone::setLED(int id, float freq, int duration)
{
    // ID
    id = abs(id % ARDRONE_NB_LED_ANIM_MAYDAY);

    // Default frequency
    if (freq <= 0.0) {
        float default_freq[ARDRONE_NB_LED_ANIM_MAYDAY] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        freq = default_freq[id];
    }

    // Default frequency
    if (freq <= 0.0) {
        int default_duration[ARDRONE_NB_LED_ANIM_MAYDAY] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
        duration = default_duration[id];
    }

    // Send a command
    if (mutexCommand) pthread_mutex_lock(mutexCommand);
    sockCommand.sendf("AT*LED=%d,%d,%d,%d\r", ++seq, id, *(int*)(&freq), duration);
    if (mutexCommand) pthread_mutex_unlock(mutexCommand);
}

// --------------------------------------------------------------------------
//! @brief   Start or stop recording video.
//! @param   activate Enable / Disable flag
//! @note    This function is only for AR.Drone 2.0. 
//!          You should set a USB key with > 100MB to your drone
//! @return  None
// --------------------------------------------------------------------------
void ARDrone::setVideoRecord(bool activate)
{
    // AR.Drone 2.0
    if (version.major == ARDRONE_VERSION_2) {
        // Finalize video
        finalizeVideo();

        // Enable/Disable video recording
        if (mutexCommand) pthread_mutex_lock(mutexCommand);
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", ++seq, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        if (activate) sockCommand.sendf("AT*CONFIG=%d,\"video:video_on_usb\",\"TRUE\"\r",  ++seq);
        else          sockCommand.sendf("AT*CONFIG=%d,\"video:video_on_usb\",\"FALSE\"\r", ++seq);
        if (mutexCommand) pthread_mutex_unlock(mutexCommand);
        msleep(100);

        // Output video with MP4_360P_H264_720P_CODEC / H264_360P_CODEC
        if (mutexCommand) pthread_mutex_lock(mutexCommand);
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", ++seq, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        if (activate) sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", ++seq, 0x82);
        else          sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", ++seq, 0x81);
        if (mutexCommand) pthread_mutex_unlock(mutexCommand);
        msleep(100);

        // Initialize video
        initVideo();
    }
}

// --------------------------------------------------------------------------
//! @brief   Set outdoor mode.
//! @param   activate Enable / Disable flag
//! @return  None
// --------------------------------------------------------------------------
void ARDrone::setOutdoorMode(bool activate)
{
    // AR.Drone 2.0
    if (version.major == ARDRONE_VERSION_2) {
        // Enable/Disable outdoor mode
        if (mutexCommand) pthread_mutex_lock(mutexCommand);
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", ++seq, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        if (activate) sockCommand.sendf("AT*CONFIG=%d,\"control:outdoor\",\"TRUE\"\r",  ++seq);
        else          sockCommand.sendf("AT*CONFIG=%d,\"control:outdoor\",\"FALSE\"\r", ++seq);
        if (mutexCommand) pthread_mutex_unlock(mutexCommand);
        msleep(100);

        // Without/With shell
        if (mutexCommand) pthread_mutex_lock(mutexCommand);
        sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", ++seq, ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
        if (activate) sockCommand.sendf("AT*CONFIG=%d,\"control:flight_without_shell\",\"TRUE\"\r",  ++seq);
        else          sockCommand.sendf("AT*CONFIG=%d,\"control:flight_without_shell\",\"FALSE\"\r", ++seq);
        if (mutexCommand) pthread_mutex_unlock(mutexCommand);
        msleep(100);
    }
}

// --------------------------------------------------------------------------
//! @brief   Stop hovering.
//! @return  None
// --------------------------------------------------------------------------
void ARDrone::resetWatchDog(void)
{
    // Get the state
    if (mutexCommand) pthread_mutex_lock(mutexNavdata);
    int state = navdata.ardrone_state;
    if (mutexCommand) pthread_mutex_unlock(mutexNavdata);

    // If AR.Drone is in Watch-Dog, reset it
    if (state & ARDRONE_COM_WATCHDOG_MASK) {
        if (mutexCommand) pthread_mutex_lock(mutexCommand);
        sockCommand.sendf("AT*COMWDG=%d\r", ++seq);
        if (mutexCommand) pthread_mutex_unlock(mutexCommand);
    }
}

// --------------------------------------------------------------------------
//! @brief   Disable the emergency lock.
//! @return  None
// --------------------------------------------------------------------------
void ARDrone::resetEmergency(void)
{
    // Get the state
    if (mutexCommand) pthread_mutex_lock(mutexNavdata);
    int state = navdata.ardrone_state;
    if (mutexCommand) pthread_mutex_unlock(mutexNavdata);

    // If AR.Drone is in emergency, reset it
    if (state & ARDRONE_EMERGENCY_MASK) {
        if (mutexCommand) pthread_mutex_lock(mutexCommand);
        sockCommand.sendf("AT*REF=%d,290717952\r", ++seq);
        if (mutexCommand) pthread_mutex_unlock(mutexCommand);
    }
}

// --------------------------------------------------------------------------
//! @brief  Finalize AT command.
//! @return  None
// --------------------------------------------------------------------------
void ARDrone::finalizeCommand(void)
{
    // Destroy the thread
    if (threadCommand) {
        pthread_cancel(*threadCommand);
        pthread_join(*threadCommand, NULL);
        delete threadCommand;
        threadCommand = NULL;
    }

    // Delete the mutex
    if (mutexCommand) {
        pthread_mutex_destroy(mutexCommand);
        delete mutexCommand;
        mutexCommand = NULL;
    }

    // Close the socket
    sockCommand.close();
}
