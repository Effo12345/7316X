/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * Copyright (c) 2017-2021, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter, more convenient naming pattern. If this isn't desired, simply
 * comment the following line out.
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
 * not convienent for most student programmers.
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS

#include "api.h"

#define PI 3.14159265359

//Class to hold any necessary timing values
class timer {
  int prevTime = pros::millis();
public:
  int time_abs;
  int time_relative;
  int time(){
    time_abs = pros::millis();
    time_relative = time_abs - prevTime;
    return time_relative;
  }
  void reset() {
    prevTime = pros::millis();
  }
};

//Rate limiter used in pure pursuit algorithm
class rateLimiter {
  timer clock;        //Timer to get time since last call
  double output;      //Output value (must persist between function calls)
public:
  double constrain(double input, double maxRateChange) {
    double maxChange = clock.time() * maxRateChange;
    double temp = input - output;

    if(temp < -maxChange)
      temp = -maxChange;
    else if(temp > maxChange)
      temp = maxChange;

    clock.reset();
    output += temp;
    return output;
  }
};

extern pros::Motor driveFL;
extern pros::Motor driveML;
extern pros::Motor driveBL;
extern pros::Motor driveFR;
extern pros::Motor driveMR;
extern pros::Motor driveBR;
extern pros::Motor lift;
extern pros::Motor intake;


extern pros::ADIEncoder tracking;
extern pros::Rotation leftEncoder;
extern pros::Rotation rightEncoder;
extern pros::Imu gyro;

extern pros::ADIDigitalOut frontClip;
extern pros::ADIDigitalOut backClip;
extern pros::ADIDigitalOut clipGuard;

extern pros::Controller master;

//Misc functions
void ResetSensors(bool calibrateGyro);

//Enumerated values to increase the readability of various functions
enum intakeState {off, on};


/**
 * You should add more #includes here
 */
//#include "okapi/api.hpp"
//#include "pros/api_legacy.h"

/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace pros::literals;
// using namespace okapi;

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
#include <iostream>
#include <array>
#include <optional>
#endif

#endif  // _PROS_MAIN_H_
