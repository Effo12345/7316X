#include "main.h"
#include "units.hpp"

//Multithreading protection
pros::Mutex threadProtector;

//Variables for experimented PIDTurn
const float Ki_active_t = 10;
const float Ki_limit_t  = 100000;


//General drivetrain functions
void drive_voltage(int left, int right) {
  driveFL.move_voltage(left);
  driveML.move_voltage(left);
  driveBL.move_voltage(left);

  driveFR.move_voltage(right);
  driveMR.move_voltage(right);
  driveBR.move_voltage(right);
}

void drive_velocity(int left, int right) {
  driveFL.move_velocity(left);
  driveML.move_velocity(left);
  driveBL.move_velocity(left);

  driveFR.move_velocity(right);
  driveMR.move_velocity(right);
  driveBR.move_velocity(right);
}


//Op control specific drivetrain function
void drive_op(int left, int right) {
  driveFL.move(left);
  driveML.move(left);
  driveBL.move(left);

  driveFR.move(right);
  driveMR.move(right);
  driveBR.move(right);
}


//PID-based function used when grabbing neutral mobile goals
//Optimized to be as fast as possible
void hyperGrab(double setpoint)
{
  timer clock; 

  drive_voltage(12000, 12000);
  while((setpoint - Units.DegToIn(tracking.get_value())) > 5 && clock.time() < 2000)
    pros::delay(25);


  double kP = 7.0;
  double kI = 0.0;
  double kD = 3.0;

  float error = 6;
  float prevError;


  setpoint = Units.InToDeg(setpoint);

  while(error > 5 /*true std::abs(error) > 5*/ && clock.time() < 2000)
  {
    error = setpoint - tracking.get_value();
    float integral = integral + error;

    if(error == 0)
      integral = 0;

    if(integral > 12000)
		{
			integral = 0;
		}

    float derivative = error - prevError;
    float power = (error * kP) + (integral * kI) + (derivative * kD);
    prevError = error;

    drive_voltage(power, power);

    pros::lcd::set_text(1, std::to_string(Units.DegToIn(tracking.get_value())));

    pros::delay(15);
  }
  drive_voltage(0, 0);
}

//PID for translational movement
//Optimized for both accuracy and precision
void translatePID(double setpoint)
{
  double kP = 15.0;
  double kI = 0.000001;
  double kD = 8.0;

  float error = 11;
  float prevError;


  setpoint = Units.InToDeg(setpoint);

  while(std::abs(error) > 10)
  {
    error = setpoint - tracking.get_value();
    float integral = integral + error;

    if(error == 0)
      integral = 0;

    if(integral > 12000)
		{
			integral = 0;
		}

    float derivative = error - prevError;
    float power = (error * kP) + (integral * kI) + (derivative * kD);
    prevError = error;

    drive_voltage(power, power);

    pros::delay(15);
  }
  drive_voltage(0, 0);
}


/*
 * @brief PID-based turning
 *
 * Turns the robot on a point based on the distance to the desired heading from
 * the current heading. Has a time limit so the auton continues, even if the
 * robot does not quite reach the desired heading. This function cannot be
 * called while odometry is running.
 *
 * @param     setpoint     Desired heading
 * @param     kP, kI       Tunings variables (proportional and integral)
 * @param     time         Time limit
 */
void turnPID(int setpoint, float kP, float kI, int time)
{
  //float kP = 170.0;
  //float kI = 1.5; //1.4
  float kD = 0.6;

  setpoint *= 1000;

  float error = 5;
  float prevError;
  float integral = 0;
  timer clock; //Timer to limit time spent turning

  while(std::abs(error) > 0.5 && clock.time() < time)
  {
    error = setpoint - (gyro.get_rotation() * 1000);
    
    if(std::fabs(error) < Ki_active_t)
      integral = integral + error;
    else
      integral = 0;

    if(integral > Ki_limit_t)
      integral = Ki_limit_t;
    else if(integral < -1 * Ki_limit_t)
      integral = -1 * Ki_limit_t;

    float derivative = error - prevError;
    float power = (error * kP) + (integral * kI) + (derivative * kD);
    prevError = error;

    std::string aOut = "A: " + std::to_string(gyro.get_rotation());
    std::string errorOut = "Error: " + std::to_string(error);
    std::string powerOut = "Power: " + std::to_string(power);
    pros::lcd::set_text(1, aOut);
    pros::lcd::set_text(2, errorOut);
    pros::lcd::set_text(3, powerOut);

    drive_voltage(power, -power);

    pros::delay(15);
  }
  drive_voltage(0, 0);
  //tracking.reset();
}


//Experimental turn PID
void PIDTurn(double setpoint) {
  float kP = 0.24;
  float kI = 0.001;
  float kD = 0.6;

  float error = 2.0;
  float prevError;
  float integral;

  setpoint *= 1000;

  timer clock;

  while(std::fabs(error) > 1.0 && clock.time() < 500) {
    float adj_heading = gyro.get_rotation();

    error = setpoint - (adj_heading * 1000); 

    if(std::fabs(error) < Ki_active_t)
      integral = integral + error;
    else
      integral = 0;

    if(integral > Ki_limit_t)
      integral = Ki_limit_t;
    else if(integral < -1 * Ki_limit_t)
      integral = -1 * Ki_limit_t;

    float derivative = error - prevError;
    float power = (error * kP) + (integral * kI) + (derivative * kD);
    prevError = error;

    std::string aOut = "A: " + std::to_string(adj_heading);
    std::string errorOut = "Error: " + std::to_string(error/1000);
    std::string powerOut = "Power: " + std::to_string(power);
    pros::lcd::set_text(4, aOut);
    pros::lcd::set_text(5, errorOut);
    pros::lcd::set_text(6, powerOut);

    drive_voltage(power, -power);

    pros::delay(20);
  }
  drive_voltage(0, 0);
  //tracking.reset();
}


//Timing-based goal grab function (only used at the end of autons so doesn't
//need to be precise)
void goalGrab(int time) {
  drive_voltage(-6000, -6000);
  pros::delay(time);
  drive_voltage(0, 0);
}
