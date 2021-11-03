#include "define.h"

int loopExits = 0;
bool failedLoopExit = false;

float leftError = 6;
float rightError = 6;

void DriveTrainPID(float setpoint)
{
  float kP = 0.5; //Original 1.2
  float kI = 0.0;
  float kD = 0.2; //Original 0.3

  float error = 6;
  float prevError;

  while(std::abs(error) > 5)
  {
    error = setpoint - (((leftEncoder.get_position() / 100) + (rightEncoder.get_position() / 100)) / 2);
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

    for(pros::Motor m : driveTrain)
      m.move_velocity(power);

    pros::lcd::set_text(1, std::to_string(((leftEncoder.get_position() / 100) + (rightEncoder.get_position() / 100)) / 2));
  }
  driveFL.move_velocity(0);
  driveBL.move_velocity(0);
  driveFR.move_velocity(0);
  driveBR.move_velocity(0);
}

void TurnPID(int setpoint)
{
  float kP = 1.0;
  float kI = 0.0;
  float kD = 0.0;

  float error = 11;
  float prevError;

  while(true/*std::abs(error) > 10*/)
  {
    error = setpoint - imu.get_rotation();
    float integral = integral + rightError;

    float derivative = error - prevError;
    float power = (error * kP) + (integral * kI) + (derivative * kD);
    prevError = error;

    std::string errorOut = "Error: " + std::to_string(error);
    std::string powerOut = "Power: " + std::to_string(power);
    pros::lcd::set_text(1, errorOut);
    pros::lcd::set_text(2, powerOut);

    driveFL.move_velocity(power);
    driveBL.move_velocity(power);
    driveFR.move_velocity(-power);
    driveBR.move_velocity(-power);

  }
  driveFL.move_velocity(0);
  driveBL.move_velocity(0);
  driveFR.move_velocity(0);
  driveBR.move_velocity(0);
}

void SmallLiftPID(void* setpoint)
{
  float error = 0;

  while(true )
  {
    error = smallLiftSetpoint - smallLift.get_position();

    float power = (error * smallLiftKP);
    smallLift.move_velocity(power);

    pros::lcd::set_text(2, std::to_string(power));

    if(std::abs(error) < 3)
    {
      loopExits ++;
      pros::c::task_suspend(smallLiftTask);
    }

    pros::delay(15);
  }
  //liftMotor.move_velocity(0);
}

void BigLiftPID(void* setpoint)
{
  float error = 0;
  float prevError;

  float kP = 0.4;
  float kD = 0.0;

  while(std::abs(error) > 5)
  {
    error = bigLiftSetpoint - ((bigLift1.get_position() + bigLift2.get_position()) / 2);

    float derivative = error - prevError;
    float power = (error * kP) + (derivative * kD);
    prevError = error;

    bigLift1.move_velocity(power);
    bigLift2.move_velocity(power);

    if(std::abs(error) < 3)
      pros::c::task_suspend(bigLiftTask);

    pros::delay(15);
  }
}




void OnLeftButton()
{
	//If left button has been clicked before...
	if(autonSelect == 1)
	{
		//...then run left full auton
		autonSelect = 4;
		pros::lcd::set_text(3, "Full auton");
	}
  //If the center button has been clicked before...
  else if(autonSelect == 2)
  {
    //... then run left full win point auton
    autonSelect = 5;
    pros::lcd::set_text(3, "Full win point");
  }
	//If right button was clicked before this...
  else if(autonSelect == 3)
  {
		//...then run right full auton
		autonSelect = 6;
    pros::lcd::set_text(3, "Full auton");
  }
	//If this is the first button being pressed...
	else if(autonSelect == 0)
	{
		//..then set the value to left
		autonSelect = 1;
		pros::lcd::set_text(2, "Left");
	}
}

void OnCenterButton()
{
	//If left button was clicked before this...
	if(autonSelect == 1)
	{
		//Then run left grab auton
		autonSelect = 7;
		pros::lcd::set_text(3, "Grab");
	}
  //If the center button has been clicked before...
  else if(autonSelect == 2)
  {
    //... give error message
    pros::lcd::set_text(3, "Invalid selection");
  }
	//If right button was pressed before this...
	else if(autonSelect == 3)
	{
		//Then run right grab auton
		autonSelect = 8;
		pros::lcd::set_text(3, "Grab");
	}
	//If this is the first button being pressed...
	else if(autonSelect == 0)
	{
		//...then set the value to win point
		autonSelect = 2;
		pros::lcd::set_text(2, "Full win point");
	}
}

void OnRightButton()
{
  //If the left button has been clicked before...
  if(autonSelect == 1)
  {
    //...then run left win point auton
    autonSelect = 9;
    pros::lcd::set_text(3, "Win point");
  }
  //If the center button has been clicked before...
  else if(autonSelect == 2)
  {
    //... then run right full win point auton
    autonSelect = 10;
    pros::lcd::set_text(3, "Right");
  }
  //If right button has been clicked before this...
  else if(autonSelect == 3)
  {
    //...then run right win point auton
    autonSelect = 11;
    pros::lcd::set_text(3, "Win point");
  }
  //If this is the first button being pressed...
  else if(autonSelect == 0)
  {
    //..then set the value to right
    autonSelect = 3;
    pros::lcd::set_text(2, "Right");
  }
}




double RateLimiter(double input, int lastCall, double maxRateChange, double prevOutput)
{
  int timeSinceLastCall = pros::millis()/1000 - lastCall/1000;

  double maxChange = timeSinceLastCall * maxRateChange;
  double output;

  if((input - prevOutput) < -maxChange)
    output = -maxChange;
  else if(input - prevOutput > maxChange)
    output = maxChange;
  else
    output = input - prevOutput;

  return output;
}
