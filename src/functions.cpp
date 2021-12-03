#include "define.h"

//Template functions to convert between inches and degrees
template <class N>
N InToDeg (N input) {
  N output = input * wheelConversionFactor;
  return output;
}

template <class N>
N DegToIn (N input) {
  N output = input / wheelConversionFactor;
  return output;
}


//Set time limits for PID loops (idea)

//Main DriveTrain movement PID
void DriveTrainPID(float setpoint)
{
  float kP = 25.0;
  float kI = 0.0;
  float kD = 3.0;

  float error = 16;
  float prevError;


  setpoint = InToDeg(setpoint);

  pros::lcd::set_text(1, std::to_string(setpoint));

  while(std::abs(error) > 15)
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

    for(auto &m : driveTrain) {m.move_voltage(power);}

    if(error < 0)
      pros::lcd::set_text(6, std::to_string(error));
    else
      pros::lcd::set_text(6, "No steady state");

    pros::lcd::set_text(2, std::to_string(((leftEncoder.get_position() / 100) + (rightEncoder.get_position() / 100)) / 2));
    pros::lcd::set_text(3, std::to_string(power));
    pros::lcd::set_text(4, std::to_string(error));

    pros::delay(15);
  }
  for(auto &m : driveTrain) {m.move_voltage(0);}
}

//Forces the robot into walls to grab mobile goals with the back clip
void WallPush(int time)
{
  for(auto &m : driveTrain) {m.move_voltage(-12000);}
  pros::delay(time);
  for(auto &m : driveTrain) {m.move_voltage(0);}
}


//Coarse turn PID for turns of at least 90 degrees and Fine turn for less than that
void CoarseTurn(int setpoint)
{
  float kP = 100.0;
  float kI = 0.0;
  float kD = 8.0;

  float error = 5;
  float prevError;

  while(std::abs(error) > 0.5)
  {
    error = setpoint - imu.get_rotation();
    float integral = integral + error;

    float derivative = error - prevError;
    float power = (error * kP) + (integral * kI) + (derivative * kD);
    prevError = error;

    std::string aOut = "A: " + std::to_string(imu.get_rotation());
    std::string errorOut = "Error: " + std::to_string(error);
    std::string powerOut = "Power: " + std::to_string(power);
    pros::lcd::set_text(1, aOut);
    pros::lcd::set_text(2, errorOut);
    pros::lcd::set_text(3, powerOut);

    for(auto &m : driveTrainL) {m.move_voltage(power);}
    for(auto &m : driveTrainR) {m.move_voltage(-power);}

    pros::delay(15);
  }
  for(auto &m : driveTrain) {m.move_voltage(0);}
  leftEncoder.reset_position();
  rightEncoder.reset_position();
}

void FineTurn(int setpoint)
{
  float kP = 170.0;
  float kI = 1.0;
  float kD = 6.0;

  float error = 5;
  float prevError;

  while(std::abs(error) > 0.5)
  {
    error = setpoint - imu.get_rotation();
    float integral = integral + error;

    float derivative = error - prevError;
    float power = (error * kP) + (integral * kI) + (derivative * kD);
    prevError = error;

    std::string aOut = "A: " + std::to_string(imu.get_rotation());
    std::string errorOut = "Error: " + std::to_string(error);
    std::string powerOut = "Power: " + std::to_string(power);
    pros::lcd::set_text(1, aOut);
    pros::lcd::set_text(2, errorOut);
    pros::lcd::set_text(3, powerOut);

    for(auto &m : driveTrainL) {m.move_voltage(power);}
    for(auto &m : driveTrainR) {m.move_voltage(-power);}

    pros::delay(15);
  }
  for(auto &m : driveTrain) {m.move_voltage(0);}
  leftEncoder.reset_position();
  rightEncoder.reset_position();
}

//Perpendicular turn (multiples of 90 degrees)
void PTurn(int setpoint)
{
  float kP = 115.3;
  float kI = 0.0;
  float kD = 8.0;

  float error = 5;
  float prevError;

  while(std::abs(error) > 0.5)
  {
    error = setpoint - imu.get_rotation();
    float integral = integral + error;

    float derivative = error - prevError;
    float power = (error * kP) + (integral * kI) + (derivative * kD);
    prevError = error;

    std::string aOut = "A: " + std::to_string(imu.get_rotation());
    std::string errorOut = "Error: " + std::to_string(error);
    std::string powerOut = "Power: " + std::to_string(power);
    pros::lcd::set_text(1, aOut);
    pros::lcd::set_text(2, errorOut);
    pros::lcd::set_text(3, powerOut);

    for(auto &m : driveTrainL) {m.move_voltage(power);}
    for(auto &m : driveTrainR) {m.move_voltage(-power);}

    pros::delay(15);
  }
  for(auto &m : driveTrain) {m.move_voltage(0);}
  leftEncoder.reset_position();
  rightEncoder.reset_position();
}

//The robot arcs as it moves for more efficient routes
void ArcMove(float setpoint, float reduction, turnDirection direction)
{
  //Left PID variables
  float kP = 25.0;
  float kI = 0.0;
  float kD = 3.0;

  float error = 10;
  float prevError;


  //PID loop
  while(std::abs(error) > 5)
  {
    error = setpoint - (rightEncoder.get_position() / 100);
    float integral = integral + error;

    if(error == 0)
      integral = 0;

    if(integral > 12000)
			integral = 0;

    float derivative = error - prevError;
    float power = (error * kP) + (integral * kI) + (derivative * kD);
    prevError = error;


    //Assign power based on turning direction
    if(direction == left)
    {
      for(auto &m : driveTrainL) {m.move_voltage(power * reduction);}
      for(auto &m : driveTrainR) {m.move_voltage(power);}
    }
    else
    {
      for(auto &m : driveTrainL) {m.move_voltage(power);}
      for(auto &m : driveTrainR) {m.move_voltage(power * reduction);}
    }

    pros::delay(15);
  }
  //for(auto &m : driveTrain) {m.move_voltage(0);}
}

//Runs the ring intake for a certain number of rotation
void RingIntake(int rotations)
{
  intake.tare_position();

  intake.move_velocity(500);

  pros::lcd::set_text(5, std::to_string(intake.get_position()));

  while(intake.get_position() < (rotations * 360))
    pros::delay(20);

  intake.move_velocity(0);
}

//Grab driver load rings during driver control
void RingGrab()
{
  leftEncoder.reset_position();
  rightEncoder.reset_position();
  intake.move_velocity(600);
  while(master.get_digital(DIGITAL_DOWN))
  {
    while(DegToIn(((leftEncoder.get_position() / 100) + (rightEncoder.get_position() / 100)) / 2) < 5 && master.get_digital(DIGITAL_DOWN))
      for(auto &m : driveTrain) {m.move_velocity(25);}

      while(DegToIn(((leftEncoder.get_position() / 100) + (rightEncoder.get_position() / 100)) / 2) > 0 && master.get_digital(DIGITAL_DOWN))
        for(auto &m : driveTrain) {m.move_velocity(-25);}
  }
  intake.move_velocity(0);
}

//Move the lift based on an enumerated value (down, platform, up)
void Lift(liftState state)
{
  if(state == up)
  {
    lift.move_velocity(200);

    while(lift.get_position() < 2150)
      pros::delay(20);

    lift.move_velocity(0);
  }
  else if(state == down)
  {
    lift.move_velocity(-200);

    while(lift.get_position() > 5)
      pros::delay(20);

    lift.move_velocity(0);
  }
  else
  {
    lift.move_velocity(200);

    while(lift.get_position() < 100)
      pros::delay(20);

    lift.move_velocity(0);
  }
}


//Start of auton selector
void OnLeftButton()
{
	//If left button has been clicked before...
	if(autonSelect == 1)
	{
		//...then run left Primary auton
		autonSelect = 4;
		pros::lcd::set_text(3, "Primary auton");
	}
  //If the center button has been clicked before...
  else if(autonSelect == 2)
  {
    //... then run left Double Grab auton
    autonSelect = 5;
    pros::lcd::set_text(3, "Full Grab L");
  }
	//If right button was clicked before this...
  else if(autonSelect == 3)
  {
		//...then run right primary auton
		autonSelect = 6;
    pros::lcd::set_text(3, "Primary auton");
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
    //... Then run full win point auton
    autonSelect = 12;
    pros::lcd::set_text(3, "Full Win Point");
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
		//...then set the value to Misc
		autonSelect = 2;
		pros::lcd::set_text(2, "Misc");
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
    //... then run right full grab
    autonSelect = 10;
    pros::lcd::set_text(3, "Double Grab R");
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



//Rate limiter for Pure Pursuit algorithm
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
