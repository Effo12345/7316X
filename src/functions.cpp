#include "define.h"

template <class N>
N InToDeg (N input) {
  N output = input * wheelConversionFactor;
  return output;
}

//Use move_voltage
//max: 12000

//Set time limits for PID loops
void DriveTrainPID(float setpoint)
{
  float kP = 25.0;
  float kI = 0.0;
  float kD = 3.0;

  float error = 11;
  float prevError;


  setpoint = InToDeg(setpoint);

  pros::lcd::set_text(1, std::to_string(setpoint));

  while(std::abs(error) > 10)
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

void WallPush()
{
  for(auto &m : driveTrain) {m.move_voltage(-12000);}
  pros::delay(700);
  for(auto &m : driveTrain) {m.move_voltage(0);}
}

//Coarse turn PID first, then fine turn
void TurnPID(int setpoint)
{
  float kP = 120.0;
  float kI = 0.0;
  float kD = 8.0;

  float error = 5;
  float prevError;

  while(std::abs(error) > 3)
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

void ArcMove(float turn, float smallArcD)
{
  //Find closest angle to turn towards
  float turnSetpoint = std::abs(turn - imu.get_rotation());
  if(turnSetpoint > 180)
    turnSetpoint -= 360;

  //Drivetrain PID variables
  float driveKP = 1.0;
  float driveKI = 0.0;
  float driveKD = 0.0;

  float driveError = 6;
  float drivePrevError;


  //Turn PID variables
  float turnKP = 1.0;
  float turnKI = 0.0;
  float turnKD = 0.0;

  float turnError = 6;
  float turnPrevError;

  //Set the encoder used to be the one following the shortest arc
  int encoderIn;
  std::array<pros::Motor, 3> largeArc = driveTrainL;
  std::array<pros::Motor, 3> smallArc = driveTrainR;
  if(turnSetpoint > 0)
  {
    encoderIn = 1;
    std::array<pros::Motor, 3> largeArc = driveTrainL;
    std::array<pros::Motor, 3> smallArc = driveTrainR;
  }
  else
  {
    encoderIn = 0;
    std::array<pros::Motor, 3> largeArc = driveTrainR;
    std::array<pros::Motor, 3> smallArc = driveTrainL;
  }

  while(std::abs(driveError) > 5)
  {
    //Calculate drivePower
    driveError = smallArcD - (encoders[encoderIn].get_position() / 100);
    float integral = integral + driveError;

    if(driveError == 0)
      integral = 0;

    if(integral > 12000)
		{
			integral = 0;
		}

    float derivative = driveError - drivePrevError;
    float drivePower = (driveError * driveKP) + (integral * driveKI) + (derivative * driveKD);
    drivePrevError = driveError;



    //Calculate turnPower
    turnError = turnSetpoint - imu.get_rotation();
    float turnIntegral = turnIntegral + turnError;

    if(turnError == 0)
      turnIntegral = 0;

    if(turnIntegral > 12000)
		{
			turnIntegral = 0;
		}

    float turnDerivative = turnError - turnPrevError;
    float turnPower = (turnError * turnKP) + (turnIntegral * turnKI) + (turnDerivative * turnKD);
    turnPrevError = turnError;


    for(auto &m : largeArc) {m.move_velocity(drivePower + turnPower);}
    for(auto &m : smallArc) {m.move_velocity(drivePower);}

    pros::delay(15);
  }
}


void BigLiftPID(void* setpoint)
{
  float error = 0;
  float prevError;

  float kP = 0.4;
  float kD = 0.0;

  while(std::abs(error) > 5)
  {
    error = bigLiftSetpoint - lift.get_position();

    float derivative = error - prevError;
    float power = (error * kP) + (derivative * kD);
    prevError = error;

    lift.move_velocity(power);

    if(std::abs(error) < 3)
      pros::c::task_suspend(bigLiftTask);

    pros::delay(15);
  }
}

void RingIntake(int rotations)
{
  intake.move_velocity(500);

  pros::lcd::set_text(5, std::to_string(intake.get_position()));

  while(intake.get_position() < (rotations * 360))
    pros::delay(20);

  intake.move_velocity(0);

  intake.tare_position();
}

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
