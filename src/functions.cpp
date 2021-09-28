#include "define.h"

int loopExits = 0;
bool failedLoopExit = false;

void SmallLiftPID(void* setpoint)
{
  float error = 0;
  float kP = 0.25;

  while(true )
  {
    error = smallLiftSetpoint - smallLift.get_position();

    float power = (error * kP);
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
  float kP;

  while(true)
  {
    error = bigLiftSetpoint - ((bigLift1.get_position() + bigLift2.get_position()) / 2);

    float power = (error * kP);
    bigLift1.move_velocity(power);
    bigLift2.move_velocity(power);

    if(std::abs(error) < 3)
      pros::c::task_suspend(bigLiftTask);

    pros::delay(15);
  }
}
