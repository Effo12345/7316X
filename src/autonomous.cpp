#include "functions.h"

void RightWinPoint()
{
  smallLiftTask = pros::c::task_create(SmallLiftPID, (void*)1330, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Small lift");
	smallLiftSetpoint = -1650;
	smallLiftKP = 0.1;

	while(smallLift.get_position() > smallLiftSetpoint + 20)
		pros::delay(20);

	driveTrainSetpoint = -700; //Original: 1469
	driveTrainTask = pros::c::task_create(DriveTrainPID, (void*)1330, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Drivetrain");

	while((((leftEncoder.get_position() / 100) + (rightEncoder.get_position() / 100)) / 2) > driveTrainSetpoint + 20)
		pros::delay(20);

	smallLiftSetpoint = -1115;
	smallLiftKP = 0.25;

	while(smallLift.get_position() < smallLiftSetpoint - 10)
		pros::delay(20);

	driveTrainSetpoint = 0;

	intake.tare_position();
	intake.move_velocity(200);

	while((intake.get_position()) < (360 * 10))
		pros::delay(20);

	intake.move_velocity(0);

  pros::delay(250);
  pros::c::task_delete(driveTrainTask);
	pros::c::task_delete(smallLiftTask);
}

void LeftWinPoint()
{
  smallLiftTask = pros::c::task_create(SmallLiftPID, (void*)1330, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Small lift");
  smallLiftSetpoint = -1650;
  smallLiftKP = 0.1;

  while(smallLift.get_position() > smallLiftSetpoint + 20)
    pros::delay(20);

  driveTrainSetpoint = -500; //Original: 1469
  driveTrainTask = pros::c::task_create(DriveTrainPID, (void*)1330, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Drivetrain");

  while((((leftEncoder.get_position() / 100) + (rightEncoder.get_position() / 100)) / 2) > driveTrainSetpoint + 20)
    pros::delay(20);

  smallLiftSetpoint = -1115;
  smallLiftKP = 0.25;

  while(smallLift.get_position() < smallLiftSetpoint - 10)
    pros::delay(20);

  driveTrainSetpoint = 0;

  intake.tare_position();
  intake.move_velocity(200);

  while((intake.get_position()) < (360 * 10))
    pros::delay(20);

  intake.move_velocity(0);

  driveFL.move_velocity(0);
  driveFR.move_velocity(0);
  driveBL.move_velocity(0);
  driveBR.move_velocity(0);

  pros::c::task_delete(driveTrainTask);
	pros::c::task_delete(smallLiftTask);
}

void RightGrab()
{
  clip.set_value(false);
	driveTrainSetpoint = 1500; //Original: 1469
	driveTrainTask = pros::c::task_create(DriveTrainPID, (void*)1330, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Drivetrain");
	pros::delay(500);

	bigLift1.move_velocity(-100);
	bigLift2.move_velocity(-100);
	pros::delay(500);
	bigLift1.move_velocity(0);
	bigLift2.move_velocity(0);

	bigLift1.tare_position();
	bigLift2.tare_position();

	while((((leftEncoder.get_position() / 100) + (rightEncoder.get_position() / 100)) / 2) < (driveTrainSetpoint - 30))
		pros::delay(20);
	clip.set_value(true);
	pros::delay(250);

	driveTrainSetpoint = 100;

  while((((leftEncoder.get_position() / 100) + (rightEncoder.get_position() / 100)) / 2) > (driveTrainSetpoint + 30))
		pros::delay(20);

  pros::c::task_delete(driveTrainTask);

/*
  driveFL.move_velocity(100);
  driveFR.move_velocity(-100);
  driveBL.move_velocity(100);
  driveBR.move_velocity(-100);

  while(imu.get_rotation() < 90)
    pros::delay(20);
*/
    driveFL.move_velocity(0);
    driveFR.move_velocity(0);
    driveBL.move_velocity(0);
    driveBR.move_velocity(0);

}

void LeftGrab()
{
  clip.set_value(false);
	driveTrainSetpoint = 1500; //Original: 1469
	driveTrainTask = pros::c::task_create(DriveTrainPID, (void*)1330, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Drivetrain");
	pros::delay(500);

	bigLift1.move_velocity(-100);
	bigLift2.move_velocity(-100);
	pros::delay(500);
	bigLift1.move_velocity(0);
	bigLift2.move_velocity(0);

	bigLift1.tare_position();
	bigLift2.tare_position();

	while((((leftEncoder.get_position() / 100) + (rightEncoder.get_position() / 100)) / 2) < (driveTrainSetpoint - 30))
		pros::delay(20);
	clip.set_value(true);
	pros::delay(250);

	driveTrainSetpoint = 100;

  while((((leftEncoder.get_position() / 100) + (rightEncoder.get_position() / 100)) / 2) > (driveTrainSetpoint))
		pros::delay(20);

  pros::c::task_delete(driveTrainTask);
}

void LeftFull()
{
  clip.set_value(false);
	driveTrainSetpoint = 1500; //Original: 1469
	driveTrainTask = pros::c::task_create(DriveTrainPID, (void*)1330, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Drivetrain");
	pros::delay(500);

	smallLiftTask = pros::c::task_create(SmallLiftPID, (void*)1330, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Small lift");
	smallLiftSetpoint = -1650;
	smallLiftKP = 0.1;

	bigLift1.move_velocity(-100);
	bigLift2.move_velocity(-100);
	pros::delay(500);
	bigLift1.move_velocity(0);
	bigLift2.move_velocity(0);

	bigLift1.tare_position();
	bigLift2.tare_position();

	while((((leftEncoder.get_position() / 100) + (rightEncoder.get_position() / 100)) / 2) < (driveTrainSetpoint - 30))
		pros::delay(20);
	clip.set_value(true);
	pros::delay(250);

	driveTrainSetpoint = 750;

	while((((leftEncoder.get_position() / 100) + (rightEncoder.get_position() / 100)) / 2) > (driveTrainSetpoint))
	{
		if((bigLift1.get_position() + bigLift2.get_position()) / 2 < 400)
		{
			bigLift1.move_velocity(100);
			bigLift2.move_velocity(100);
		}
		else
		{
			bigLift1.move_velocity(0);
			bigLift2.move_velocity(0);
		}
		pros::delay(20);
	}

	pros::delay(100);

	leftEncoder.reset_position();
	rightEncoder.reset_position();
	driveTrainSetpoint = 0;
	pros::c::task_delete(driveTrainTask);
	//pros::c::task_suspend(driveTrainTask);
	pros::delay(250);

  driveFL.move_velocity(0);
  driveFR.move_velocity(0);
  driveBL.move_velocity(0);
  driveBR.move_velocity(0);

  driveFL.move_velocity(100);
  driveFR.move_velocity(-100);
  driveBL.move_velocity(100);
  driveBR.move_velocity(-100);

  while(imu.get_rotation() < 90)
    pros::delay(20);

  driveFL.move_velocity(0);
  driveFR.move_velocity(0);
  driveBL.move_velocity(0);
  driveBR.move_velocity(0);
	pros::lcd::set_text(7, "Turn finished");
	//TurnPID(-1900, 230); //Original: -305, 230
	//pros::lcd::set_text(1, "Turn completed");

	leftEncoder.reset_position();
	rightEncoder.reset_position();

	pros::delay(100);
	driveTrainSetpoint = -400;
	driveTrainKP = 0.5;
	pros::c::task_resume(driveTrainTask);

	while(std::abs(driveError) > 10)
		pros::delay(20);

	pros::lcd::set_text(1, "Moved back");
	smallLiftSetpoint = -1115;
	smallLiftKP = 0.25;
	pros::lcd::set_text(1, "Moved lift up");
	driveTrainSetpoint = 0;

	//driveTrainSetpoint = 5;
	//pros::c::task_resume(driveTrainTask);

	intake.tare_position();
	intake.move_velocity(200);

	while((intake.get_position()) < 360)
		pros::delay(20);

	intake.move_velocity(0);
	//big lift top position: 2269

	pros::c::task_delete(driveTrainTask);
	pros::c::task_delete(smallLiftTask);
}

void None()
{

}
