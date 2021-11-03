#include "functions.h"
#include "purepursuit.h"

int stickMultiplier = 1;
bool intakeToggle = false;
bool clipToggle = true;
bool smallLiftValue = true;
//bool bigLiftValue = false;


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();

	pros::lcd::register_btn0_cb(OnLeftButton);
	pros::lcd::register_btn1_cb(OnCenterButton);
	pros::lcd::register_btn2_cb(OnRightButton);

	//Set the braking mode for the mobile goal lift so it holds its position when no button is being pressed
	smallLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	bigLift1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	bigLift2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	smallLift.tare_position();

	leftEncoder.reset_position();
	leftEncoder.set_reversed(true);
	rightEncoder.reset_position();
	backEncoder.reset();

	imu.reset();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	/*
	pros::lcd::set_text(1, "Ran autonon");
	autonomous();
	*/
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	//PurePursuitInit();

	autonPointers[autonSelect]();

/*
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



	TurnPID(-60);
	pros::lcd::set_text(7, "Turn finished");
	//TurnPID(-1900, 230); //Original: -305, 230
	//pros::lcd::set_text(1, "Turn completed");

/*
	bigLift1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	bigLift2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	while((bigLift1.get_position() + bigLift2.get_position()) / 2 > 100)
	{
		pros::lcd::set_text(5, std::to_string((bigLift1.get_position() + bigLift2.get_position()) / 2));
		pros::delay(10);
	}
	clip.set_value(false);


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
	*/
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	//Create lift tasks
	smallLiftTask = pros::c::task_create(SmallLiftPID, (void*)1330, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Small lift");
	pros::c::task_suspend(smallLiftTask);
	smallLiftSetpoint = -1115;

	bigLiftTask = pros::c::task_create(BigLiftPID, (void*)1330, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Big lift");
	pros::c::task_suspend(bigLiftTask);

	fclose(targetVelocityL);
	fclose(targetVelocityR);
	fclose(measuredVelocityL);
	fclose(measuredVelocityR);

	bigLift1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	bigLift2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	while (true) {
		//Updates the on-screen buttons
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

		//Button to invert sticks
		if(master.get_digital_new_press(DIGITAL_A))
			stickMultiplier *= -1;

		//Button to toggle ringle intake
		if(master.get_digital_new_press(DIGITAL_B))
			intakeToggle = !intakeToggle;

			//Button to toggle the pneumatic clip
			if(master.get_digital_new_press(DIGITAL_Y))
				clipToggle = !clipToggle;

		//Get the values of the y-axes of the left and right sticks, and store them in left and right respectively
		int left = master.get_analog(ANALOG_LEFT_Y) * stickMultiplier;
		int right = master.get_analog(ANALOG_RIGHT_Y) * stickMultiplier;

		//-1330
		//-1246 (better)
		//Run small lift PID when the L1 or L2 buttons are pressed
		if(master.get_digital_new_press(DIGITAL_L1) && !smallLiftValue)
		{
			smallLiftSetpoint = -1115;
			smallLiftKP = 0.25;
			smallLiftValue = true;
			pros::c::task_resume(smallLiftTask);
		}
		else if(master.get_digital_new_press(DIGITAL_L2) && smallLiftValue)
		{
			smallLiftSetpoint = -1574;
			smallLiftKP = 0.1;
			smallLiftValue = false;
			pros::c::task_resume(smallLiftTask);
		}

		//Calculate the movement of the lift based on the L1 and L2 buttons
		int smallLiftValue = (master.get_digital(DIGITAL_L1) - master.get_digital(DIGITAL_L2));
		int bigLiftValue = (master.get_digital(DIGITAL_R1) - master.get_digital(DIGITAL_R2));

		//Set the power of the drivetrain motors based on the controller sticks
		if(stickMultiplier + 1)
		{
			driveFL.move(left);
			driveBL.move(left);
			driveFR.move(right);
			driveBR.move(right);
		}
		else
		{
			driveFL.move(right);
			driveBL.move(right);
			driveFR.move(left);
			driveBR.move(left);
		}

		//Set the power of the mobile goal lifts motor based on the value calculated above
		//smallLift.move_velocity(smallLiftValue * 100);
		bigLift1.move_velocity(bigLiftValue * 100);
		bigLift2.move_velocity(bigLiftValue * 100);

		//Set the power of the ringle intake based on value calculated above
		intake.move_velocity(600 * intakeToggle);

		//Sets the state of the pneumatic clip based on the value calculated above
		clip.set_value(clipToggle);

		//pros::lcd::set_text(1, std::to_string(smallLift.get_position()));

		pros::lcd::set_text(1, std::to_string(leftEncoder.get_position() / 100));
		pros::lcd::set_text(2, std::to_string(rightEncoder.get_position() / 100));
		pros::lcd::set_text(3, std::to_string(backEncoder.get_value()));
		pros::lcd::set_text(4, std::to_string(imu.get_rotation()));

		pros::delay(20);
	}
}
