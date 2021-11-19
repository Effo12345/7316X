#include "functions.h"
#include "purepursuit.h"

int stickMultiplier = 1;
int stickScale = 4;
bool intakeToggle = false;
bool frontClipToggle = true;
bool backClipToggle = false;
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
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


	leftEncoder.reset_position();
	rightEncoder.set_reversed(true);
	rightEncoder.reset_position();
	backEncoder.reset();

	frontClip.set_value(true);

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
void competition_initialize() {}

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
	//Create lift task
	/*
	bigLiftTask = pros::c::task_create(BigLiftPID, (void*)1330, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Big lift");
	pros::c::task_suspend(bigLiftTask);

	fclose(targetVelocityL);
	fclose(targetVelocityR);
	fclose(measuredVelocityL);
	fclose(measuredVelocityR);

	bigLift1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	bigLift2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	*/

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

			//Button to toggle the front pneumatic clip
			if(master.get_digital_new_press(DIGITAL_R1))
				frontClipToggle = !frontClipToggle;

			//Button to toggle the back pneumatic clip
			if(master.get_digital_new_press(DIGITAL_R2))
				backClipToggle = !backClipToggle;


		//Get the values of the y-axes of the left and right sticks, and store them in left and right respectively
		int left = master.get_analog(ANALOG_LEFT_Y) * stickMultiplier;
		int right = master.get_analog(ANALOG_RIGHT_Y) * stickMultiplier;

		//Calculate the movement of the lift based on the L1 and L2 buttons
		int liftValue = (master.get_digital(DIGITAL_L1) - master.get_digital(DIGITAL_L2));

		//Set the power of the drivetrain motors based on the controller sticks
		if(stickMultiplier + 1)
		{
			for(auto &m : driveTrainL) {m.move(left);}
			for(auto &m : driveTrainR) {m.move(right);}
		}
		else
		{
			for(auto &m : driveTrainL) {m.move(right);}
			for(auto &m : driveTrainR) {m.move(left);}
		}

		//Set the power of the mobile goal lifts motor based on the value calculated above
		//smallLift.move_velocity(smallLiftValue * 100);
		lift.move_velocity(liftValue * 200);

		//Set the power of the ringle intake based on value calculated above
		intake.move_velocity(400 * intakeToggle);

		//Sets the state of the pneumatic clip based on the value calculated above
		frontClip.set_value(frontClipToggle);
		backClip.set_value(backClipToggle);

		//pros::lcd::set_text(1, std::to_string(smallLift.get_position()));

		pros::lcd::set_text(1, std::to_string(imu.get_rotation()));
		pros::lcd::set_text(2, std::to_string(rightEncoder.get_position() / 100));
		pros::lcd::set_text(3, std::to_string(backEncoder.get_value()));
		pros::lcd::set_text(4, std::to_string(imu.get_rotation()));

		pros::delay(20);
	}
}
