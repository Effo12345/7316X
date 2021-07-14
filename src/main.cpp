#include "odometry.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();

	pros::lcd::register_btn1_cb(on_center_button);

	lEncoder.reset();
	rEncoder.reset();
	bEncoder.reset();
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
void autonomous() {}

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
	int left = 0;
	int right = 0;
	int back = 0;

	position.leftLst = 0;
	position.rightLst = 0;
	position.backLst = 0;

	while (true) {
		left = lEncoder.get_value();
		right = rEncoder.get_value();
		back = bEncoder.get_value();

		/*
		pros::lcd::set_text(1, std::to_string(left));
		pros::lcd::set_text(2, std::to_string(right));
		pros::lcd::set_text(3, std::to_string(back));
		pros::delay(15);
		*/

		trackPosition(left, right, back, position);
		/*
		std::string str1 = "X: " + std::to_string(position.x);
	  std::string str2 = "Y: " + std::to_string(position.y);
	  std::string str3 = "A: " + std::to_string(position.a);
	  pros::lcd::set_text(1, str1);
	  pros::lcd::set_text(2, str2);
	  pros::lcd::set_text(3, str3);
		*/

		pros::lcd::set_text(2, std::to_string(left));
		pros::lcd::set_text(3, std::to_string(right));
		pros::lcd::set_text(4, std::to_string(back));

		pros::delay(10);
	}
}
