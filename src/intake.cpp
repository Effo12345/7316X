#include "functions.hpp"

//Runs the ring intake for a certain number of rotation
void intake_rot(int rotations)
{
  intake.tare_position();

  intake.move_velocity(500);

  pros::lcd::set_text(5, std::to_string(intake.get_position()));

  while(intake.get_position() < (rotations * 360))
    pros::delay(20);

  intake.move_velocity(0);
}

//Function to set a persistent state of the intake
void ringIntake(intakeState state) {
  if(state == on)
    intake.move_velocity(600);
  else
    intake.move_velocity(0);
}

//Move the robot forward slower to grab rings
void ringMove(int time) {
  drive_velocity(50, 50);
  pros::delay(time);
  drive_velocity(0, 0);
}

void auto_intake(bool autoIntake) {
  if(autoIntake) {
    if(lift.get_position() > 146)
      intake.move_velocity(600);
    else
      intake.move_velocity(0);
  }
}
