#include "main.h"

#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP

//Externs for functions from other cpp files
//drive.cpp
void drive_voltage(int left, int right);
void drive_op(int left, int right);
void drive_velocity(int left, int right);
void hyperGrab(double setpoint);
void translatePID(double setpoint);
void turnPID(int setpoint, float kP, float kI, int time);
void goalGrab(int time);
void PIDTurn(double setpoint);

//intake.cpp
void intake_rot(int rotations);
void ringIntake(intakeState state);
void ringMove(int time);
void auto_intake(bool autoIntake);

//interface.cpp
void initSelector();
void updateSelector();
void (*getSelection()) ();
void None();

//autonomous.cpp
void LeftGrab();
void RightGrab();
void LeftFull();
void RightFull();
void LeftWinPoint();
void RightWinPoint();
void FullWinPoint();
void DoubleGrab();
void Skills();


#endif //FUNCTIONS_HPP
