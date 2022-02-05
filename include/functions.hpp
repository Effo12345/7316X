#include "main.h"

#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP

//Externs for functions from other cpp files
//drive.cpp
void start_odom();
void quit_odom();
void start_pp(std::vector<std::vector<double>> waypoints, int lookaheadDistance);
void start_pp_async(std::vector<std::vector<double>> waypoints, int lookaheadDistance);
void quit_pp();
void drive_voltage(int left, int right);
void drive_op(int left, int right);
void speedPID(double setpoint);

//interface.cpp
void initSelector();
void updateSelector();
void (*getSelection()) ();

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
