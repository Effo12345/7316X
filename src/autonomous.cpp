#include "functions.h"

void RightWinPoint()
{

}

void LeftWinPoint()
{

}

void RightGrab()
{

}

void LeftGrab()
{

}

void LeftFull()
{
  //Move to neutral mobile goal
  DriveTrainPID(100);

  //Activate front pneumatic clip
  frontClip.set_value(true);

  //Move drivetrain back
  //Potentially add arc move here
  DriveTrainPID(50);

  

}

void None()
{

}
