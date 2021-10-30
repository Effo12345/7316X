#include "define.h"

//L_DISTANCE_IN and R_DISTANCE_IN are the distances from the tracking center to the left and right tracking wheels prespectively
//sPos is likely a struct containing the previous positions of the encoders

float L_DISTANCE_IN = 6.50;
float R_DISTANCE_IN = 7.00;
float S_DISTANCE_IN = 0.00;

void trackPosition(void* test){
  while(true)
  {
    int left = leftEncoder.get_position() / 100;
    int right = rightEncoder.get_position() / 100;
    int back = backEncoder.get_value();

    float L = (left - position.leftLst) / wheelConversionFactor     /*41.671*/; //The amount the left tracking wheel moved in inches
    float R = (right - position.rightLst) / wheelConversionFactor; //The amount the right tracking wheel moved in inches
    float S = (back - position.backLst) / 45.837; //The amount the tracking wheel moved in inches

    //Update the last values
    position.leftLst = left;
    position.rightLst = right;
    position.backLst = back;

    float h; //The hypotenuse of the triangle formed by the middle of the robot on the starting position and ending position and the middle of the circle it travels around
      float i; //Half on the angle that I've traveled
      float h2; //The same as h but using the back instead of the side wheels
      float a = (L - R) / (L_DISTANCE_IN + R_DISTANCE_IN); // The angle that I've traveled


    if (a){
      i = (a/2.0);
          float sinI = sin(i);
          h = (((R / a) + R_DISTANCE_IN) * sinI) * 2.0;
          h2 = (((S / a) + S_DISTANCE_IN) * sinI) * 2.0;
      }
      else{
          h = R; //Sets relative distance traveled to the right wheel as since their deltas are equal they traveled the same distance
          i = 0; //Half of angle traveled will always be zero if the angle traveled is sero
          h2 = S; //Back wheel movement is simply the difference in the back wheel if the angle didnt change
      }

    float p = i + position.a; //The global ending angle of the robot
    float cosP = cos(p); //Cleans up final variable storage math
    float sinP = sin(p); //Cleans up final variable storage math

    //Update the global position
    position.y += h * cosP;
    position.x += h * sinP;

    //Update global position with back wheeldata
    position.y += h2 * -sinP; //-sin(x) = sin(-x)
    position.x += h2 * cosP; //cos(x) = cos(-x)

    //Update the global rotation value according to relative value calculated earlier
    position.a += a;

    std::string xOutput = "X: " + std::to_string(position.x);
    pros::lcd::set_text(1, xOutput);

    std::string yOutput = "Y: " + std::to_string(position.y);
    pros::lcd::set_text(2, yOutput);

    std::string aOutput = "Rotation: " + std::to_string(position.a);
    pros::lcd::set_text(3, aOutput);

    pros::delay(10);
  }
}
