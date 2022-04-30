#include "main.h"

#ifndef STATEMACHINE_HPP
#define STATEMACHINE_HPP

//State machine to store the current state of the robot (to be expanded in future)
class stateMachine {
private:
  bool holdingGoal = false;
  bool guardDown = false;

  int prevTracking = 0;
  int settledTime = 0;
public:
  void setHoldGoal() {
    holdingGoal = true;
  }

  void setGuard() {
    guardDown = true;
  }


  bool getGoalState() {
    return holdingGoal;
  }

  bool getGuardState() {
    return guardDown;
  }

  bool isSettled() {
    if((std::abs(tracking.get_value() - prevTracking)) > 0) {
      prevTracking = tracking.get_value();
      settledTime++;
      return false;
    }
    else if(settledTime > 100) {
      prevTracking = tracking.get_value();
      settledTime = 0;
      return true;
    }
  }
};

extern stateMachine stateMachine;

extern void initTOW();
extern void stopTOW();
extern void tmp();
extern pros::Task tOW;
extern bool forceQuit_t;

#endif //STATEMACHINE_HPP
