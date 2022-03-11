#ifndef STATEMACHINE_HPP
#define STATEMACHINE_HPP

//State machine to store the current state of the robot (to be expanded in future)
class stateMachine {
private:
  bool holdingGoal = false;
  bool guardDown = false;
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
};

extern stateMachine stateMachine;

#endif //STATEMACHINE_HPP
