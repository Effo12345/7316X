#include "functions.hpp"

//Declare new type, "selection", to efficiently hold selector data
struct selection {
  std::string name;
  void (*pointer) (){};
};

//Empty function
void None() {}

//Declares function pointers to hold the call to auton functions
void (*grabL) (){&LeftGrab}, (*grabR) (){&RightGrab}, (*winPointL) (){&LeftWinPoint},
                  (*winPointR) (){&RightWinPoint}, (*fullL) (){&LeftFull},
                  (*fullR) (){&RightFull}, (*dGrab) (){&DoubleGrab},
                  (*wP) (){&FullWinPoint}, (*skills) (){&Skills}, (*none) (){&None};

//std::optional stores whether the variable has been initialized
std::optional<int> yVal;
//Final output to be called at the start of autonomous
selection autonToRun;

//Create 2D array of selections that will be accessed based on the order of
//buttons pressed. The array holds both the auton's name (to be displayed on the screen)
//and its function pointer.
std::array<std::array<selection, 3>, 3> selectArray {{
  {{{"Left full", fullL }, {"Left grab", grabL}, {"Left win point", winPointL}}},
  {{{"Full win point", wP}, {"Skills", skills}, {"Double grab", dGrab}}},
  {{{"Right full", fullR}, {"Right grab", grabR}, {"Right win point", winPointR}}}
}};

//Called every update in op_control. Updates the buttons as they are pressed
void updateSelector() {
	pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
	                  (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
}

/*
 * @brief Computes which auton to select
 *
 * Inputs the value being pressed by the buttons and uses this as an index
 * to access the selectArray.
 *
 * @param      val     Value of button being pressed (0 for left button,
 *                                                    1 for center button,
 *                                                    2 for right button)
 */
void selector(int val) {
  if(yVal) {
    autonToRun = selectArray[*yVal][val];
    pros::lcd::set_text(1, selectArray[*yVal][val].name);
  }
  else {
    yVal = val;
  }
}

//Returns function pointer to be called in autonomous
void (*getSelection()) () {
  //Check if yVal is initialized to avoid a segmentation fault
  if(!yVal)
    return none;
  return autonToRun.pointer;
}

void onLeftButton() {
  selector(0);
}

void onCenterButton() {
  selector(1);
}

void onRightButton() {
  selector(2);
}

//Called in initialize
void initSelector() {
  pros::lcd::initialize();

  pros::lcd::register_btn0_cb(onLeftButton);
  pros::lcd::register_btn1_cb(onCenterButton);
  pros::lcd::register_btn2_cb(onRightButton);
}
