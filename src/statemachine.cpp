#include "statemachine.hpp"

class stateMachine stateMachine;

bool forceQuit_t = false;

void TugOfWar() {
    int startingPos = tracking.get_value();
    while(tracking.get_value() - startingPos < 200 && !forceQuit_t) {
        pros::delay(100);
    }

    while(forceQuit_t)
        pros::delay(100);
}

void test() {

}

pros::Task tOW(test, "tmp");

void initTOW() {
    pros::Task tOW_t(TugOfWar, "ToW");
    tOW = tOW_t;
}

void stopTOW() {
    forceQuit_t = true;
}