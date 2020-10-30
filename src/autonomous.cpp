#include <iostream>

#include "main.h"
#include "okapi/api.hpp"
#include "motordefs.hpp"
#include "psa.hpp"
#include "auton.hpp"

using std::endl;



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

void noAuton(){}

void autonomous() {
    std::cout << "hi" << endl;
    MotorDefs mtrDefs;
    ProgrammingSkillsAuton psa(&mtrDefs, true);
    psa.runAuton();
    // mtrDefs.left_mtr_t->move(70);
    // mtrDefs.left_mtr_b->move(70);
    // mtrDefs.right_mtr_t->move(70);
    // mtrDefs.right_mtr_b->move(70);
} 
