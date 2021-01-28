#include <iostream>
#include "autonselection.hpp"
#include "main.h"
#include "okapi/api.hpp"
#include "motordefs.hpp"
#include "psa.hpp"
#include "tga.hpp"
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
    MotorDefs* mtrDefs = new MotorDefs();
    switch (autonSelected) {
        case 0:
            {
                ProgrammingSkillsAuton psa(mtrDefs, true);
                psa.runAuton();
            }
            break;
        case 1:
            {
                ThreeGoalAuton tga(mtrDefs, true);
                tga.runAuton();
            }
            break;
        default:
            noAuton();
            break;
    }    
} 
