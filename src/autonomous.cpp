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
    /*
    pros::Vision vision_sensor(10);

    pros::vision_signature_s_t RED_SIG =
    pros::Vision::signature_from_utility(1, 4937, 8847, 6892, -1703, 1, -851, 1.800, 0);

    vision_sensor.set_signature(1, &RED_SIG);

    pros::vision_object_s_t rtn = vision_sensor.get_by_sig(0, 1);
    for (int i = 0; i < 2; i++) {
        std::cout << "LC : " << rtn.left_coord << std::endl;
        std::cout << "TC : " << rtn.top_coord << std::endl;
        std::cout << "W : " << rtn.width << std::endl;
        std::cout << "H : " << rtn.height << std::endl;
        std::cout << "ANGLE : " << rtn.angle << std::endl;
        std::cout << "XMID : " << rtn.x_middle_coord << std::endl;
        std::cout << "YMID : " << rtn.y_middle_coord << std::endl;
        std::cout << std::endl << std::endl;
        rtn = vision_sensor.get_by_sig(0, 1);
        pros::Task::delay(10000);
    }
    */
    MotorDefs* mtrDefs = new MotorDefs();
    ProgrammingSkillsAuton psa(mtrDefs, true);
    psa.runAuton();
} 
