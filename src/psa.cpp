#include "auton_utils.hpp"
#include "motordefs.hpp"
#include "psa.hpp"
#include "sensors.hpp"

ProgrammingSkillsAuton::ProgrammingSkillsAuton(MotorDefs *md, bool ra){
    mtrDefs = md;
    redAlliance = ra;
}

ProgrammingSkillsAuton::~ProgrammingSkillsAuton() {

}

void ProgrammingSkillsAuton::runAuton(){
    Sensors sensors;
    AutonUtils au(mtrDefs, &sensors);

    au.translate(4000);
    pros::Task::delay(100);
    au.rotate(-90, 50);
    pros::Task::delay(100);
    au.translate(4000);

    /*
    au.translate(4000);
    pros::Task::delay(100);
    au.rotate(90, 50);
    pros::Task::delay(100);

    au.translate(4000);
    pros::Task::delay(100);
    au.rotate(90, 50);
    pros::Task::delay(100);

    au.translate(4000);
    pros::Task::delay(100);
    au.rotate(90, 50);
    */
}

