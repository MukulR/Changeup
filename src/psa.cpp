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
    std::cout << "ready" << "\n";

    pros::delay(5000);
    au.globalTurn(90.0);
    pros::delay(5000);
    au.globalTurn(0.0);
    pros::delay(5000);
    au.globalTurn(180.0);
    pros::delay(5000);
    au.globalTurn(0.0);
    pros::delay(5000);
    au.globalTurn(270.0);
    pros::delay(5000);
    au.globalTurn(0.0);
}

