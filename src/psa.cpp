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

    //pros::Task(AutonUtils::index, mtrDefs);
    pros::Task(AutonUtils::indexTop, mtrDefs);
    pros::Task(AutonUtils::indexMid, mtrDefs);

    AutonUtils::enableTopIndex();
    AutonUtils::enableMidIndex();
    
    // Go forward from starting position
    au.translate(1700);
    pros::Task::delay(50);
    // Turn towards fence
    au.globalTurn(90);
    pros::Task::delay(50);
    // Start intakes and advance towards the ball on the fence
    AutonUtils::startIntakes(mtrDefs);
    au.translate(1550);
    pros::Task::delay(50);
    // The ball is now collected, return backwards
    au.translate(-1450);
    pros::Task::delay(50);
    // Turn to face the goal, and stop the intakes
    au.globalTurn(135);
    AutonUtils::stopIntakes(mtrDefs);
    pros::Task::delay(50);
    // Advance towards the goal
    au.translate(1800);
    pros::Task::delay(50);
    // Shoot one out of 3 red balls
    au.oneShot();
}

