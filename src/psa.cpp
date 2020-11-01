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
    au.translate(-1200);
    pros::Task::delay(50);
    // Turn to face the goal, and stop the intakes
    au.globalTurn(135);
    AutonUtils::stopIntakes(mtrDefs);
    pros::Task::delay(50);
    // Advance towards the goal
    au.translate(1975);
    pros::Task::delay(50);
    // Shoot one out of 3 red balls
    au.oneShot();
    // Start indexing the 2 red balls left and 1 blue ball.
    AutonUtils::enableTopIndex();
    AutonUtils::enableMidIndex();
    pros::Task::delay(1000);
    AutonUtils::startIntakes(mtrDefs);
    pros::Task::delay(1000);
    AutonUtils::stopIntakes(mtrDefs);
    pros::Task::delay(50);
    au.translate(-500);

    // ---------
    // Get rid of blue ball we picked up
    AutonUtils::startOuttake(mtrDefs);
    pros::Task::delay(200);
    au.globalTurn(270);
    AutonUtils::stopIntakes(mtrDefs);
    pros::Task::delay(50);
    au.translate(2750);
    pros::Task::delay(50);
    au.globalTurn(180);
    pros::Task::delay(50);
    au.translate(500);
    pros::Task::delay(50);
    au.doubleShot();
    AutonUtils::startIntakes(mtrDefs);
    pros::Task::delay(1000);
    AutonUtils::stopIntakes(mtrDefs);
    // -----------

    au.translate(-300);
    pros::Task::delay(50);

    au.globalTurn(270);
    pros::Task::delay(50);

    // filter ball
    AutonUtils::startIntakes(mtrDefs);
    pros::Task filtering(AutonUtils::filter, mtrDefs);

    au.translate(2800);
    AutonUtils::enableTopIndex();
    pros::Task::delay(50);
    au.globalTurn(225);
    AutonUtils::stopIntakes(mtrDefs);
    au.translate(600);
    pros::Task::delay(50);
    au.oneShot();
}

