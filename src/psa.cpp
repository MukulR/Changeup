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
    pros::Task(AutonUtils::filter, mtrDefs);

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
    au.translate(-1300);
    pros::Task::delay(50);
    // Turn to face the goal, and stop the intakes
    au.globalTurn(135);
    AutonUtils::stopIntakes(mtrDefs);
    pros::Task::delay(50);
    // Advance towards the goal
    au.translate(1975);
    au.assignMotors(30, 30);
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
    au.assignMotors(0, 0);
    au.translate(-500);

    // ---------
    // Get rid of blue ball we picked up
    AutonUtils::startOuttake(mtrDefs);
    pros::Task::delay(200);
    au.globalTurn(270);
    AutonUtils::stopIntakes(mtrDefs);
    pros::Task::delay(50);
    au.translate(2925);
    pros::Task::delay(50);

    // -------------------
    au.globalTurn(180);
    pros::Task::delay(50);
    au.translate(500);
    au.assignMotors(30, 30);
    pros::Task::delay(50);
    AutonUtils::startIntakes(mtrDefs);
    pros::Task::delay(500);
    AutonUtils::stopIntakes(mtrDefs);
    au.oneShot();
    AutonUtils::enableTopIndex();
    AutonUtils::enableMidIndex();
    pros::Task::delay(100);
    au.assignMotors(0, 0);
    // -----------

    au.translate(-400);
    pros::Task::delay(50);

    au.globalTurn(270);
    pros::Task::delay(50);

    // filter ball
    AutonUtils::enableTopIndex();
    AutonUtils::enableMidIndex();
    // Replace below delay with booleans that tell us when indexing is finished.
    pros::Task::delay(500);
    AutonUtils::enableFiltering();
    

    pros::Task::delay(200);

    // start the intake before moving forward
    AutonUtils::startIntakes(mtrDefs);

    // move forward
    au.translate(2700);
    pros::Task::delay(50);
    // stop the intakes, because the balls are in the magazine, and then index the balls
    // to the proper position
    AutonUtils::stopIntakes(mtrDefs);
    AutonUtils::enableTopIndex();
    AutonUtils::enableMidIndex();
    // the balls are indexing to the right spot, turn to face the corner goal
    au.globalTurn(225);
    pros::Task::delay(50);
    // move to the corner goal and hold position there.
    au.translate(600);
    au.assignMotors(30, 30);
    pros::Task::delay(50);
    // shoot the ball, and index the remaining balls into the correct position.
    au.oneShot();
    AutonUtils::enableTopIndex();
    AutonUtils::enableMidIndex();
    pros::Task::delay(100);
    au.assignMotors(0, 0);

    // ---------------------


    AutonUtils::enableTopIndex();
    AutonUtils::enableMidIndex();    

    au.translate(-500);
    pros::Task::delay(50);
    au.globalTurn(0);
    AutonUtils::enableFiltering();
    pros::Task::delay(300);
    AutonUtils::startIntakes(mtrDefs);
    au.translate(2875);
    pros::Task::delay(50);
    au.globalTurn(270);
}

