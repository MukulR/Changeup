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

    // Auton movements start here
    captureFirstGoal(au);
    left_corner_right_center(au);
    left_center_right_corner(au);

    // for(int i = 0; i < 3; i++) {
    //     left_corner_right_center(au);
    //     left_center_right_corner(au);
    // }
    // left_corner_right_center(au);
}

void ProgrammingSkillsAuton::captureFirstGoal(AutonUtils au){
    // Start indexing first two balls
    AutonUtils::enableTopIndex();
    AutonUtils::enableMidIndex();
    
    // Going to the goal --------------------------------------------------

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
    startHoldInGoal(au);
    pros::Task::delay(50);

    // Scoring --------------------------------------------------------------------

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
    stopHoldInGoal(au);
    // Backup from the goal
    au.translate(-500);
    // Get rid of blue ball we picked up
    AutonUtils::startOuttake(mtrDefs);
    pros::Task::delay(400);
    // Turn to get ready for the next goal
    au.globalTurn(270);
    AutonUtils::stopIntakes(mtrDefs);
    pros::Task::delay(50);
}

void ProgrammingSkillsAuton::left_corner_right_center(AutonUtils au){
    // Move towards second goal from the left corner goal to the right center goal
    au.translate(2925);
    pros::Task::delay(50);

    // -------------------
    // Face the goal and move forward
    au.globalTurn(180);
    pros::Task::delay(50);
    au.translate(500);
    // Hold power once in the goal
    startHoldInGoal(au);
    pros::Task::delay(50);
    AutonUtils::startIntakes(mtrDefs);
    pros::Task::delay(500);
    AutonUtils::stopIntakes(mtrDefs);
    // After intaking blue ball, one shot
    au.oneShot();
    // Index the balls after scoring
    AutonUtils::enableTopIndex();
    AutonUtils::enableMidIndex();
    pros::Task::delay(100);
    stopHoldInGoal(au);
    // -----------

    // Backout from the goal
    au.translate(-400);
    pros::Task::delay(50);

    // Face right heading for the next goal
    au.globalTurn(270);
    pros::Task::delay(50);
}

void ProgrammingSkillsAuton::left_center_right_corner(AutonUtils au) {
    // Index the balls to prepare for filtering
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
    // stop the intakes, because the balls are loaded, and then index the balls to the proper position
    AutonUtils::stopIntakes(mtrDefs);
    AutonUtils::enableTopIndex();
    AutonUtils::enableMidIndex();
    // the balls are indexing to the right spot, turn to face the corner goal
    au.globalTurn(225);
    pros::Task::delay(50);
    // move to the corner goal and hold position there.
    au.translate(600);
    startHoldInGoal(au);
    pros::Task::delay(50);
    // shoot the ball, and index the remaining balls into the correct position.
    au.oneShot();
    AutonUtils::enableTopIndex();
    AutonUtils::enableMidIndex();
    pros::Task::delay(100);
    stopHoldInGoal(au);

    // ---------------------
    
    // Backout from the goal
    au.translate(-500);
    pros::Task::delay(50);
    // Face correct heading for the next goal.
    au.globalTurn(0);
    // Filter out the blue ball that we picked up earlier.
    AutonUtils::enableFiltering();
    pros::Task::delay(300);
}

void ProgrammingSkillsAuton::startHoldInGoal(AutonUtils au){
    au.assignMotors(25, 25);
}

void ProgrammingSkillsAuton::stopHoldInGoal(AutonUtils au){
    au.assignMotors(0, 0);
}