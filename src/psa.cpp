#include "auton_utils.hpp"
#include "motordefs.hpp"
#include "psa.hpp"
#include "sensors.hpp"

void ProgrammingSkillsAuton::oneShotSequence(bool intakeAfterIndex, bool corner) {
    // Hold power once in the goal
    startHoldInGoal();
    pros::Task::delay(100);
    // Shoot the top red ball
    au->oneShot();

    
    if (!intakeAfterIndex) {
        // Turn on the intakes to remove the bottom blue ball from the tower
        AutonUtils::startIntakes(mtrDefs);
        if (corner) {
            pros::Task::delay(900);
        } else {
            pros::Task::delay(400);
        }
        AutonUtils::stopIntakes(mtrDefs);
    } else {
        // Give the ball a short burst of power to hit the bottom roller
        AutonUtils::startIntakes(mtrDefs);
        pros::Task::delay(50);
        AutonUtils::stopIntakes(mtrDefs);
    }

    // Index the two remaining red balls into the correct position, to make space
    // for the next blue ball we intake from the goal

    AutonUtils::enableTopIndex();
    AutonUtils::enableMidIndex();
    // Wait until indexed fully before extracting the blue ball.
    AutonUtils::waitUntilTopIndexed();
    AutonUtils::waitUntilMidIndexed();
    if (intakeAfterIndex) {
        // Turn on the intakes to remove the bottom blue ball from the tower
        AutonUtils::startIntakes(mtrDefs);
        pros::Task::delay(300);
        AutonUtils::stopIntakes(mtrDefs);
    }

    // Release the hold power
    stopHoldInGoal();
}


ProgrammingSkillsAuton::ProgrammingSkillsAuton(MotorDefs *md, bool ra){
    mtrDefs = md;
    redAlliance = ra;
    sensors = new Sensors();
    au = new AutonUtils(mtrDefs, sensors);
}

ProgrammingSkillsAuton::~ProgrammingSkillsAuton() {
    delete sensors;
    delete au;
}

void ProgrammingSkillsAuton::runAuton(){
    std::cout << "ready" << "\n";

    //pros::Task(AutonUtils::index, mtrDefs);
    pros::Task(AutonUtils::indexTop, mtrDefs);
    pros::Task(AutonUtils::indexMid, mtrDefs);
    pros::Task(AutonUtils::filter, mtrDefs);

    // Auton movements start here
    captureFirstGoal();
    
    left_corner_right_center();
    left_center_right_corner();
}

void ProgrammingSkillsAuton::captureFirstGoal(){
    
    // Start indexing first two balls
    AutonUtils::startIntakes(mtrDefs);
    AutonUtils::enableTopIndex();
    AutonUtils::enableMidIndex();

    
    // Going to the goal --------------------------------------------------

    au->translate(1700);
    pros::Task::delay(50);
    // Turn towards fence
    au->globalTurn(90);
    pros::Task::delay(50);
    au->translate(1550);
    pros::Task::delay(50);
    // The ball is now collected, return backwards
    au->translate(-1300);
    pros::Task::delay(50);
    // Turn to face the goal, and stop the intakes
    au->globalTurn(135);
    AutonUtils::stopIntakes(mtrDefs);
    pros::Task::delay(50);
    // Advance towards the goal
    au->translate(1975);
    oneShotSequence(true, /* Intake after indexing */ false);
    // Backup from the goal
    au->translate(-500);
    // Get rid of blue ball we picked up
    AutonUtils::startOuttake(mtrDefs);
    pros::Task::delay(400);
    // Turn to get ready for the next goal
    au->globalTurn(270);
    pros::Task::delay(50);
    AutonUtils::stopIntakes(mtrDefs);
}

void ProgrammingSkillsAuton::left_corner_right_center(){
    // Move towards second goal from the left corner goal to the right center goal
    au->translate(2875);
    pros::Task::delay(50);

    // -------------------
    // Face the goal and move forward
    au->globalTurn(180);
    pros::Task::delay(50);
    au->translate(500);
    // One shot sequence
    oneShotSequence(false, /* Intake before indexing */ false);
    // -----------

    // Backout from the goal
    au->translate(-400);
    pros::Task::delay(50);

    // Face right heading for the next goal
    au->globalTurn(270);
    pros::Task::delay(50);
}

void ProgrammingSkillsAuton::left_center_right_corner() {
    AutonUtils::startIntakes(mtrDefs);
    AutonUtils::enableTopIndex();
    AutonUtils::enableMidIndex();
    // Filter out blue ball, wait until filtered, then index the emaining balls.
    AutonUtils::enableFiltering();
    // move forward to corner goal
    au->translate(2700);
    // Index the ball we pick up while moving to the next corner goal.
    AutonUtils::enableTopIndex();
    AutonUtils::enableMidIndex();
    AutonUtils::waitUntilTopIndexed();
    AutonUtils::waitUntilMidIndexed();
    // stop the intakes, because the balls are loaded, and then index the balls to the proper position
    AutonUtils::stopIntakes(mtrDefs);
    // the balls are indexing to the right spot, turn to face the corner goal
    au->globalTurn(225);
    pros::Task::delay(50);
    // move to the corner goal and hold position there.
    au->translate(900);
    oneShotSequence(false, /* Intake after indexing */ true);
    // ---------------------
    
    // Backout from the goal
    au->translate(-500);
    pros::Task::delay(50);
    // Face correct heading for the next goal.
    au->globalTurn(0);
    // Filter out the blue ball that we picked up earlier.
    AutonUtils::enableFiltering();
}

void ProgrammingSkillsAuton::startHoldInGoal(){
    au->assignMotors(25, 25);
}

void ProgrammingSkillsAuton::stopHoldInGoal(){
    au->assignMotors(0, 0);
}