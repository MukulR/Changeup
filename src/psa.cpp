#include "auton_utils.hpp"
#include "motordefs.hpp"
#include "psa.hpp"
#include "sensors.hpp"

void ProgrammingSkillsAuton::threeShotSequence() {
    // Hold power once in the goal
    startHoldInGoal();
    pros::Task::delay(100);
    // Shoot the top red ball
    au->oneShot();

    // Give the ball a short burst of power to hit the bottom roller
    AutonUtils::startIntakes(mtrDefs);
    pros::Task::delay(100);
    AutonUtils::stopIntakes(mtrDefs);
    au->indexTop();
    au->indexMid();
    // Turn on the intakes to remove the bottom blue ball from the tower
    AutonUtils::startIntakes(mtrDefs);
    pros::Task::delay(300);
    AutonUtils::stopIntakes(mtrDefs);

    // Release the hold power
    stopHoldInGoal();
}

//value 1493 no ball | 200 barely in | 200 all in

void ProgrammingSkillsAuton::twoShotSequence(bool darkGoal) {
    // Hold power once in the goal
    startHoldInGoal();
    pros::Task::delay(100);
    // Give the ball a short burst of power to hit the bottom roller
    AutonUtils::startIntakes(mtrDefs);
    au->waitUntilIntaked(darkGoal);
    AutonUtils::stopIntakes(mtrDefs);

    // Shoot the top red ball
    au->oneShot();
    

    // Backup so that the ball is relieved from the goal plate
    au->assignMotors(-30, -30);
    pros::Task::delay(500);
    au->assignMotors(0, 0);

    // Run the intake so that the ball makes it on to ramp
    // AutonUtils::startIntakes(mtrDefs);
    // pros::Task::delay(300);
    // AutonUtils::stopIntakes(mtrDefs);

    // Index the two remaining red balls into the correct position, to make space
    // for the next blue ball we intake from the goal
    au->indexTop();
    AutonUtils::startIntakes(mtrDefs);
    pros::Task::delay(30);
    AutonUtils::stopIntakes(mtrDefs);
    au->indexMid();
    
    // Release the hold power
    stopHoldInGoal();
}

void ProgrammingSkillsAuton::oneShotSequence(bool slowShot) {
    // Hold power once in the goal
    startHoldInGoal();
    pros::Task::delay(100);

    // Give the ball a short burst of power to hit the bottom roller
    AutonUtils::startIntakes(mtrDefs);
    au->waitUntilIntaked(false);
    pros::Task::delay(200);
    AutonUtils::stopIntakes(mtrDefs);

    // Shoot the top red ball
    if (slowShot) {
        au->slowOneShot();
    } else {
        au->oneShot();
    }

    // Backup so that the ball is relieved from the goal plate
    au->assignMotors(-30, -30);
    pros::Task::delay(300);
    au->assignMotors(0, 0);
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

    // pros::Task(AutonUtils::index, mtrDefs);
    // pros::Task(AutonUtils::indexTop, mtrDefs);
    // pros::Task(AutonUtils::indexMid, mtrDefs);
    // pros::Task(AutonUtils::filter, mtrDefs);

    // Auton movements start here,uncomment the below 6 lines
    captureFirstGoal();
    captureSecondGoal();
    captureThirdGoal();
    captureFourthGoal();
    captureFifthGoal();
    captureSixthGoal();
    captureSeventhGoal();
    captureEighthGoal();
    captureNinthGoal();
}

void ProgrammingSkillsAuton::captureFirstGoal(){
    
    // Start indexing first two balls
    AutonUtils::startIntakes(mtrDefs);
    au->indexTop();
    // Going to the goal --------------------------------------------------

    au->translate(1250);
    pros::Task::delay(50);
    au->indexMid();
    // Turn towards fence
    au->globalTurn(90);
    pros::Task::delay(50);
    au->translate(1250);
    pros::Task::delay(50);
    // The ball is now collected, return backwards
    au->translate(-950);
    pros::Task::delay(50);
    // Turn to face the goal, and stop the intakes
    au->globalTurn(135);
    AutonUtils::stopIntakes(mtrDefs);
    pros::Task::delay(50);
    // Advance towards the goal
    au->translate(1775);
    threeShotSequence();
    // Backup from the goal
    au->translate(-400);
    // Get rid of blue ball we picked up
    AutonUtils::startOuttake(mtrDefs);
    pros::Task::delay(400);
    // Turn to get ready for the next goal
    au->globalTurn(270);
    pros::Task::delay(50);
    AutonUtils::stopIntakes(mtrDefs);
}

void ProgrammingSkillsAuton::captureSecondGoal(){
    // Move towards second goal from the left corner goal to the right center goal
    au->translate(2700);
    pros::Task::delay(50);

    // -------------------
    // Face the goal and move forward
    au->globalTurn(180);
    pros::Task::delay(50);
    au->translate(400);
    // two balls in robot
    twoShotSequence();
    // -----------

    // Backout from the goal
    au->translate(-100);
    pros::Task::delay(50);

    // Face right heading for the next goal
    au->globalTurn(270);
    pros::Task::delay(50);
}

void ProgrammingSkillsAuton::captureThirdGoal() {
    std::cout << "In captureThirdGoal" << std::endl;
    AutonUtils::startIntakes(mtrDefs);
    au->indexTop();
    au->indexMid();
    
    // Filter out blue ball, wait until filtered, then index the emaining balls.
    au->filter();
    // move forward to corner goal
    au->translate(2200);
    // Index the ball we pick up while moving to the next corner goal.
    au->indexTop();
    au->indexMid();
    // stop the intakes, because the balls are loaded, and then index the balls to the proper position
    AutonUtils::stopIntakes(mtrDefs);
    // the balls are indexing to the right spot, turn to face the corner goal
    au->globalTurn(225);
    pros::Task::delay(50);
    // move to the corner goal and hold position there.
    au->translate(700);
    // Shoot while 2 red balls in robot, dark goal is true.
    twoShotSequence(true);
    // ---------------------
    
    // Backout from the goal
    au->translate(-250);
    pros::Task::delay(50);
    // Face correct heading for the next goal.
    au->turnRightToZeroHeading();
    // Filter out the blue ball that we picked up earlier.
    au->filter();
}

void ProgrammingSkillsAuton::captureFourthGoal() {
    AutonUtils::startIntakes(mtrDefs);
    au->translate(2460);
    pros::Task::delay(1000);
    AutonUtils::stopIntakes(mtrDefs);
    pros::Task::delay(50);

    // Index the ball we pick up while moving to the next corner goal.
    au->indexTop();
    au->indexMid();

    // -------------------
    // Face the goal and move forward
    au->globalTurn(270);
    pros::Task::delay(50);
    au->translate(300);
    // two balls in robot, dark goal
    twoShotSequence(true);
    // -----------

    // Backout from the goal
    
    pros::Task::delay(50);

    // Face right heading for the next goal
    au->turnRightToZeroHeading();
    pros::Task::delay(50);
    // Filter out the blue ball that we extract from the tower.
    au->filter();
}

void ProgrammingSkillsAuton::captureFifthGoal() {
    // move forward to corner goal
    au->translate(1650);
    pros::Task::delay(50);
    // Re-index the already indexed ball just in case
    au->indexTop();
    // Turn to face the corner goal
    au->globalTurn(328);
    // Go to the corner goal
    au->translate(1250);
    // Shoot the ball and intake the blue ball
    oneShotSequence(true);
    // Back out of the corner goal
    au->translate(-400);
    // Out take the blue ball that we got from the tower
    AutonUtils::startIntakes(mtrDefs);
    mtrDefs->roller_b->move(-127);
    mtrDefs->roller_t->move(127);
    // Turn to get ready for the next goal
    au->globalTurn(90);
    pros::Task::delay(500);
    AutonUtils::stopIntakes(mtrDefs);
    AutonUtils::stopRollers(mtrDefs);
}

void ProgrammingSkillsAuton::captureSixthGoal() {
    AutonUtils::startIntakes(mtrDefs);
    au->translate(2750);
    AutonUtils::stopIntakes(mtrDefs);

    // Index the ball we pick up while moving to the next corner goal.
    au->indexTop();
    au->turnLeftToZeroHeading();

    // Go to the goal
    au->translate(450);
    // Shoot the ball and intake the blue ball
    oneShotSequence(false);
    // Back out of the corner goal
    au->translate(-185);
    // Outtake the blue ball that we got from the tower
    AutonUtils::startIntakes(mtrDefs);
    mtrDefs->roller_b->move(-127);
    mtrDefs->roller_t->move(127);
    // Turn to get ready for the next goal
    au->globalTurn(90);
    pros::Task::delay(50);
    AutonUtils::stopIntakes(mtrDefs);
    AutonUtils::stopRollers(mtrDefs);
}

void ProgrammingSkillsAuton::captureSeventhGoal() {
    AutonUtils::startIntakes(mtrDefs);
    au->translate(2250);
    pros::Task::delay(50);
    // Re-index the already indexed ball just in case
    au->indexTop();
    // Turn to face the corner goal
    au->globalTurn(45);
    // Go to the corner goal
    au->translate(700);
    // Shoot the ball and intake the blue ball
    oneShotSequence(true);
    // Back out of the corner goal
    au->translate(-425);
    // Out take the blue ball that we got from the tower
    AutonUtils::startIntakes(mtrDefs);
    mtrDefs->roller_b->move(-127);
    mtrDefs->roller_t->move(127);
    // Turn to get ready for the next goal
    au->globalTurn(180);
    pros::Task::delay(500);
    AutonUtils::stopIntakes(mtrDefs);
    AutonUtils::stopRollers(mtrDefs);
}

void ProgrammingSkillsAuton::captureEighthGoal() {
    AutonUtils::startIntakes(mtrDefs);
    au->translate(2550);
    pros::Task::delay(500);
    AutonUtils::stopIntakes(mtrDefs);

    // Index the ball we pick up while moving to the next corner goal.
    au->indexTop();
    au->globalTurn(90);

    // Go to the goal
    au->translate(400);
    // Shoot the ball and intake the blue ball
    oneShotSequence(false);
    // Back out of the corner goal
    au->translate(-145);
    // Out take the blue ball that we got from the tower
    AutonUtils::startOuttake(mtrDefs);
    pros::Task::delay(400);
    // Turn to get ready for the next goal
    au->globalTurn(267);
    pros::Task::delay(50);
    AutonUtils::stopIntakes(mtrDefs);
}

void ProgrammingSkillsAuton::captureNinthGoal() {
    // Intake the ball and get close to the middle goal
    AutonUtils::startIntakes(mtrDefs);
    au->translate(1800);
    // Slowly drive forward with intakes going slowly to lock into tower
    au->indexTop();
    mtrDefs->intake_l->move(30);
    mtrDefs->intake_r->move(-30);
    au->assignMotors(40, 40);

    pros::Task::delay(1000);
    startHoldInGoal();
    mtrDefs->intake_l->move(0);
    mtrDefs->intake_r->move(0);

    mtrDefs->intake_l->move(-127);
    mtrDefs->intake_r->move(127);
    pros::Task::delay(500);
    au->oneShot();
    pros::Task::delay(500);
    AutonUtils::startOuttake(mtrDefs);
    au->assignMotors(-40, -40);
    
}

void ProgrammingSkillsAuton::startHoldInGoal(){
    au->assignMotors(25, 25);
}

void ProgrammingSkillsAuton::stopHoldInGoal(){
    au->assignMotors(0, 0);
}