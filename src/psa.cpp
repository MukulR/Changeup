#include "autonselection.hpp"
#include "auton_utils.hpp"
#include "motordefs.hpp"
#include "psa.hpp"
#include "sensors.hpp"

pros::Task *filterTask;
pros::Task *shootBallsTask;
pros::Task *stopBallsTask;
pros::Task *backUpAndOuttakeTask;
pros::Task *moveForwardAndFilterTask;
pros::Task *indexTwoBallsTask;
pros::Task *filterAndIndexTwoBallsTask;
pros::Task *filterAndIndexOneBallTask;


const int TRANSLATE_VOLTAGE = 80;

ProgrammingSkillsAuton::ProgrammingSkillsAuton(MotorDefs *md, bool ra) {
    mtrDefs = md;
    redAlliance = ra;
    sensors = new Sensors();
    au = new AutonUtils(mtrDefs, sensors);


    filterTask = new pros::Task(AutonUtils::filter, md);
    shootBallsTask = new pros::Task(AutonUtils::shootBalls, md);
    backUpAndOuttakeTask = new pros::Task(AutonUtils::backUpAndOuttake, md);
    moveForwardAndFilterTask = new pros::Task(AutonUtils::moveForwardAndFilter, md);
    indexTwoBallsTask = new pros::Task(AutonUtils::indexTwoBalls, md);
    filterAndIndexTwoBallsTask = new pros::Task(AutonUtils::filterAndIndexTwoBalls, md);
    filterAndIndexOneBallTask = new pros::Task(AutonUtils::filterAndIndexOneBall, md);
}

ProgrammingSkillsAuton::~ProgrammingSkillsAuton() {
    delete sensors;
    delete au;

    delete filterTask;
    delete shootBallsTask;
    delete backUpAndOuttakeTask;
    delete moveForwardAndFilterTask;
    delete indexTwoBallsTask;
    delete filterAndIndexTwoBallsTask;
    delete filterAndIndexOneBallTask;
}


void getInfo(Sensors *sensors) {
    while (true) {
        std::cout << sensors->distance_l->get() << " " << sensors->distance_l->get_confidence() << " " << sensors->distance_l->get_object_velocity()
        << " | " << sensors->distance_r->get() << " " << sensors->distance_r->get_confidence() << " "  << sensors->distance_r->get_object_velocity() << std::endl;
        pros::Task::delay(50);
    }
}



void ProgrammingSkillsAuton::runAuton() {
    // au->visionTranslate(1550, 80);
    // au->cornerGoalSequence();
    captureFirstGoal();
    captureSecondGoal();
    captureThirdGoal();
    captureFourthGoal();
    captureFifthGoal();
    captureCenterGoal();
    captureSeventhGoal();
    captureEighthGoal();
    reviseNinthGoal();
}

void ProgrammingSkillsAuton::captureFirstGoal() {
    // Start indexing to pickup balls when going forward
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(100);
    mtrDefs->roller_t->move(0);
    AutonUtils::startIntakes(mtrDefs);

    // Move to the fence ball 
    indexTwoBallsTask->notify();
    au->visionTranslate(3000, 80);
    pros::Task::delay(100);

    // Move back from fence and stop intakes
    au->translate(-400, TRANSLATE_VOLTAGE);
    AutonUtils::stopIntakes(mtrDefs);
    // Turn towards the goal
    au->pidGlobalTurn(135);
    // Go to goal
    au->translate(900, TRANSLATE_VOLTAGE);
    pros::Task::delay(50);
    // Score in goals, extract blue balls, and back up.
    au->cornerGoalSequence();
    // Turn to 0 heading. 
    au->pidGlobalTurn(0);

    // au->filter();
}

void ProgrammingSkillsAuton::captureSecondGoal() {
    // Start intakes to pickup the ball, and start the task
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexOneBallTask->notify();
    // Advance towards the second goal
    au->visionTranslate(2350, 80);
    pros::Task::delay(200);
    // Turn to face the goal
    au->pidGlobalTurn(90);
    AutonUtils::stopIntakes(mtrDefs);
    // Advance to the goal
    au->translate(400, TRANSLATE_VOLTAGE);
    // Process balls in the 2nd goal
    au->nonCornerGoalSequence(-500, 90.0);
    pros::Task::delay(100);
    // Turn to face the next goals
    au->pidGlobalTurn(0);
    //Filter out blue ball
    // au->filter();
}

void ProgrammingSkillsAuton::captureThirdGoal() {
    AutonUtils::setIndexingOneBall(true);
    // Start intakes to pickup the ball, and start the task
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexOneBallTask->notify();
    // Advance towards the third goal
    au->visionTranslate(2500, 80);
    pros::Task::delay(50);

    au->pidGlobalTurn(90);
    AutonUtils::stopIntakes(mtrDefs);
    pros::Task::delay(50);
    au->translate(500, TRANSLATE_VOLTAGE, 90.0);
    pros::Task::delay(50);
    au->pidGlobalTurn(45);
    au->translate(700, TRANSLATE_VOLTAGE, 45.0);
    pros::Task::delay(50);

    au->cornerGoalSequence();

    au->pidGlobalTurn(245);
    // au->filter();
}

void ProgrammingSkillsAuton::captureFourthGoal() {
    // Start intakes to pickup the ball, and start the task
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexOneBallTask->notify();
    // Advance towards the middle ball
    au->visionTranslate(3200, 80);
    // Turn to face goal
    au->pidGlobalTurn(0);
    AutonUtils::stopIntakes(mtrDefs);
    // Go to goal
    au->translate(1700, TRANSLATE_VOLTAGE, 0.0);

    au->nonCornerGoalSequence(-200, 0.0);
    // Turn to face next goal
    au->pidGlobalTurn(270);
    // Filter out blue ball
    // au->filter();
}

void ProgrammingSkillsAuton::captureFifthGoal() {
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexOneBallTask->notify();
    // Advance towards the ball
    au->visionTranslate(2000, 80);
    // Translate now so we do not pickup the other red ball w/ vision
    au->translate(500, TRANSLATE_VOLTAGE, 270.0);
    // Turn to goal
    au->pidGlobalTurn(315);
    // Go to goal
    au->translate(900, TRANSLATE_VOLTAGE, 315.0);
    // Score/remove balls
    au->cornerGoalSequence();
    // Face next goal
    au->pidGlobalTurn(145);
    // Filter out ball
    // au->filter();
}

void ProgrammingSkillsAuton::captureCenterGoal() {
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexOneBallTask->notify();
    // Advance towards the ball
    au->visionTranslate(2900, 80);
    pros::Task::delay(200);
    // Turn to goal
    au->pidGlobalTurn(90);
    // Go to goal with intakes slowly running
    AutonUtils::startIntakes(mtrDefs);
    au->translate(1100, 60, 90.0);
    // Turn intake to full speed to outtake the balls
    au->centerSequence();
    // Turn to face next goal
    au->pidGlobalTurn(270);
}

void ProgrammingSkillsAuton::captureSeventhGoal() {
    // Start intakes to pickup the ball, and start the task
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexOneBallTask->notify();
    // Advance towards the second goal
    au->visionTranslate(1500, 50);
    au->translate(350, 50, 270.0);
    pros::Task::delay(500);
    AutonUtils::stopIntakes(mtrDefs);
    // Process balls in the 7th goal
    au->nonCornerGoalSequence(-500, 270.0);
    pros::Task::delay(100);
    // Turn to face the next goals
    au->pidGlobalTurn(180);
    //Filter out blue ball
    // au->filter();
}

void ProgrammingSkillsAuton::captureEighthGoal() {
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexOneBallTask->notify();
    // Advance towards the third goal
    au->visionTranslate(2500, 80);

    au->pidGlobalTurn(270);
    pros::Task::delay(50);
    au->translate(500, TRANSLATE_VOLTAGE, 270.0);
    pros::Task::delay(50);
    au->pidGlobalTurn(225);
    au->translate(700, TRANSLATE_VOLTAGE, 225.0);
    pros::Task::delay(50);

    au->cornerGoalSequence();
    pros::Task::delay(100);
    // Turn to face the next goals
    au->pidGlobalTurn(70);
}

void ProgrammingSkillsAuton::reviseNinthGoal() {
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexOneBallTask->notify();
    // Advance towards the middle ball
    au->visionTranslate(3200, 80);

    au->pidGlobalTurn(180);
    AutonUtils::stopIntakes(mtrDefs);
    // Go to goal
    au->translate(1550, TRANSLATE_VOLTAGE, 180.0);

    au->nonCornerGoalSequence(-200, 180.0);
}

void ProgrammingSkillsAuton::startHoldInGoal(){
    au->setDriveVoltage(25, 25);
}

void ProgrammingSkillsAuton::stopHoldInGoal(){
    au->setDriveVoltage(0, 0);
}