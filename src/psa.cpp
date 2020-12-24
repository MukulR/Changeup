#include "autonselection.hpp"
#include "auton_utils.hpp"
#include "motordefs.hpp"
#include "psa.hpp"
#include "sensors.hpp"

pros::Task *filterTask;
pros::Task *shootBallsTask;
pros::Task *stopBallsTask;
pros::Task *indexMidTask;
pros::Task *indexTwoBallsTask;
pros::Task *filterAndIndexTwoBallsTask;
pros::Task *filterAndIndexOneBallTask;


const int TRANSLATE_VOLTAGE = 100;

ProgrammingSkillsAuton::ProgrammingSkillsAuton(MotorDefs *md, bool ra) {
    mtrDefs = md;
    redAlliance = ra;
    sensors = new Sensors();
    au = new AutonUtils(mtrDefs, sensors);


    filterTask = new pros::Task(AutonUtils::filter, md);
    shootBallsTask = new pros::Task(AutonUtils::shootBalls, md);
    indexMidTask = new pros::Task(AutonUtils::indexMid, md);
    indexTwoBallsTask = new pros::Task(AutonUtils::indexTwoBalls, md);
    filterAndIndexTwoBallsTask = new pros::Task(AutonUtils::filterAndIndexTwoBalls, md);
    filterAndIndexOneBallTask = new pros::Task(AutonUtils::filterAndIndexOneBall, md);
}

ProgrammingSkillsAuton::~ProgrammingSkillsAuton() {
    delete sensors;
    delete au;

    delete filterTask;
    delete shootBallsTask;
    delete indexMidTask;
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
    //au->twoInOneOut(-400, 0.0);
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
    au->visionTranslate(3050, 80, false);
    pros::Task::delay(100);

    // Move back from fence 
    au->translate(-400, TRANSLATE_VOLTAGE);
    // Turn towards the goal and stop intakes
    au->pidGlobalTurn(135);
    AutonUtils::stopIntakes(mtrDefs);
    // Go to goal
    au->translate(900, TRANSLATE_VOLTAGE);
    pros::Task::delay(50);
    // Score in goals, extract blue balls, and back up.
    au->cornerGoalSequence(true);
    // Turn to 0 heading. 
    au->pidGlobalTurn(335);
    // au->pidGlobalTurn(0);
}

void ProgrammingSkillsAuton::captureSecondGoal() {
    // Start intakes to pickup the ball, and start the task
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexTwoBallsTask->notify();
    // Advance to pickup the ball next to center goal
    au->translate(1300, 80, 335.0);
    au->visionTranslate(1525, 100, true);
    
    // Turn to face the goal
    au->pidGlobalTurn(90);
    // Track and pickup the second ball
    au->visionTranslate(1600, 50, false);
    // move farther into the goal so that we can score
    au->translate(350, 50, 90.0);
    pros::Task::delay(100);
    // Process balls in the 2nd goal
    au->twoInOneOut(-500, 90.0);
    pros::Task::delay(100);
    // Turn to face the next goals
    au->pidGlobalTurn(0);
}

void ProgrammingSkillsAuton::captureThirdGoal() {
    AutonUtils::setIndexingOneBall(true);
    // Start intakes to pickup the ball, and start the task
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexOneBallTask->notify();
    // Advance towards the third goal
    au->visionTranslate(2500, 80, true);
    pros::Task::delay(50);

    au->pidGlobalTurn(90);
    AutonUtils::stopIntakes(mtrDefs);
    pros::Task::delay(50);
    au->translate(500, TRANSLATE_VOLTAGE, 90.0);
    pros::Task::delay(50);
    au->pidGlobalTurn(45);
    au->translate(750, TRANSLATE_VOLTAGE, 45.0);
    pros::Task::delay(50);

    au->cornerGoalSequence(false);

    au->pidGlobalTurn(245);
    // au->filter();
}

void ProgrammingSkillsAuton::captureFourthGoal() {
    // Start intakes to pickup the ball, and start the task
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexOneBallTask->notify();
    // Advance towards the middle ball
    au->visionTranslate(3150, 80, false);
    // Turn to face goal
    au->pidGlobalTurn(0);
    AutonUtils::stopIntakes(mtrDefs);
    // Go to goal
    au->translate(1800, TRANSLATE_VOLTAGE, 0.0);

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
    au->visionTranslate(2000, 80, false);
    // Translate now so we do not pickup the other red ball w/ vision
    au->translate(500, TRANSLATE_VOLTAGE, 270.0);
    // Turn to goal
    au->pidGlobalTurn(315);
    // Go to goal
    au->translate(1000, TRANSLATE_VOLTAGE, 315.0);
    // Score/remove balls
    au->cornerGoalSequence(false);
    // Face next goal
    au->pidGlobalTurn(145);
    // Filter out ball
    // au->filter();
}

void ProgrammingSkillsAuton::captureCenterGoal() {
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexOneBallTask->notify();
    // Advance towards the ball
    au->visionTranslate(2825, 80, true);
    pros::Task::delay(200);
    // Turn to goal
    au->pidGlobalTurn(110);
    pros::Task::delay(100);
    // Go to goal with intakes slowly running
    AutonUtils::startIntakes(mtrDefs);
    au->translate(1050, 60, 110.0);
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
    au->visionTranslate(1500, 50, false);
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
    au->visionTranslate(2500, 80, false);

    au->pidGlobalTurn(270);
    pros::Task::delay(50);
    au->translate(575, TRANSLATE_VOLTAGE, 270.0);
    pros::Task::delay(50);
    au->pidGlobalTurn(225);
    au->translate(750, TRANSLATE_VOLTAGE, 225.0);
    pros::Task::delay(50);

    au->cornerGoalSequence(false);
    pros::Task::delay(100);
    // Turn to face the next goals
    au->pidGlobalTurn(70);
}

void ProgrammingSkillsAuton::reviseNinthGoal() {
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexOneBallTask->notify();
    // Advance towards the middle ball
    au->visionTranslate(3050, 80, false);

    au->pidGlobalTurn(183);
    AutonUtils::stopIntakes(mtrDefs);
    // Go to goal
    au->translate(1850, TRANSLATE_VOLTAGE, 183.0);

    au->nonCornerGoalSequence(-200, 180.0);
}

void ProgrammingSkillsAuton::startHoldInGoal(){
    au->setDriveVoltage(25, 25);
}

void ProgrammingSkillsAuton::stopHoldInGoal(){
    au->setDriveVoltage(0, 0);
}