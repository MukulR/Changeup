#include "autonselection.hpp"
#include "auton_utils.hpp"
#include "motordefs.hpp"
#include "psa.hpp"
#include "sensors.hpp"

pros::Task *filterTask;
pros::Task *shootBallsTask;
pros::Task *stopBallsTask;
pros::Task *indexMidTask;
pros::Task *filterAndIndexMidTask;
pros::Task *indexTwoBallsTask;
pros::Task *filterAndIndexTwoBallsTask;
pros::Task *filterAndIndexOneBallTask;


const int TRANSLATE_VOLTAGE = 100;

const int FIRST_GOAL = 1;
const int THIRD_GOAL = 3;
const int FIFTH_GOAL = 5;
const int EIGTH_GOAL = 8;

ProgrammingSkillsAuton::ProgrammingSkillsAuton(MotorDefs *md, bool ra) {
    mtrDefs = md;
    redAlliance = ra;
    sensors = new Sensors();
    au = new AutonUtils(mtrDefs, sensors);


    filterTask = new pros::Task(AutonUtils::filter, md);
    shootBallsTask = new pros::Task(AutonUtils::shootBalls, md);
    indexMidTask = new pros::Task(AutonUtils::indexMid, md);
    filterAndIndexMidTask = new pros::Task(AutonUtils::filterAndIndexMid, md);
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
    delete filterAndIndexMidTask;
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
    captureFirstGoal();
    captureSecondGoal();
    captureThirdOrEigthGoal(THIRD_GOAL);
    captureFourthGoal();
    captureFifthGoal();
    captureCenterGoal();
    captureSeventhGoal();
    captureThirdOrEigthGoal(EIGTH_GOAL);
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
    au->cornerGoalSequence();
    // Turn to 0 heading. 
    au->pidGlobalTurn(335);
    // au->pidGlobalTurn(0);
}

void ProgrammingSkillsAuton::captureSecondGoal() {
    // Start intakes to pickup the ball, and start the task
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexTwoBallsTask->notify();
    // Advance to pickup the ball next to center goal
    au->translate(1400, 100, 325.0, false);
    au->visionTranslate(1525, 80, false);
    
    // Turn to face the goal
    au->pidGlobalTurn(90);
    // Track and pickup the second ball
    au->visionTranslate(1650, 80, false);
    // move farther into the goal so that we can score
    indexTwoBallsTask->notify();
    au->translate(500, 50, 90.0);
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
    indexTwoBallsTask->notify();
    au->translate(750, TRANSLATE_VOLTAGE, 45.0);
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
    au->visionTranslate(3150, 110, false);
    // Turn to face goal
    au->pidGlobalTurn(0);
    AutonUtils::stopIntakes(mtrDefs);
    // Go to goal
    au->translate(1800, 127, 0.0);

    au->nonCornerGoalSequence(-200, 0.0);
    // Turn to face next goal
    au->pidGlobalTurn(270);
    // Filter out blue ball
    // au->filter();
}

void ProgrammingSkillsAuton::captureFifthGoal() {
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexTwoBallsTask->notify();
    // Advance towards the ball
    au->visionTranslate(3710, 80, false);
    pros::Task::delay(100);
    // Translate now so we do not pickup the other red ball w/ vision
    au->translate(-550, TRANSLATE_VOLTAGE, 245.0);
    // Turn to goal
    au->pidGlobalTurn(315);
    // Go to goal
    indexTwoBallsTask->notify();
    au->translate(1000, TRANSLATE_VOLTAGE, 315.0);
    // Score/remove balls
    au->cornerGoalSequence();
    // Face next goal
    au->pidGlobalTurn(145);
    // Filter out ball
    // au->filter();
}

void ProgrammingSkillsAuton::captureCenterGoal() {
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexMidTask->notify();
    // Advance towards the ball
    au->translate(300, 80, 140.0, false);
    au->visionTranslate(2825, 100, true);
    pros::Task::delay(200);
    // Turn to goal
    au->pidGlobalTurn(100);
    pros::Task::delay(100);
    // Go to goal with intakes slowly running
    AutonUtils::startIntakes(mtrDefs);
    au->setDriveVoltage(80, 80);
    pros::Task::delay(750);
    au->setDriveVoltage(0, 0);
    // Turn intake to full speed to outtake the balls
    au->centerSequence();
    // Turn to face next goal
    au->pidGlobalTurn(270);
}

void ProgrammingSkillsAuton::captureSeventhGoal() {
    // Start intakes to pickup the ball, and start the task
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexOneBallTask->notify();
    // Advance towards the seventh goal
    au->visionTranslate(1500, 80, false);
    au->translate(500, 50, 270.0);
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

    au->cornerGoalSequence();
    pros::Task::delay(100);
    // Turn to face the next goals
    au->pidGlobalTurn(70);
}

void ProgrammingSkillsAuton::reviseNinthGoal() {
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexOneBallTask->notify();
    // Advance towards the middle ball
    au->visionTranslate(3050, 127, false);

    au->pidGlobalTurn(183);
    AutonUtils::stopIntakes(mtrDefs);
    // Go to goal
    au->translate(1850, 127, 183.0);

    au->nonCornerGoalSequence(-200, 180.0);
}

void ProgrammingSkillsAuton::captureThirdOrEigthGoal(int goalNumber) {
    // Start intakes to pickup the ball, and start the task
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexTwoBallsTask->notify();
    // Advance towards the third or eigth goal
    au->visionTranslate(2500, 100, true);
    pros::Task::delay(50);
    // Turn based on parameter
    switch (goalNumber) {
        case THIRD_GOAL:
            au->pidGlobalTurn(105);
            break;
        case EIGTH_GOAL:
            au->pidGlobalTurn(285);
            break;
        default:
            break;
    }
    // Track and pick up ball against the fence
    au->visionTranslate(1400, 90, false);
    pros::Task::delay(100);

     // Move back from fence 
     switch (goalNumber) {
        case THIRD_GOAL:
            au->translate(-450, TRANSLATE_VOLTAGE, 105.0);
            break;
        case EIGTH_GOAL:
            au->translate(-500, TRANSLATE_VOLTAGE, 285.0);
            break;
        default:
            break;
    }
    // Turn towards the goal and stop intakes
    switch (goalNumber) {
        case THIRD_GOAL:
            au->pidGlobalTurn(45);
            break;
        case EIGTH_GOAL:
            au->pidGlobalTurn(225);
            break;
        default:
            break;
    }
    AutonUtils::stopIntakes(mtrDefs);
    indexTwoBallsTask->notify();
    // Go to goal
    au->translate(900, TRANSLATE_VOLTAGE);
    pros::Task::delay(50);
    // Score in goals, extract blue balls, and back up.
    au->cornerGoalSequence();
    // Turn to next ball heading.
    switch (goalNumber) {
        case THIRD_GOAL:
            au->pidGlobalTurn(245);
            break;
        case EIGTH_GOAL:
            au->pidGlobalTurn(70);
            break;
        default:
            break;
    }
}

void ProgrammingSkillsAuton::startHoldInGoal(){
    au->setDriveVoltage(25, 25);
}

void ProgrammingSkillsAuton::stopHoldInGoal(){
    au->setDriveVoltage(0, 0);
}