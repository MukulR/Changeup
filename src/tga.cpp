#include "autonselection.hpp"
#include "auton_utils.hpp"
#include "motordefs.hpp"
#include "tga.hpp"
#include "sensors.hpp"

pros::Task *filterTsk;
pros::Task *shootBallsTsk;
pros::Task *stopBallsTsk;
pros::Task *indexMidTsk;
pros::Task *indexTwoBallsTsk;
pros::Task *filterAndIndexTwoBallsTsk;
pros::Task *filterAndIndexOneBallTsk;

const int TRANSLATE_VOLTAGE = 100;

ThreeGoalAuton::ThreeGoalAuton(MotorDefs *md, bool ra) {
    mtrDefs = md;
    redAlliance = ra;
    sensors = new Sensors();
    au = new AutonUtils(mtrDefs, sensors);


    filterTsk = new pros::Task(AutonUtils::filter, md);
    shootBallsTsk = new pros::Task(AutonUtils::shootBalls, md);
    indexMidTsk = new pros::Task(AutonUtils::indexMid, md);
    indexTwoBallsTsk = new pros::Task(AutonUtils::indexTwoBalls, md);
    filterAndIndexTwoBallsTsk = new pros::Task(AutonUtils::filterAndIndexTwoBalls, md);
    filterAndIndexOneBallTsk = new pros::Task(AutonUtils::filterAndIndexOneBall, md);
}

ThreeGoalAuton::~ThreeGoalAuton() {
    delete sensors;
    delete au;

    delete filterTsk;
    delete shootBallsTsk;
    delete indexMidTsk;
    delete indexTwoBallsTsk;
    delete filterAndIndexTwoBallsTsk;
    delete filterAndIndexOneBallTsk;
}

void ThreeGoalAuton::runAuton() { 
    // captureFirstGoal();
    // captureSecondGoal();
    // captureThirdGoal();
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(3000);
    mtrDefs->roller_t->move(0);
}

void ThreeGoalAuton::captureFirstGoal() {
    // Start indexing to pickup balls when going forward
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(100);
    mtrDefs->roller_t->move(0);
    AutonUtils::startIntakes(mtrDefs);

    // Move to the fence ball 
    indexTwoBallsTsk->notify();
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
    // Turn to face middle ball's heading. 
    au->pidGlobalTurn(290);
}

void ThreeGoalAuton::captureSecondGoal() {
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexOneBallTsk->notify();
    // Advance towards the middle ball
    au->visionTranslate(3050, 80, false);

    au->pidGlobalTurn(180);
    AutonUtils::stopIntakes(mtrDefs);
    // Go to goal
    au->translate(1850, 115, 183.0);

    au->nonCornerGoalSequence(-200, 180.0);

    au->pidGlobalTurn(270);    
}

void ThreeGoalAuton::captureThirdGoal() {
    AutonUtils::startIntakes(mtrDefs);
    filterAndIndexTwoBallsTsk->notify();
    // Advance towards the ball
    au->visionTranslate(3710, 80, false);
    pros::Task::delay(100);
    // Translate now so we do not pickup the other red ball w/ vision
    au->translate(-550, TRANSLATE_VOLTAGE, 245.0);
    // Turn to goal
    au->pidGlobalTurn(225);
    // Go to goal
    indexTwoBallsTsk->notify();
    au->translate(1000, TRANSLATE_VOLTAGE, 315.0);
    // Score/remove balls
    au->cornerGoalSequence();
}