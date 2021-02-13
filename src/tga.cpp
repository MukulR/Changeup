#include "autonselection.hpp"
#include "auton_utils.hpp"
#include "motordefs.hpp"
#include "tga.hpp"
#include "sensors.hpp"

const int TRANSLATE_VOLTAGE = 100;

ThreeGoalAuton::ThreeGoalAuton(MotorDefs *md, bool ra) {
    mtrDefs = md;
    redAlliance = ra;
    sensors = new Sensors();
    au = new AutonUtils(mtrDefs, sensors);
}

ThreeGoalAuton::~ThreeGoalAuton() {
    delete sensors;
    delete au;
}

void ThreeGoalAuton::runAuton() { 
    captureFirstGoal();
    captureSecondGoal();
}

void indexTwoBallsSync(MotorDefs *mtrDefs, AutonUtils *au) {
    mtrDefs->roller_t->move(-80);
    mtrDefs->roller_b->move(-80);
    while(!AutonUtils::ballAtTop()) {
        
    }
    mtrDefs->roller_t->move(-10);

    au->setDriveVoltage(-40, -40);
    pros::Task::delay(300);
    au->setDriveVoltage(40, 40);
    pros::Task::delay(300);
    au->setDriveVoltage(0, 0);

    while(!AutonUtils::ballAtMid()) {

    }
    mtrDefs->roller_b->move(0);
    mtrDefs->roller_t->move(0);
}

void doubleShot(MotorDefs *mtrDefs) {
    // mtrDefs->intake_r->move(60);
    // mtrDefs->intake_l->move(-60);
    // pros::Task::delay(200);
    // mtrDefs->intake_r->move(0);
    // mtrDefs->intake_l->move(0);

    mtrDefs->roller_b->move(80);
    pros::Task::delay(200);
    mtrDefs->roller_b->move(15);
    
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(300);
    mtrDefs->roller_t->move(0);

    pros::Task::delay(50);

    mtrDefs->roller_b->move(-127);
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(300);
    mtrDefs->roller_b->move(0);
    pros::Task::delay(600);
    mtrDefs->roller_t->move(0);
}

void ThreeGoalAuton::captureFirstGoal() {
    // Start indexing to pickup balls when going forward
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(200);
    mtrDefs->roller_t->move(0);
    au->translate(1800, TRANSLATE_VOLTAGE);
    pros::Task::delay(100);

    // Turn towards the goal and stop intakes
    std::cout << redAlliance << std::endl;
    redAlliance ? au->pidGlobalTurn(135) : au->pidGlobalTurn(225);  
    // Go to goal while using a task
    AutonUtils::startIntakes(mtrDefs);
    au->translate(975, TRANSLATE_VOLTAGE);
    indexTwoBallsSync(mtrDefs, au);
    AutonUtils::stopIntakes(mtrDefs);
    pros::Task::delay(50);
    // Score in goals, and back up.
    doubleShot(mtrDefs);
    pros::Task::delay(200);
    //AutonUtils::startIntakes(mtrDefs);
    au->translate(-800, TRANSLATE_VOLTAGE);
    pros::Task::delay(100);
    //AutonUtils::stopIntakes(mtrDefs);
    // Turn to face next goal's heading. 
    redAlliance ? au->pidGlobalTurn(270) : au->pidGlobalTurn(93);  
}

void ThreeGoalAuton::captureSecondGoal() {
    mtrDefs->roller_t->move(127);
    mtrDefs->roller_b->move(-127);
    // Advance towards the middle ball
    AutonUtils::startIntakes(mtrDefs);
    redAlliance ? au->translate(4750, 80, 270) : au->translate(4800, 80, 93);  
    AutonUtils::stopIntakes(mtrDefs);

    redAlliance ? au->pidGlobalTurn(225) : au->pidGlobalTurn(135);  
    mtrDefs->roller_t->move(0);
    mtrDefs->roller_b->move(0);

    AutonUtils::startIntakes(mtrDefs);
    au->translate(1000, TRANSLATE_VOLTAGE);
    indexTwoBallsSync(mtrDefs, au);
    AutonUtils::stopIntakes(mtrDefs);
    pros::Task::delay(50);
    // Score in goals, and back up.
    doubleShot(mtrDefs);
    pros::Task::delay(200);
    //AutonUtils::startIntakes(mtrDefs);
    au->translate(-1000, TRANSLATE_VOLTAGE);
    //AutonUtils::stopIntakes(mtrDefs);
}