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
    //captureThirdGoal();
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
    au->startIntakes(mtrDefs);
    au->translate(300, 80, 90.0, false);
    au->pidGlobalTurn(125);
    au->stopIntakes(mtrDefs);
    au->translate(100, 80, 125, false);

    mtrDefs->roller_t->move(-127);
    pros::Task::delay(900);
    au->translate(-500, 127, 125, false);
    au->pidGlobalTurn(300);
    mtrDefs->roller_t->move(0);
}

void ThreeGoalAuton::captureSecondGoal() {
    // indexTwoBallsTask->notify();
    // au->visionTranslate(2000, 120, false);
    // au->pidGlobalTurn(180.0);
    // au->translate(2000, 127, 180.0, false);

    // mtrDefs->roller_t->move(-127);
    // pros::Task::delay(1000);

    // au->translate(-100, 127, 180);
    // au->pidGlobalTurn(270.0);
}

void ThreeGoalAuton::captureThirdGoal() {
    // indexTwoBallsTask->notify();
    // au->visionTranslate(2000, 120, false);

    // mtrDefs->roller_t->move(-127);
    // mtrDefs->intake_l->move(127);
    // mtrDefs->intake_r->move(-127);
    // pros::Task::delay(1000);

    // au->translate(-500, 127);
}