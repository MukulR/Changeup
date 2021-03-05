#include "autonselection.hpp"
#include "auton_utils.hpp"
#include "motordefs.hpp"
#include "tga.hpp"
#include "sensors.hpp"

const int TRANSLATE_VOLTAGE = 100;

pros::Task *indexOneTopTask;


ThreeGoalAuton::ThreeGoalAuton(MotorDefs *md, bool ra) {
    mtrDefs = md;
    redAlliance = ra;
    sensors = new Sensors();
    au = new AutonUtils(mtrDefs, sensors);

    indexOneTopTask = new pros::Task(AutonUtils::indexOneTop, md);
}

ThreeGoalAuton::~ThreeGoalAuton() {
    delete sensors;
    delete au;

    delete indexOneTopTask;
}

void ThreeGoalAuton::runAuton() { 
    captureFirstGoal();
    captureSecondGoal();
    captureThirdGoal();
    captureFourthGoal();
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
    au->translate(300, 80, redAlliance ? 90.0 : 270.0, false);
    au->pidGlobalTurn(redAlliance ? 125 : 235);
    au->stopIntakes(mtrDefs);
    au->translate(100, 80, redAlliance ? 125 : 235, false);

    mtrDefs->roller_t->move(-127);
    pros::Task::delay(900);
    au->translate(-600, 127, redAlliance ? 125 : 235, false);
    au->pidGlobalTurn(redAlliance ? 270 : 90);
    mtrDefs->roller_t->move(0);
}

void ThreeGoalAuton::captureSecondGoal() {
    AutonUtils::startIntakes(mtrDefs);
    indexOneTopTask->notify();
    au->translate(redAlliance ? 2000 : 2100, 127, redAlliance ? 270 : 90, true);
    au->pidGlobalTurn(180.0);
    AutonUtils::stopIntakes(mtrDefs);
    au->translate(500, 90, 180.0);
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(900);
    au->translate(-400, 90, 180.0);
    mtrDefs->roller_t->move(0);
    au->pidGlobalTurn(redAlliance ? 270 : 90);
}

void ThreeGoalAuton::captureThirdGoal() {
    AutonUtils::startIntakes(mtrDefs);
    au->translate(2200, 127, redAlliance ? 270 : 90, true);
    au->pidGlobalTurn(redAlliance ? 225 : 135);
    pros::Task::delay(100);
    au->translate(475, 127, -1);
    AutonUtils::indexTopBlocking(mtrDefs);
    AutonUtils::stopIntakes(mtrDefs);
    au->translate(300, 80, -1, true);
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(900);
    au->translate(-600, 127, redAlliance ? 225 : 135);
    au->pidGlobalTurn(redAlliance ? 60 : 300);
}

void ThreeGoalAuton::captureFourthGoal() {
    au->translate(2300, 127, redAlliance ? 65 : 285, true);
    au->pidGlobalTurn(350);
    AutonUtils::startOuttake(mtrDefs);
    au->visionTranslate(1700, 60, false);
}
