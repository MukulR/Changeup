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
    // au->signatureVisionTranslate(1400, 80, false, true);
    // AutonUtils::startIntakes(mtrDefs);
    // au->visionTranslate(6000, 80, false, true);
    // driveUntilPushed();
    
    if (redAlliance) {
        captureFirstGoal();
        captureSecondGoal();
        captureThirdGoal();
        captureFourthGoal();
    } else {
        captureFirstGoalStates();
        captureSecondGoalStates();
        captureThirdGoalStates();
        captureFourthGoalBackwardStates();
        captureFifthGoalStates();
    }
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
    au->translate(1900, 127, 270, true);
    au->pidGlobalTurn(180.0);
    AutonUtils::stopIntakes(mtrDefs);
    au->translate(650, 90, 180.0);
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(900);
    au->translate(-300, 90, 180.0);
    mtrDefs->roller_t->move(0);
    au->pidGlobalTurn(270);
}

void ThreeGoalAuton::captureThirdGoal() {
    AutonUtils::startIntakes(mtrDefs);
    au->translate(2300, 127, 270, true);
    au->pidGlobalTurn(225);
    pros::Task::delay(100);
    au->translate(425, 127, -1);
    AutonUtils::indexTopBlocking(mtrDefs);
    AutonUtils::stopIntakes(mtrDefs);
    au->translate(300, 80, -1, true);
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(900);
    au->translate(-600, 127, 219, false);
}

void ThreeGoalAuton::captureFourthGoal() {
    au->translate(-2975, 80, 219, true);
    au->setDriveVoltage(2, -100);
    while (sensors->imu->get_heading() < 271) {
        pros::Task::delay(10);
    }
    au->setDriveVoltage(-5, 127);
    pros::Task::delay(1000);
    au->setDriveVoltage(0, 0);
}

void ThreeGoalAuton::captureFirstGoalStates() {
    AutonUtils::startIntakes(mtrDefs);
    au->translate(300, 80, 270.0, false);
    au->pidGlobalTurn(225);
    AutonUtils::stopIntakes(mtrDefs);

    au->translate(100, 80, 225, false);
    AutonUtils::startOuttakeFast(mtrDefs);
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(900);

    au->translate(-1000, 127, 225, true);
    AutonUtils::stopIntakes(mtrDefs);
    au->pidGlobalTurn(0);
}

void ThreeGoalAuton::captureSecondGoalStates() {
    AutonUtils::startIntakes(mtrDefs);
    au->visionTranslate(1550, 80, true, true);
    pros::Task::delay(100);
    indexOneTopTask->notify();
    au->pidGlobalTurn(270);
    pros::Task::delay(50);
    au->translate(-900, 80, 270.0, false);
    pros::Task::delay(300);
    au->setDriveVoltage(-30, 100);
    while (sensors->imu->get_heading() > 163) {
        pros::Task::delay(10);
    }
    au->setDriveVoltage(0, 0);
    AutonUtils::stopIntakes(mtrDefs);
}

void ThreeGoalAuton::captureThirdGoalStates() {
    au->translate(2650, 80, 163.0, false);
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(900);

    au->translate(-250, 80, 180, true);
    pros::Task::delay(100);
}

void ThreeGoalAuton::captureFourthGoalStates() {
    au->pidGlobalTurn(90);
    au->translate(2300, 80, 90, true);
    pros::Task::delay(100);
    au->pidGlobalTurn(135);
    AutonUtils::startIntakes(mtrDefs);
    au->translate(425, 127, -1);
    AutonUtils::indexTopBlocking(mtrDefs);
    AutonUtils::stopIntakes(mtrDefs);
    au->translate(300, 80, -1, false);
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(900);
    au->translate(-600, 127, 135, false);
}

void ThreeGoalAuton::captureFourthGoalBackwardStates() {
    au->pidGlobalTurn(270);
    au->translate(-2350, 80, 270, true);
    pros::Task::delay(100);
    au->pidGlobalTurn(315);
    au->translate(-1000, 80, 315, false);
    pros::Task::delay(100);
    au->translate(500, 80, 310, false);
}

void ThreeGoalAuton::captureFifthGoalStates() {

    au->signatureVisionTranslate(3300, 80, false, true);
    au->setDriveVoltage(80, -15);
    while(sensors->imu->get_heading() < 330) {
        pros::Task::delay(10);
    }
    au->setDriveVoltage(-15, 80);
    pros::Task::delay(50);
    au->setDriveVoltage(0,0);
    pros::Task::delay(200);
}


// void ThreeGoalAuton::driveUntilPushed() {
//     au->setDriveVoltage(80, 80);
//     while(!intake_bumper.get_value()) {
//         pros::Task::delay(10);
//     }
//     indexOneTopTask->notify();
//     au->setDriveVoltage(-70, -70);
//     pros::Task::delay(150);
//     au->setDriveVoltage(0,0);
//     au->pidGlobalTurn(270);
// }