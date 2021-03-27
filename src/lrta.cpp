#include "autonselection.hpp"
#include "auton_utils.hpp"
#include "motordefs.hpp"
#include "lrta.hpp"
#include "sensors.hpp"

const int TRANSLATE_VOLTAGE = 100;

pros::Task *indexOneTopTask;


LRTAuton::LRTAuton(MotorDefs *md, bool ra, bool fg) {
    mtrDefs = md;
    redAlliance = ra;
    fiveGoal = fg;
    sensors = new Sensors();
    au = new AutonUtils(mtrDefs, sensors);

    indexOneTopTask = new pros::Task(AutonUtils::indexOneTop, md);
}

LRTAuton::~LRTAuton() {
    delete sensors;
    delete au;

    delete indexOneTopTask;
}

void LRTAuton::runAuton() {
    // au->signatureVisionTranslate(1400, 80, false, redAlliance);
    // AutonUtils::startIntakes(mtrDefs);
    // au->visionTranslate(8000, 80, false, true);
    // pros::Task::delay(100);
    // au->pidGlobalTurn(270);
    // au->visionTranslate(0, 80, false, true);
    // driveUntilPushed();
    // std::cout << fiveGoal << std::endl;
    if (fiveGoal) {
        captureFirstGoalStates();
        captureSecondGoalStates();
        captureThirdGoalStates();
        captureFourthGoalBackwardStates();
        captureFifthGoalStates();
    } else {
        captureFirstGoal();
        captureSecondGoal();
        captureThirdGoal();
        captureFourthGoal();
    }
}

void LRTAuton::captureFirstGoal() {
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

void LRTAuton::captureSecondGoal() {
    AutonUtils::startIntakes(mtrDefs);
    indexOneTopTask->notify();
    au->translate(2050, 80, 270, true);
    au->pidGlobalTurn(180.0);
    AutonUtils::stopIntakes(mtrDefs);
    au->translate(400, 80, 180.0, false);
    mtrDefs->roller_t->move(-100);
    pros::Task::delay(900);
    au->translate(-300, 60, 180.0);
    mtrDefs->roller_t->move(0);
    au->pidGlobalTurn(270);
}

void LRTAuton::captureThirdGoal() {
    AutonUtils::startIntakes(mtrDefs);
    au->translate(2550, 80, 270, true);
    au->pidGlobalTurn(225);
    pros::Task::delay(100);
    au->translate(300, 80, -1, true);
    AutonUtils::indexTopBlocking(mtrDefs);
    AutonUtils::stopIntakes(mtrDefs);
    au->translate(300, 80, -1, true);
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(900);
    au->translate(-600, 80, 219, false);
}

void LRTAuton::captureFourthGoal() {
    au->translate(-3050, 80, 219, true);
    au->setDriveVoltage(2, -100);
    while (sensors->imu->get_heading() < 271) {
        pros::Task::delay(10);
    }
    au->setDriveVoltage(-5, 127);
    pros::Task::delay(1000);
    au->setDriveVoltage(0, 0);
}

void LRTAuton::captureFirstGoalStates() {
    AutonUtils::startIntakes(mtrDefs);
    au->translate(300, 80, 270.0, false);
    au->pidGlobalTurn(225);
    AutonUtils::stopIntakes(mtrDefs);

    au->translate(200, 80, 225, false);
    au->setDriveVoltage(20, 20);
    AutonUtils::startOuttakeFast(mtrDefs);
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(900);

    au->translate(-1000, 127, 225, true);
    AutonUtils::stopIntakes(mtrDefs);
    au->pidGlobalTurn(0);
}

void LRTAuton::captureSecondGoalStates() {
    AutonUtils::startIntakes(mtrDefs);
    au->visionTranslate(1700, 80, false, true);
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

void LRTAuton::captureThirdGoalStates() {
    au->translate(2650, 80, 163.0, false);
    au->setDriveVoltage(20, 20);
    pros::Task::delay(200);
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(900);

    au->translate(-250, 80, 180, true);
    pros::Task::delay(100);
}

void LRTAuton::captureFourthGoalStates() {
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

void LRTAuton::captureFourthGoalBackwardStates() {
    au->pidGlobalTurn(270);
    au->translate(-2575, 80, 270, true);
    pros::Task::delay(100);
    au->pidGlobalTurn(315);
    au->translate(-1200, 80, 315, false);
    pros::Task::delay(100);
    au->translate(500, 80, 310, false);
}

void LRTAuton::captureFifthGoalStates() {

    au->signatureVisionTranslate(3350, 80, false, redAlliance);
    au->setDriveVoltage(80, -15);
    while(sensors->imu->get_heading() < 330) {
        pros::Task::delay(10);
    }
    au->setDriveVoltage(-15, 80);
    pros::Task::delay(50);
    au->setDriveVoltage(0,0);
    pros::Task::delay(200);
}


// void LRTAuton::driveUntilPushed() {
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