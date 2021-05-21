#include "autonselection.hpp"
#include "auton_utils.hpp"
#include "motordefs.hpp"
#include "lrta.hpp"
#include "sensors.hpp"

const int TRANSLATE_VOLTAGE = 100;

pros::Task *indexOneTopTask;


LRTAuton::LRTAuton(MotorDefs *md, bool ra) {
    mtrDefs = md;
    redAlliance = ra;
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
    // Just home row
    // captureFirstGoal();
    // captureSecondGoal();
    // captureThirdGoal();
    // captureFirstGoal();


    // Home Row & Goal E
    captureFirstGoalHRE();
    captureSecondGoalHRE();
    captureThirdGoalHRE();
    captureFourthGoalHRE();
    captureFifthGoalHRE();


    // Home Row, Goal E & Either Goal B or H Depending on alliance color
    /*
    captureFirstGoalHRBHE();
    captureSecondGoalHRBHE();
    captureThirdGoalHRBHE();
    captureFourthGoalHRBHE();
    captureFifthGoalHRBHE();
    captureSixthGoalHRBHE();
    */
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// -------------------------HOME ROW FUNCTIONS---------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


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
    au->translate(400, 80, -1, true);
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

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// -------------------------HOME ROW + E FUNCTIONS-----------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------



void LRTAuton::captureFirstGoalHRE() {
    AutonUtils::startIntakes(mtrDefs);
    au->translate(300, 80, 270.0, false);
    au->pidGlobalTurn(225);
    AutonUtils::stopIntakes(mtrDefs);

    au->translate(100, 80, 225, false);
    au->setDriveVoltage(20, 20);
    AutonUtils::startOuttakeFast(mtrDefs);
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(900);

    au->translate(-1000, 127, 225, true);
    AutonUtils::stopIntakes(mtrDefs);
    au->pidGlobalTurn(0);
}

void LRTAuton::captureSecondGoalHRE() {
    AutonUtils::startIntakes(mtrDefs);
    au->visionTranslate(1700, 80, false, true);
    pros::Task::delay(100);
    indexOneTopTask->notify();
    au->pidGlobalTurn(270);
    pros::Task::delay(50);
    au->translate(-1100, 80, 270.0, false);
    pros::Task::delay(300);
    au->setDriveVoltage(-30, 100);
    while (sensors->imu->get_heading() > 163) {
        pros::Task::delay(10);
    }
    au->setDriveVoltage(0, 0);
    AutonUtils::stopIntakes(mtrDefs);
}

void LRTAuton::captureThirdGoalHRE() {
    au->translate(2550, 80, 163.0, false);
    au->setDriveVoltage(15, 15);
    pros::Task::delay(200);
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(900);

    au->translate(-250, 80, -1.0, true);
    pros::Task::delay(100);
}

void LRTAuton::captureFourthGoalHRE() {
    au->pidGlobalTurn(270);
    au->translate(-2750, 80, 270, true);
    pros::Task::delay(100);
    au->pidGlobalTurn(315);
    au->translate(-1200, 80, 315, false);
    pros::Task::delay(100);
    au->translate(500, 80, 310, false);
}


void LRTAuton::captureFifthGoalHRE() {
    au->translate(500, 115, 310, false);
    au->signatureVisionTranslate(3150, 80, false, false, redAlliance);
    au->setDriveVoltage(80, -15);
    while(sensors->imu->get_heading() < 315) {
        pros::Task::delay(10);
    }
    mtrDefs->intake_l->move(-10);
    mtrDefs->intake_r->move(10);
    au->setDriveVoltage(-15, 80);
    pros::Task::delay(50);
    au->setDriveVoltage(0,0);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// -------------------------HOME ROW + B/H FUNCTIONS---------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

void LRTAuton::captureFirstGoalHRBH() {
    au->startIntakes(mtrDefs);
    au->translate(300, 80, 90.0, false);
    au->pidGlobalTurn(125);
    au->stopIntakes(mtrDefs);
    au->translate(100, 80, 125, false);

    mtrDefs->roller_t->move(-127);
    pros::Task::delay(900);
    au->translate(-600, 127, 125, false);
    au->pidGlobalTurn(270);
    mtrDefs->roller_t->move(0);
}

void LRTAuton::captureSecondGoalHRBH() {
    indexOneTopTask->notify();
    au->translate(2050, 80, 270, true);
    au->pidGlobalTurn(180.0);
    au->translate(400, 80, 180.0, false);

    au->startIntakes(mtrDefs);
    au->startRollers(mtrDefs);
    pros::Task::delay(900);
    while (!AutonUtils::ballAtTop()) {
        pros::Task::delay(10);
    }    
    while(!AutonUtils::ballAtBottom()) {
        pros::Task::delay(10);
    }
    au->stopIntakes(mtrDefs);
    au->stopRollers(mtrDefs);

    mtrDefs->roller_t->move(-127);
    au->startOuttakeFast(mtrDefs);
    pros::Task::delay(900);
    au->translate(-250, 80, 180, true);
    au->pidGlobalTurn(270.0);
}

void LRTAuton::captureThirdGoalHRBH() {
    AutonUtils::startIntakes(mtrDefs);
    indexOneTopTask->notify();
    au->translate(2500, 80, 270, true);
    au->pidGlobalTurn(225);
    au->translate(800, 80, 225, false);
    au->setDriveVoltage(20, 20);
    AutonUtils::stopIntakes(mtrDefs);
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(900);
    mtrDefs->roller_t->move(0);
    au->translate(-950, 80, 225, true);
    au->pidGlobalTurn(315);
}

void LRTAuton::captureFourthGoalHRBH() {
    au->translate(1550, 80, 335, false);
    AutonUtils::startOuttakeFast(mtrDefs);
    pros::Task::delay(500);
    au->translate(-1000, 80, 335, false);
    pros::Task::delay(300);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// -------------------------HOME ROW + B/H + E FUNCTIONS-----------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

void LRTAuton::captureFirstGoalHRBHE() {
    au->setDriveVoltage(0, 80);
    pros::Task::delay(500);
    au->setDriveVoltage(0,0);
    AutonUtils::startOuttakeFast(mtrDefs);
    au->translate(-500, 80, 225, true);
    au->pidGlobalTurn(0);
}

void LRTAuton::captureSecondGoalHRBHE() {
    AutonUtils::startIntakes(mtrDefs);
    au->visionTranslate(1700, 80, false, true);
    indexOneTopTask->notify();
    au->translate(-650, 80, 45, true);
    au->pidGlobalTurn(315);

    AutonUtils::startOuttakeFast(mtrDefs);
    mtrDefs->roller_b->move(40);
    au->translate(300, 80, 315, false);
    pros::Task::delay(500);
    AutonUtils::stopIntakes(mtrDefs);
    mtrDefs->roller_b->move(-30);
    AutonUtils::startOuttake(mtrDefs);

    au->translate(-300, 80, 315, false);

}

void LRTAuton::captureThirdGoalHRBHE() {
    au->setDriveVoltage(-60, 20);
    while(sensors->imu->get_heading() > 253) {
        pros::Task::delay(10);
    }
    mtrDefs->roller_b->move(0);

    au->translate(-1500, 80, 253, false);
    pros::Task::delay(600);
}

void LRTAuton::captureFourthGoalHRBHE() {
    au->setDriveVoltage(-30, 100);
    while (sensors->imu->get_heading() > 166) {
        pros::Task::delay(10);
    }
    au->setDriveVoltage(0, 0);

    indexOneTopTask->notify();
    au->translate(2450, 110, 166.0, false);
    au->setDriveVoltage(20, 20);
    pros::Task::delay(200);
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(500);

    au->translate(-250, 80, 180, true);
    pros::Task::delay(100);
}

void LRTAuton::captureFifthGoalHRBHE() {
    au->pidGlobalTurn(270);
    if (redAlliance) {
        au->translate(-2500, 115, 270, false);
    } else {
        au->translate(-2650, 115, 270, false);
    }
    au->pidGlobalTurn(315);
    au->translate(-1000, 100, 315, false);
}

void LRTAuton::captureSixthGoalHRBHE() {
    au->translate(500, 115, 310, false);
    au->signatureVisionTranslate(3350, 80, false, false, redAlliance);
    au->setDriveVoltage(80, -15);
    while(sensors->imu->get_heading() < 315) {
        pros::Task::delay(10);
    }
    au->setDriveVoltage(-15, 80);
    pros::Task::delay(50);
    au->setDriveVoltage(0,0);
    pros::Task::delay(200);
}