#include "autonselection.hpp"
#include "auton_utils.hpp"
#include "motordefs.hpp"
#include "psa.hpp"
#include "sensors.hpp"

pros::Task *indexTopTask;
pros::Task *indexMidTask;
pros::Task *filterTask;
pros::Task *shootBallsTask;
pros::Task *stopBallsTask;
pros::Task *backUpAndOuttakeTask;
pros::Task *moveForwardAndFilterTask;

ProgrammingSkillsAuton::ProgrammingSkillsAuton(MotorDefs *md, bool ra) {
    mtrDefs = md;
    redAlliance = ra;
    sensors = new Sensors();
    au = new AutonUtils(mtrDefs, sensors);



    indexTopTask = new pros::Task(AutonUtils::indexTop, md);
    indexMidTask = new pros::Task(AutonUtils::indexMid, md);
    filterTask = new pros::Task(AutonUtils::filter, md);
    shootBallsTask = new pros::Task(AutonUtils::shootBalls, md);
    backUpAndOuttakeTask = new pros::Task(AutonUtils::backUpAndOuttake, md);
    moveForwardAndFilterTask = new pros::Task(AutonUtils::moveForwardAndFilter, md);
}

ProgrammingSkillsAuton::~ProgrammingSkillsAuton() {
    delete sensors;
    delete au;
    delete indexTopTask;
    delete indexMidTask;
    delete filterTask;
    delete shootBallsTask;
    delete backUpAndOuttakeTask;
    delete moveForwardAndFilterTask;
}



void ProgrammingSkillsAuton::runAuton(){
    std::cout << "ready" << "\n";
    // Auton movements start here,uncomment the below 6 lines
    captureFirstGoal();
    // captureSecondGoal();
    // captureThirdGoal();
    // captureFourthGoal();
    // captureFifthGoal();
    // captureSixthGoal();
    // captureSeventhGoal();
    // captureEighthGoal();
    // captureNinthGoal();
}

void ProgrammingSkillsAuton::captureFirstGoal() {
    au->twoInTwoOut();
}


void ProgrammingSkillsAuton::startHoldInGoal(){
    au->setDriveVoltage(25, 25);
}

void ProgrammingSkillsAuton::stopHoldInGoal(){
    au->setDriveVoltage(0, 0);
}