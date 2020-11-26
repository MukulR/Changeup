#include "autonselection.hpp"
#include "auton_utils.hpp"
#include "motordefs.hpp"
#include "psa.hpp"
#include "sensors.hpp"

pros::Task *indexTopTask;
pros::Task *indexMidTask;
pros::Task *filterTask;
pros::Task *shootBallsTask;

void ProgrammingSkillsAuton::threeShotSequence() {
    // Hold power once in the goal
    startHoldInGoal();
    pros::Task::delay(100);
    // Shoot the top red ball
    au->oneShot();

    // Give the ball a short burst of power to hit the bottom roller
    AutonUtils::startIntakes(mtrDefs);
    pros::Task::delay(100);
    AutonUtils::stopIntakes(mtrDefs);
    au->indexTop();
    au->indexMid();
    // Turn on the intakes to remove the bottom blue ball from the tower
    AutonUtils::startIntakes(mtrDefs);
    pros::Task::delay(300);
    AutonUtils::stopIntakes(mtrDefs);

    // Release the hold power
    stopHoldInGoal();
}

//value 1493 no ball | 200 barely in | 200 all in

void ProgrammingSkillsAuton::twoShotSequence(bool darkGoal) {
    // Hold power once in the goal
    startHoldInGoal();
    pros::Task::delay(100);
    // Give the ball a short burst of power to hit the bottom roller
    AutonUtils::startIntakes(mtrDefs);
    au->waitUntilIntaked(darkGoal);
    AutonUtils::stopIntakes(mtrDefs);

    // Shoot the top red ball
    au->oneShot();
    

    // Backup so that the ball is relieved from the goal plate
    au->assignMotors(-30, -30);
    pros::Task::delay(500);
    au->assignMotors(0, 0);

    // Run the intake so that the ball makes it on to ramp
    // AutonUtils::startIntakes(mtrDefs);
    // pros::Task::delay(300);
    // AutonUtils::stopIntakes(mtrDefs);

    // Index the two remaining red balls into the correct position, to make space
    // for the next blue ball we intake from the goal
    au->indexTop();
    AutonUtils::startIntakes(mtrDefs);
    pros::Task::delay(30);
    AutonUtils::stopIntakes(mtrDefs);
    au->indexMid();
    
    // Release the hold power
    stopHoldInGoal();
}

void ProgrammingSkillsAuton::thirdGoalSequence() {
    // Hold power once in the goal
    startHoldInGoal();
    pros::Task::delay(100);

    // Shoot the top red ball
    au->oneShot();

     // Index the remaining red ball into the top position
    au->indexTop();

     // Give the ball a short burst of power to hit the bottom roller
    AutonUtils::startIntakes(mtrDefs);

    // Release the hold power
    stopHoldInGoal();

    // Backup so that the ball is relieved from the goal plate
    au->assignMotors(-30, -30);
    pros::Task::delay(500);
    au->assignMotors(0, 0);
   


    // Backout from the goal
    // Turn on the intakes to remove the bottom blue balls from the robot
    
    au->translate(-450);

    mtrDefs->roller_b->move(127);
    AutonUtils::startOuttake(mtrDefs);

    pros::Task::delay(100);
    mtrDefs->roller_b->move(0);
    // Face correct heading for the next goal.
    au->turnRightToZeroHeading();
    AutonUtils::stopIntakes(mtrDefs);
    
}

void ProgrammingSkillsAuton::oneShotSequence(bool slowShot) {
    // Hold power once in the goal
    startHoldInGoal();
    pros::Task::delay(100);

    // Give the ball a short burst of power to hit the bottom roller
    AutonUtils::startIntakes(mtrDefs);
    au->waitUntilIntaked(false);
    pros::Task::delay(200);
    AutonUtils::stopIntakes(mtrDefs);

    // Shoot the top red ball
    if (slowShot) {
        au->slowOneShot();
    } else {
        au->oneShot();
    }

    // Backup so that the ball is relieved from the goal plate
    au->assignMotors(-30, -30);
    pros::Task::delay(300);
    au->assignMotors(0, 0);
    stopHoldInGoal();
}

ProgrammingSkillsAuton::ProgrammingSkillsAuton(MotorDefs *md, bool ra) {
    mtrDefs = md;
    redAlliance = ra;
    sensors = new Sensors();
    au = new AutonUtils(mtrDefs, sensors);



    indexTopTask = new pros::Task(AutonUtils::indexTop, md);
    indexMidTask = new pros::Task(AutonUtils::indexMid, md);
    filterTask = new pros::Task(AutonUtils::filter, md);
    shootBallsTask = new pros::Task(AutonUtils::shootBalls, md);
}

ProgrammingSkillsAuton::~ProgrammingSkillsAuton() {
    delete sensors;
    delete au;
    delete indexTopTask;
    delete indexMidTask;
    delete filterTask;
    delete shootBallsTask;
}

void ProgrammingSkillsAuton::runAuton(){
    std::cout << "ready" << "\n";

    // pros::Task(AutonUtils::index, mtrDefs);
    // pros::Task(AutonUtils::indexTop, mtrDefs);
    // pros::Task(AutonUtils::indexMid, mtrDefs);
    // pros::Task(AutonUtils::filter, mtrDefs);

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

    // au->pidGlobalTurn(0.0);
    // pros::Task::delay(2000);
    // au->translate(3000, 0.0);
    // au->translate(1500, 10.0);


    // thirdGoalSequence();
}

void ProgrammingSkillsAuton::captureFirstGoal() {
    // au->visionTranslate(2000, 80);
    // pros::Task::delay(2000);
    // indexTopTask->notify();
    // pros::Task::delay(5000);
    // indexTopTask->suspend();
    // pros::Task::delay(2000);
    // indexTopTask->notify();
    au->twoInTwoOut();
}


void ProgrammingSkillsAuton::startHoldInGoal(){
    au->assignMotors(25, 25);
}

void ProgrammingSkillsAuton::stopHoldInGoal(){
    au->assignMotors(0, 0);
}