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
pros::Task *indexTask;

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
    indexTask = new pros::Task(AutonUtils::index, md);
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
    delete indexTask;
}

void getInfo(Sensors *sensors) {
    while (true) {
        std::cout << sensors->distance_l->get() << " " << sensors->distance_l->get_confidence() << " " << sensors->distance_l->get_object_velocity()
        << " | " << sensors->distance_r->get() << " " << sensors->distance_r->get_confidence() << " "  << sensors->distance_r->get_object_velocity() << std::endl;
        pros::Task::delay(50);
    }
}



void ProgrammingSkillsAuton::runAuton(){
    // Auton movements start here,uncomment the below 6 lines
    // getInfo(sensors);
    au->translateWithDS();
    // captureFirstGoal();
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
    AutonUtils::setIndexingOneBall(false);
    // Start indexing to pickup balls when going forward
    mtrDefs->roller_t->move(-127);
    pros::Task::delay(20);
    mtrDefs->roller_t->move(0);
    AutonUtils::startIntakes(mtrDefs);

    // Move to the fence ball 
    indexTask->notify();
    au->visionTranslate(3000, 80);
    pros::Task::delay(100);

    // Move back from fence and stop intakes
    au->translate(-450);
    AutonUtils::stopIntakes(mtrDefs);
    // Turn towards the goal
    au->pidGlobalTurn(135);
    // Go to goal
    au->translate(900);
    pros::Task::delay(50);
    // Score in goals, extract blue balls, and back up.
    au->cornerGoalSequence();
    // Turn to 0 heading. 
    au->pidGlobalTurn(0);

    au->filter();;
}

void ProgrammingSkillsAuton::captureSecondGoal() {
    // Tell indexing task that we are planning to index only one ball to the top.
    AutonUtils::setIndexingOneBall(true);
    // Start intakes to pickup the ball, and start the task
    AutonUtils::startIntakes(mtrDefs);
    indexTask->notify();
    // Advance towards the second goal
    au->visionTranslate(2100, 80);
    pros::Task::delay(200);
    // Turn to face the goal
    au->pidGlobalTurn(90);
    AutonUtils::stopIntakes(mtrDefs);
    // Advance to the goal
    au->translate(400);
    // Process balls in the 2nd goal
    au->nonCornerGoalSequence();
    // Turn to face the next goals
    au->pidGlobalTurn(355);
    //Filter out blue ball
    au->filter();
}

void ProgrammingSkillsAuton::captureThirdGoal() {
    AutonUtils::setIndexingOneBall(true);
    // Start intakes to pickup the ball, and start the task
    AutonUtils::startIntakes(mtrDefs);
    indexTask->notify();
    // Advance towards the second goal
    au->visionTranslate(2100, 80);
}


void ProgrammingSkillsAuton::startHoldInGoal(){
    au->setDriveVoltage(25, 25);
}

void ProgrammingSkillsAuton::stopHoldInGoal(){
    au->setDriveVoltage(0, 0);
}