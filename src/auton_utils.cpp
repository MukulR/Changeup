#include <iostream>

#include "auton_utils.hpp"
#include "motordefs.hpp"
#include "sensors.hpp"

using std::endl;

pros::ADIAnalogIn line_top ('D');
pros::ADIAnalogIn line_mid ('B');

AutonUtils::AutonUtils(MotorDefs* mtrDefs, Sensors* sensors) {
    this->mtrDefs = mtrDefs;
    this->sensors = sensors;
}

AutonUtils::~AutonUtils() {}

void AutonUtils::resetDriveEncoders(){
    mtrDefs->left_mtr_b->tare_position();
    mtrDefs->left_mtr_t->tare_position();
    mtrDefs->right_mtr_b->tare_position();
    mtrDefs->right_mtr_t->tare_position();
}

double AutonUtils::avgDriveEncoderValue() {
    return (fabs(mtrDefs->left_mtr_b->get_position()) + 
            fabs(mtrDefs->left_mtr_t->get_position()) + 
            fabs(mtrDefs->right_mtr_b->get_position()) + 
            fabs(mtrDefs->right_mtr_t->get_position())) / 4;
}

void AutonUtils::translate(int units) {
    // Initial imu rotation used for alignment
    double imu_initial = sensors->imu->get_rotation();

    // Motor power for drive
    int voltage = 50;

    // Reset drive encoders
    resetDriveEncoders();

    // Direction of movement
    int direction = abs(units) / units;
    double velocityScale = 4.1415926;

    // drive forward units
    while(avgDriveEncoderValue() < fabs(units)){
        // When turning left, the imu returns a negative value so we subtract a negative number(add) to speed up the left and slow down the right.
        // When turning right, the imu returns a positive value so we add a positive number to speed up the right and slow down the left.
        double leftVoltage = (direction * voltage) - ((sensors->imu->get_rotation() - imu_initial) * velocityScale);
        double rightVoltage = (direction * voltage) + ((sensors->imu->get_rotation() - imu_initial) * velocityScale);

        assignMotors(leftVoltage, rightVoltage);
        
        pros::delay(10);
    }

    // brake
    assignMotors(-20 * direction, -20 * direction);
    pros::delay(50);

    // set drive to neutral
    assignMotors(0, 0);
}

void AutonUtils::assignMotors(int leftVoltage, int rightVoltage) {
    *(mtrDefs->left_mtr_t) = (leftVoltage);
    *(mtrDefs->left_mtr_b) = (leftVoltage);
    *(mtrDefs->right_mtr_t) = (rightVoltage);
    *(mtrDefs->right_mtr_b) = (rightVoltage);
}

void AutonUtils::globalTurn(double angle) {
    // Initial imu heading
    double imuInitial = sensors->imu->get_heading();

    // If turning left is shorter, turn left(represented by 1). Otherwise, turn right(represented by -1).
    if(determineTurnDirection(imuInitial, angle)){
        // std::cout << "imuInit: " << imuInitial << " angle: " << angle << "\n";
        turnInGivenDirection(angle, 1);
    } else {
        // std::cout << "imuInit: " << imuInitial << " angle: " << angle << "\n";
        turnInGivenDirection(angle, -1);
    }
}

// Tells us which way we should turn to reach the desired angle faster. True = Left. False = Right.
bool AutonUtils::determineTurnDirection(double angle_current, double angle_desired) {   
    double angleTurningRight = std::fmod((360.0 - angle_current + angle_desired), 360.0);
    double angleTurningLeft = 360.0 - angleTurningRight;

    if(angleTurningRight > angleTurningLeft){
        return true;
    } else {
        return false;
    }
}

// Turns the robot in a given direction
void AutonUtils::turnInGivenDirection(double angle, double direction){
    // Function constants: voltage - robot speed, angleBuffer - precision of turn.
    int voltage = 50;
    double angleBuffer = 1.0;

    // Turn the robot based on direction
    assignMotors(-voltage * direction, voltage * direction);

    // Robot's current heading
    double imuCurrent = sensors->imu->get_heading();

    // Delay until the value of the imu reaches the desired amount
    while (imuCurrent > (angle + angleBuffer) || imuCurrent < (angle - angleBuffer)) {
        pros::Task::delay(10);
        // Update imu's current value
        imuCurrent = sensors->imu->get_heading();
    }

    // Coast
    assignMotors(40 * direction, -40 * direction);
    pros::Task::delay(50);
    assignMotors(0, 0);

    int voltageForCorrection = 0.5 * voltage;
    if (determineTurnDirection(sensors->imu->get_heading(), angle)) {
        // Correct undershoot at a slower speed
        assignMotors(-voltageForCorrection, voltageForCorrection);

        if(angle != 0.0){
            while(sensors->imu->get_heading() > angle) {
                pros::Task::delay(10);
            }
        } else {
            while(sensors->imu->get_heading() > 1.0) {
                pros::Task::delay(10);
            }
        }
    } else if (!determineTurnDirection(sensors->imu->get_heading(), angle)) {
        // Correct any overshoot at a slower speed
        assignMotors(voltageForCorrection, -voltageForCorrection);

        if(angle != 0.0){
            while(sensors->imu->get_heading() < angle) {
                pros::Task::delay(10);
            }
        } else {
            while(sensors->imu->get_heading() < 359.0) {
                pros::Task::delay(10);
            }
        }
    }
    // If anything needed to be corrected, it is corrected
    // and the motors can be stopped.
    assignMotors(0, 0);
}

void AutonUtils::rotate(int degrees, int voltage) {
    // determine the direction of the turn
    int direction = abs(degrees) / degrees;

    // reset IMU;
    //sensors->imu->reset();
    double imuVal = 0.0;

    std::cout << sensors->imu->get_rotation() << endl;
    // assign power to start the turn
    assignMotors(-voltage * direction, voltage * direction);
    //delay until the value of the imu reaches the desired amount
    while (fabs(imuVal) <= fabs(degrees)) {
        std::cout << sensors->imu->get_rotation() << endl;
        pros::Task::delay(10);
        imuVal = sensors->imu->get_rotation();
    }

    assignMotors(40, -40);
    pros::Task::delay(50);
    assignMotors(0, 0);
    std::cout << sensors->imu->get_rotation() << endl;
    // Let the robot decelerate

    // pros::Task::delay(100);
    // std::cout << sensors->imu->get_rotation() << endl;

    int voltageForCorrection = 0.5 * voltage;
    if (fabs(sensors->imu->get_rotation()) > abs(degrees)) {
        std::cout << "Here 1" << endl;
        // Correct overshoot at a slower speed
        assignMotors(voltageForCorrection * direction, -voltageForCorrection * direction);
        while(fabs(imuVal) > fabs(degrees)) {
            std::cout << sensors->imu->get_rotation() << endl;
            imuVal = sensors->imu->get_rotation();
            pros::Task::delay(10);
        }

    } else if (fabs(sensors->imu->get_rotation()) < fabs(degrees)) {
        std::cout << "Here 2" << endl;
        // Correct any undershoot at a slower speed
        assignMotors(-voltageForCorrection * direction, voltageForCorrection * direction);
        while(fabs(imuVal) < abs(degrees)) {
            std::cout << sensors->imu->get_rotation() << endl;
            pros::Task::delay(10);
            imuVal = sensors->imu->get_rotation();
        }
    }

    // If anything needed to be corrected, it is corrected
    // and the motors can be stopped.
    assignMotors(0, 0);
    // while (true) {
    //     std::cout << sensors->imu->get_rotation() << endl;
    //     pros::Task::delay(10);
    // }
}


bool indexingTop = false;
bool indexingMid = false;
bool filteringEnabled = false;

void AutonUtils::enableTopIndex() {
    indexingTop = true;
}

void AutonUtils::disableTopIndexing() {
    indexingTop = false;
}

void AutonUtils::enableMidIndex() {
    indexingMid = true;
}

void AutonUtils::disableMidIndexing() {
    indexingMid = false;
}

void AutonUtils::enableFiltering() {
    filteringEnabled = true;
}

void AutonUtils::indexTop(void* param) {
    MotorDefs* mtrDefs = (MotorDefs*)param;
    while (true) {
        // Case when we want to load the top ball, and the indexer is fully empty
        if (line_top.get_value() >= 2800 && indexingTop) {
            mtrDefs->intake_l->move(127);
            mtrDefs->intake_r->move(-127);
            mtrDefs->roller_b->move(-80);
            mtrDefs->roller_t->move(-80);
            // Wait until the top ball slot is filled
            while(line_top.get_value() >= 2800 && indexingTop) {
                pros::Task::delay(10);
            }
            indexingTop = false;
            stopIntakes(mtrDefs);
            stopRollers(mtrDefs);
        }
        pros::Task::delay(10);
    }
}


void AutonUtils::indexMid(void* param) {
    MotorDefs* mtrDefs = (MotorDefs*)param;
    while (true) {
        // Case when top is full, and we need to fill the middle spot.
        if (line_top.get_value() <= 2800 && line_mid.get_value() >= 2750 && indexingMid){
            mtrDefs->intake_l->move(127);
            mtrDefs->intake_r->move(-127);
            mtrDefs->roller_b->move(-80);
            while(line_mid.get_value() >= 2750 && indexingMid) {
                pros::Task::delay(10);
            }
            indexingMid = false;
            stopIntakes(mtrDefs);
            stopRollers(mtrDefs);
        }
        pros::Task::delay(10);
    }
}

void AutonUtils::filter(void* param){
    MotorDefs* mtrDefs = (MotorDefs*)param;
    while(true) {
        if (filteringEnabled) {
            mtrDefs->roller_b->move(-80);
            mtrDefs->roller_t->move(127);
            pros::Task::delay(1500);
            filteringEnabled = false;
            mtrDefs->roller_b->move(0);
            mtrDefs->roller_t->move(0);
        }
        pros::Task::delay(10);
        
    }
}

// ----------------------------------------

void AutonUtils::startIntakes(MotorDefs* mtrDefs) {
    mtrDefs->intake_l->move(127);
    mtrDefs->intake_r->move(-127);
}

void AutonUtils::startOuttake(MotorDefs* mtrDefs) {
    mtrDefs->intake_l->move(-127);
    mtrDefs->intake_r->move(127);
}

void AutonUtils::stopIntakes(MotorDefs* mtrDefs) {
    mtrDefs->intake_l->move(0);
    mtrDefs->intake_r->move(0);
}

void AutonUtils::stopRollers(MotorDefs* mtrDefs) {
    mtrDefs->roller_t->move(0);
    mtrDefs->roller_b->move(0);
}


void AutonUtils::oneShot() {
    mtrDefs->intake_r->move(60);
    mtrDefs->intake_l->move(-60);
    pros::Task::delay(200);
    mtrDefs->intake_r->move(0);
    mtrDefs->intake_l->move(0);

    mtrDefs->roller_b->move(80);
    pros::Task::delay(100);
    mtrDefs->roller_b->move(0);

    pros::Task::delay(50);

    mtrDefs->roller_t->move(-127);
    pros::Task::delay(350);
    mtrDefs->roller_t->move(0);
}

void AutonUtils::doubleShot() {
    mtrDefs->intake_r->move(60);
    mtrDefs->intake_l->move(-60);
    pros::Task::delay(200);
    mtrDefs->intake_r->move(0);
    mtrDefs->intake_l->move(0);

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