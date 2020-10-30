#include <iostream>

#include "auton_utils.hpp"
#include "motordefs.hpp"
#include "sensors.hpp"

using std::endl;

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
        double leftVoltage = (direction * voltage) - (sensors->imu->get_rotation() * velocityScale);
        double rightVoltage = (direction * voltage) + (sensors->imu->get_rotation() * velocityScale);

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

void AutonUtils::rotate(int degrees, int voltage) {
    // determine the direction of the turn
    int direction = abs(degrees) / degrees;

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