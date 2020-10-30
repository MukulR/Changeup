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

void AutonUtils::translate(double units) {

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