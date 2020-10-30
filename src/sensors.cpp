#include "main.h"
#include "sensors.hpp"

const int IMU_PORT = 11;
const char LINE_T_PORT = 'D';
const char LINE_M_PORT = 'B';

Sensors::Sensors() {
    imu = new pros::Imu(IMU_PORT);
    imu->reset();
    while (imu->is_calibrating()) {
        pros::Task::delay(10);
    }
    line_t = new pros::ADIAnalogIn(LINE_T_PORT);
    line_m = new pros::ADIAnalogIn(LINE_M_PORT);
}

Sensors::~Sensors() {
    delete imu;
    delete line_t;
    delete line_m;
}

