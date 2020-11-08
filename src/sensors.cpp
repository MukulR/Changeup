#include "main.h"
#include "sensors.hpp"

const int IMU_PORT = 11;
const char LINE_T_PORT = 'D';
const char LINE_M_PORT = 'B';
const char LIMIT_T_PORT = 'A';

Sensors::Sensors() {
    imu = new pros::Imu(IMU_PORT);
    imu->reset();
    pros::Task::delay(3000);
    // line_t = new pros::ADIAnalogIn(LINE_T_PORT);
    // line_m = new pros::ADIAnalogIn(LINE_M_PORT);
    // limit_t = new pros::ADIDigitalIn(LIMIT_T_PORT);
}

Sensors::~Sensors() {
    delete imu;
    // delete line_t;
    // delete line_m;
    // delete limit_t;
}

