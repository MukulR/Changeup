#include "main.h"

#include "autonselection.hpp"
#include "sensors.hpp"

const int IMU_PORT = 12;
const int OPTICAL_PORT = 6;
const int VISION_PORT = 20;
const char LINE_T_PORT = 'D';
const char LINE_M_PORT = 'B';
const char LIMIT_T_PORT = 'A';

Sensors::Sensors() {
    imu = inertial_sensor;
    optical = new pros::Optical(OPTICAL_PORT);
    vision = new pros::Vision(VISION_PORT);
}

Sensors::~Sensors() {
    delete imu;
    delete optical;
    delete vision;
}

