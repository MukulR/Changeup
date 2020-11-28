#include "main.h"

#include "autonselection.hpp"
#include "sensors.hpp"

const int IMU_PORT = 11;
const char LINE_T_PORT = 'D';
const char LINE_M_PORT = 'B';
const char LIMIT_T_PORT = 'A';

Sensors::Sensors() {
    imu = inertial_sensor;
    optical = optical_sensor;
    vision = vision_sensor;
}

Sensors::~Sensors() {
    delete imu;
    delete optical;
    delete vision;
}

