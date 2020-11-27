#include "main.h"

#include "autonselection.hpp"
#include "sensors.hpp"

const int IMU_PORT = 11;
const char LINE_T_PORT = 'D';
const char LINE_M_PORT = 'B';
const char LIMIT_T_PORT = 'A';

Sensors::Sensors() {
    distance_l = distance_sensor_l;
    distance_r = distance_sensor_r;
    imu = inertial_sensor;
    optical = optical_sensor;
    vision = vision_sensor;
    // line_t = new pros::ADIAnalogIn(LINE_T_PORT);
    // line_m = new pros::ADIAnalogIn(LINE_M_PORT);
    // limit_t = new pros::ADIDigitalIn(LIMIT_T_PORT);
}

Sensors::~Sensors() {
    delete distance_l;
    delete distance_r;
    delete imu;
    delete optical;
    delete vision;
    // delete line_t;
    // delete line_m;
    // delete limit_t;
}

