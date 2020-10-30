#ifndef _SENSORS_HPP_
#define _SENSORS_HPP_

namespace pros {
    class Imu;
    class ADIAnalogIn;
}

class Sensors {
    public:
        pros::Imu* imu;
        pros::ADIAnalogIn* line_t;
        pros::ADIAnalogIn* line_m;

        Sensors();
        ~Sensors();
};

#endif // _SENSORS_HPP_