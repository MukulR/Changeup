#ifndef _SENSORS_HPP_
#define _SENSORS_HPP_

namespace pros {
    class ADIAnalogIn;
    class ADIDigitalIn;

    class Distance;
    class Imu;
    class Optical;
    class Vision;
}

class Sensors {
    public:
        pros::ADIAnalogIn* line_t;
        pros::ADIAnalogIn* line_m;
        pros::ADIDigitalIn* limit_t;

        pros::Distance* distance_l;
        pros::Distance* distance_r;
        pros::Imu* imu;
        pros::Optical* optical;
        pros::Vision* vision;

        Sensors();
        ~Sensors();
};

#endif // _SENSORS_HPP_