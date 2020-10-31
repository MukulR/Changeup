#ifndef _AUTON_UTILS_HPP_
#define _AUTON_UTILS_HPP_

class MotorDefs;
class Sensors;

namespace pros {
    class Imu;
}

class AutonUtils {
    private:
        MotorDefs* mtrDefs;
        Sensors* sensors;
    public:
        AutonUtils(MotorDefs* mtrDefs, Sensors* sensors);
        ~AutonUtils();
        void assignMotors(int left, int right);
        void translate(int units);
        void resetDriveEncoders();
        double avgDriveEncoderValue();
        void rotate(int degrees, int voltage);
        void globalTurn(double angle);
        bool determineTurnDirection(double angle_current, double angle_desired);
        void turnInGivenDirection(double angle, double direction);
};

#endif // _AUTON_UTILS_HPP_