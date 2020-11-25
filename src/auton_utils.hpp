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
        void translate(int units, double angle = -1.0);
        void resetDriveEncoders();
        double avgDriveEncoderValue();
        void rotate(int degrees, int voltage);
        void globalTurn(double angle);

        // PID Turn Stuff
        void pidGlobalTurn(double angle);
        void pidRotate(double angle, int direction);
        void assignMotorsVol(int leftVoltage, int rightVoltage);
        double determineError(double imu_cur, double imu_desired, int direction);

        void visionTranslate(int units, int speed);
        
        void turnRightToZeroHeading();
        void turnLeftToZeroHeading();

        bool determineTurnDirection(double angle_current, double angle_desired);
        void turnInGivenDirection(double angle, double direction);

        void oneShot();
        void slowOneShot();
        void doubleShot();

        void indexTop();
        void indexMid();
        void indexMidRollers();
        void filter();

            /*
        static void indexTop(void* param);
        static void indexMid(void* param);
        static void filter(void* param);
        */

        static void enableTopIndex();
        static void disableTopIndexing();

        static void enableMidIndex();
        static void disableMidIndexing();

        static void enableFiltering();
        static void waitUntilFiltered();
        static void waitUntilTopIndexed();
        static void waitUntilMidIndexed();

        void waitUntilIntaked(bool darkGoal);
    

        static void startIntakes(MotorDefs* mtrDefs);
        static void stopIntakes(MotorDefs* mtrDefs);
        static void stopRollers(MotorDefs* mtrDefs);
        static void startOuttake(MotorDefs* mtrDefs);        
};

#endif // _AUTON_UTILS_HPP_