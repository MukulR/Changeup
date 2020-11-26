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
        void setDriveVoltage(int leftVoltage, int rightVoltage);
        void translate(int units, double angle = -1.0);
        void resetDriveEncoders();
        double avgDriveEncoderValue();

        // PID Turn Stuff
        void pidGlobalTurn(double angle);
        void pidRotate(double angle, int direction);
        void setDriveSpeed(int leftSpeed, int rightSpeed);
        double determineError(double imu_cur, double imu_desired, int direction);

        void visionTranslate(int units, int speed);
        
        

        void oneShot();
        void slowOneShot();


        static bool notBlueBall();
        void twoInTwoOut();


        void indexTop();
        void indexMid();
        void indexMidRollers();
        void filter();

        static void indexTop(void* param);
        static void indexMid(void* param);
        static void filter(void* param);
        static void shootBalls(void* param);
        static void backUpAndOuttake(void* param);
        static void moveForwardAndFilter(void* param);

    

        static void startIntakes(MotorDefs* mtrDefs);
        static void stopIntakes(MotorDefs* mtrDefs);
        static void startRollers(MotorDefs* mtrDefs);
        static void stopRollers(MotorDefs* mtrDefs);
        static void startOuttake(MotorDefs* mtrDefs);        
};

#endif // _AUTON_UTILS_HPP_