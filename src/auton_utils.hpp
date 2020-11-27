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
        void goalTranslate(int units, bool parallelOuttake);
        void translateWithDS();
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
        void cornerGoalSequence();
        void nonCornerGoalSequence(int moveBackDistance, double heading);
        void centerSequence();

        void indexMidRollers();
        void filter();

        // static void index(void* param);
        static void indexTwoBalls(void* param);
        static void indexOneBall(void* param);
        static void filter(void* param);
        static void shootBalls(void* param);
        static void backUpAndOuttake(void* param);
        static void moveForwardAndFilter(void* param);

    

        static void startIntakes(MotorDefs* mtrDefs);
        static void startIntakesSlow(MotorDefs* mtrDefs);
        static void stopIntakes(MotorDefs* mtrDefs);
        static void startRollers(MotorDefs* mtrDefs);
        static void stopRollers(MotorDefs* mtrDefs);
        static void startOuttake(MotorDefs* mtrDefs);  
        static void startOuttakeFast(MotorDefs* mtrDefs);
        static bool blueBallInFilteringPos(Sensors *sensors);    
        static bool ballAtTop();
        static bool ballAtMid(); 

        static void setIndexingOneBall(bool value); 
};

#endif // _AUTON_UTILS_HPP_