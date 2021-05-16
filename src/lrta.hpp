#ifndef _LRTAuton_HPP_
#define _LRTAuton_HPP_

#include <string>

class MotorDefs;
class AutonUtils;
class Sensors;

namespace pros {
    class Task;
}

/**
 * Changeup Three goal game autonomous
 */
class LRTAuton {
    private:
        MotorDefs *mtrDefs;
        Sensors *sensors;
        AutonUtils *au;
        bool redAlliance;
        std::string autonType;

    public:
        LRTAuton(MotorDefs *mtrDefs, bool ra, std::string at);
        ~LRTAuton();
        void runAuton();
    private:
        void captureFirstGoal();
        void captureSecondGoal();
        void captureThirdGoal();
        void captureFourthGoal();

        void captureFirstGoalHRE();
        void captureSecondGoalHRE();
        void captureThirdGoalHRE();
        void captureFourthGoalHRE();
        void captureFourthGoalBackwardHRE();
        void captureFifthGoalHRE();

        void captureFirstGoalHRBH();
        void captureSecondGoalHRBH();
        void captureThirdGoalHRBH();
        void captureFourthGoalHRBH();

        void startHoldInGoal();
        void stopHoldInGoal();
};

#endif // _LRTAuton_HPP_