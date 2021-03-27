#ifndef _LRTAuton_HPP_
#define _LRTAuton_HPP_

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
        bool fiveGoal;

    public:
        LRTAuton(MotorDefs *mtrDefs, bool ra, bool fg);
        ~LRTAuton();
        void runAuton();
    private:
        void captureFirstGoal();
        void captureSecondGoal();
        void captureThirdGoal();
        void captureFourthGoal();

        void captureFirstGoalStates();
        void captureSecondGoalStates();
        void captureThirdGoalStates();
        void captureFourthGoalStates();
        void captureFourthGoalBackwardStates();
        void captureFifthGoalStates();

        void startHoldInGoal();
        void stopHoldInGoal();
};

#endif // _LRTAuton_HPP_