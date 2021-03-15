#ifndef _THREEGOALAUTON_HPP_
#define _THREEGOALAUTON_HPP_

class MotorDefs;
class AutonUtils;
class Sensors;

namespace pros {
    class Task;
}

/**
 * Changeup Three goal game autonomous
 */
class ThreeGoalAuton {
    private:
        MotorDefs *mtrDefs;
        Sensors *sensors;
        AutonUtils *au;
        bool redAlliance;

    public:
        ThreeGoalAuton(MotorDefs *mtrDefs, bool ra);
        ~ThreeGoalAuton();
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

        void startHoldInGoal();
        void stopHoldInGoal();
};

#endif // _THREEGOALAUTON_HPP_