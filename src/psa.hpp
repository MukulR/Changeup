#ifndef _PROGRAMMINGSKILLSAUTON_HPP_
#define _PROGRAMMINGSKILLSAUTON_HPP_

class MotorDefs;
class AutonUtils;
class Sensors;

namespace pros {
    class Task;
}

/**
 * Changeup Skills autonomous
 */
class ProgrammingSkillsAuton {
    private:
        MotorDefs *mtrDefs;
        Sensors *sensors;
        AutonUtils *au;
        bool redAlliance;

    public:
        ProgrammingSkillsAuton(MotorDefs *mtrDefs, bool ra);
        ~ProgrammingSkillsAuton();
        void runAuton();
    private:
        void captureFirstGoal();
        void captureSecondGoal();
        void captureFourthGoal();
        void captureFifthGoal();
        void captureSixthGoal();
        void captureCenterGoal();
        // void reviseNinthGoal();

        void captureThirdOrSeventhGoal(int goalNumber);


        void startHoldInGoal();
        void stopHoldInGoal();
};

#endif // _PROGRAMMINGSKILLSAUTON_HPP_