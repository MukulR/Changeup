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
        void captureThirdGoal();
        void captureFourthGoal();
        void captureFifthGoal();
        void captureSixthGoal();
        void captureSeventhGoal();
        void captureEighthGoal();
        void captureNinthGoal();


        void startHoldInGoal();
        void stopHoldInGoal();

        // Repeated sequences
        void threeShotSequence();
        void twoShotSequence(bool darkGoal = false);
        void thirdGoalSequence();
        void oneShotSequence(bool slowShot);
};

#endif // _PROGRAMMINGSKILLSAUTON_HPP_