#ifndef _PROGRAMMINGSKILLSAUTON_HPP_
#define _PROGRAMMINGSKILLSAUTON_HPP_

class MotorDefs;
class AutonUtils;
class Sensors;

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
        void left_corner_right_center();
        void left_center_right_corner();
        void startHoldInGoal();
        void stopHoldInGoal();

        // Repeated sequences
        void oneShotSequence(bool intakeAfterIndex, bool corner);
};

#endif // _PROGRAMMINGSKILLSAUTON_HPP_