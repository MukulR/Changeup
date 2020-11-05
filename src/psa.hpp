#ifndef _PROGRAMMINGSKILLSAUTON_HPP_
#define _PROGRAMMINGSKILLSAUTON_HPP_

class MotorDefs;
class AutonUtils;

/**
 * Changeup Skills autonomous
 */
class ProgrammingSkillsAuton {
    private:
        MotorDefs *mtrDefs;
        bool redAlliance;
    public:
        ProgrammingSkillsAuton(MotorDefs *mtrDefs, bool ra);
        ~ProgrammingSkillsAuton();
        void runAuton();
    private:
        void captureFirstGoal(AutonUtils au);
        void left_corner_right_center(AutonUtils au);
        void left_center_right_corner(AutonUtils au);
        void startHoldInGoal(AutonUtils au);
        void stopHoldInGoal(AutonUtils au);
};

#endif // _PROGRAMMINGSKILLSAUTON_HPP_