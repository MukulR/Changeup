#ifndef _AUTON_HPP_
#define _AUTON_HPP_

class MotorDefs;

/**
 * Autonomous that stacks cubes in the single stack zone.
 */
class Auton {
    private:
        MotorDefs *mtrDefs;
        bool redAlliance;

    public:
        Auton(MotorDefs *mtrDefs, bool ra);
        ~Auton();
        void runAuton();
        
};

#endif // _AUTON_HPP_