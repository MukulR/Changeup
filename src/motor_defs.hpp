//motor definitions for full robot
#ifndef _MOTOR_DEFS_H_
#define _MOTOR_DEFS_H_

namespace pros {
    class Motor;
}

class MotorDefs {
    public:
        pros::Motor *left_mtr_t;
        pros::Motor *left_mtr_b;
        pros::Motor *right_mtr_t;
        pros::Motor *right_mtr_b;
        pros::Motor *intake_r;
        pros::Motor *intake_l;
        pros::Motor *roller_t;
        pros::Motor *roller_b;

        MotorDefs();
        ~MotorDefs();
};
#endif //_MOTOR_DEFS_H_
