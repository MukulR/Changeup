#include "main.h"
#include "commons.hpp"
#include "motor_defs.hpp"

pros::ADIAnalogIn linet ('D');
pros::ADIAnalogIn linem ('B');

const double SLOW_DOWN_DISTANCE = 0.75;

void driveRobot(double degrees, int speed, void* param){
    MotorDefs* mtrDefs = (MotorDefs*)param;
	int delay_ms = speed; // delay units eq. to speed
	mtrDefs->left_mtr_t->tare_position();
	mtrDefs->right_mtr_t->tare_position();
	int setSpeed = speed;
	mtrDefs->left_mtr_t->move(-setSpeed);
	mtrDefs->left_mtr_b->move(-setSpeed);
	mtrDefs->right_mtr_t->move(setSpeed);
	mtrDefs->right_mtr_b->move(setSpeed);
	while(true){
		int degreesTraveled = (mtrDefs->left_mtr_t->get_position() + mtrDefs->right_mtr_t->get_position()) / 2;
		if (degreesTraveled >= degrees){
			mtrDefs->left_mtr_t->move(setSpeed);
			mtrDefs->left_mtr_b->move(setSpeed);
			mtrDefs->right_mtr_t->move(-setSpeed);
			mtrDefs->right_mtr_b->move(-setSpeed);
			pros::Task::delay(delay_ms);
			mtrDefs->left_mtr_t->move(0);
			mtrDefs->left_mtr_b->move(0);
			mtrDefs->right_mtr_t->move(0);
			mtrDefs->right_mtr_b->move(0);
			break;
		} else if (degreesTraveled >= degrees * SLOW_DOWN_DISTANCE){ // slow down after traveling 3/4 of the distance
			setSpeed = speed / 2;
			mtrDefs->left_mtr_t->move(-setSpeed);
			mtrDefs->left_mtr_b->move(-setSpeed);
			mtrDefs->right_mtr_t->move(setSpeed);
			mtrDefs->right_mtr_b->move(setSpeed);
		}
	}
}

void driveRobotBack(double degrees, int speed, void* param){
    MotorDefs* mtrDefs = (MotorDefs*)param;
	int delay_ms = speed; // delay units eq. to speed
	mtrDefs->left_mtr_t->tare_position();
	mtrDefs->right_mtr_t->tare_position();
	int setSpeed = speed;
	mtrDefs->left_mtr_t->move(setSpeed);
	mtrDefs->left_mtr_b->move(setSpeed);
	mtrDefs->right_mtr_t->move(-setSpeed);
	mtrDefs->right_mtr_b->move(-setSpeed);
	while(true){
		int degreesTraveled = (mtrDefs->left_mtr_t->get_position() + mtrDefs->right_mtr_t->get_position()) / 2;
		if (abs(degreesTraveled) >= abs(degrees)){
			mtrDefs->left_mtr_t->move(-setSpeed);
			mtrDefs->left_mtr_b->move(-setSpeed);
			mtrDefs->right_mtr_t->move(setSpeed);
			mtrDefs->right_mtr_b->move(setSpeed);
			pros::Task::delay(delay_ms * -1);
			mtrDefs->left_mtr_t->move(0);
			mtrDefs->left_mtr_b->move(0);
			mtrDefs->right_mtr_t->move(0);
			mtrDefs->right_mtr_b->move(0);
			break;
		} else if (abs(degreesTraveled) >= abs(degrees) * SLOW_DOWN_DISTANCE){ // slow down after traveling 3/4 of the distance
			setSpeed = speed / 2;
			mtrDefs->left_mtr_t->move(speed);
			mtrDefs->left_mtr_b->move(speed);
			mtrDefs->right_mtr_t->move(-speed);
			mtrDefs->right_mtr_b->move(-speed);
		}
	}
}

void turnWithoutSensors(double angle, int multiplier, void* param) {
    MotorDefs* mtrDefs = (MotorDefs*)param;
	const double TURN_SCALE_FACTOR = 480.0 / 90.0;
	double scaled_angle = TURN_SCALE_FACTOR * angle;
    mtrDefs->left_mtr_t->move_relative(scaled_angle * multiplier, 200);
    mtrDefs->left_mtr_b->move_relative(scaled_angle * multiplier, 200);
    mtrDefs->right_mtr_t->move_relative(-scaled_angle * multiplier, 200);
    mtrDefs->right_mtr_b->move_relative(-scaled_angle * multiplier, 200);
	while((abs(mtrDefs->right_mtr_b->get_position() - mtrDefs->right_mtr_b->get_target_position()) + 
        abs(mtrDefs->left_mtr_b->get_position() - mtrDefs->left_mtr_b->get_target_position()) +
        abs(mtrDefs->left_mtr_t->get_position() - mtrDefs->left_mtr_t->get_target_position()) +
        abs(mtrDefs->right_mtr_t->get_position() - mtrDefs->right_mtr_t->get_target_position())) > 6 * 4 /* Number of drive mtrs */ ) {
        pros::Task::delay(5);
    }
}
