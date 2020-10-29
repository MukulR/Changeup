#include "main.h"
#include "motordefs.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);


pros::ADIAnalogIn line_t ('D');
pros::ADIAnalogIn line_m ('B');

MotorDefs mtrDefs;

bool detection_enabled = false;

void drive(void* param){
	std::cout << "Here" << "\n";
	while(true) {
		int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		float scaledTurn = ((turn * 100) * 0.75) / 100;
		float leftMtrVals = (forward + scaledTurn);
		float rightMtrVals = -(scaledTurn - forward);
		if(leftMtrVals > 127){
			leftMtrVals = 127;
		}
		if(leftMtrVals < -127){
			leftMtrVals = -127;
		}
		if(rightMtrVals > 127){
			rightMtrVals = 127;
		}
		if(rightMtrVals < -127){
			rightMtrVals = -127;
		}
		mtrDefs.left_mtr_t->move(leftMtrVals);
		mtrDefs.left_mtr_b->move(leftMtrVals);
		mtrDefs.right_mtr_t->move(rightMtrVals);
		mtrDefs.right_mtr_b->move(rightMtrVals);
		// The below delay is required for tasks to work in PROS.
		pros::Task::delay(10);
	}
}

void stopAll() {
	detection_enabled = false;
	mtrDefs.roller_b->move(0);
	mtrDefs.roller_t->move(0);
	mtrDefs.intake_l->move(0);
	mtrDefs.intake_r->move(0);
}

void intake(void* param) {
	
	while(true) {
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			mtrDefs.intake_r->move(-127);
			mtrDefs.intake_l->move(127);
			while(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
				pros::Task::delay(10);
			}
			mtrDefs.intake_r->move(0);
			mtrDefs.intake_l->move(0);
		}

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			mtrDefs.intake_r->move(127);
			mtrDefs.intake_l->move(-127);
			while(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
				pros::Task::delay(10);
			}
			mtrDefs.intake_r->move(0);
			mtrDefs.intake_l->move(0);
		}
	}
}

void rollers(void* param) {
	
	while(true) {
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			mtrDefs.roller_t->move(-127);
			mtrDefs.roller_b->move(-127);
			while(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
				pros::Task::delay(10);
			}
			mtrDefs.roller_t->move(0);
			mtrDefs.roller_b->move(0);
		}

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			mtrDefs.roller_t->move(127);
			mtrDefs.roller_b->move(-127);
			while(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
				pros::Task::delay(10);
			}
			mtrDefs.roller_t->move(0);
			mtrDefs.roller_b->move(0);
		}
	}
}


void index(void *param){
	
	while (true) {
		std::cout << detection_enabled << "\n";
		if (detection_enabled) {
			if (line_t.get_value() >= 2800 && line_m.get_value() >= 2750){
				std::cout << "Here1" << "\n";
				mtrDefs.intake_l->move(127);
				mtrDefs.intake_r->move(-127);
				mtrDefs.roller_b->move(-80);
				mtrDefs.roller_t->move(-80);
				while(line_t.get_value() >= 2800 && detection_enabled) {
					if (!detection_enabled){
						return;
					}
					pros::Task::delay(10);
				}
				mtrDefs.roller_b->move(0);
				mtrDefs.roller_t->move(0);
				mtrDefs.intake_l->move(0);
				mtrDefs.intake_r->move(0);
			}

			if (line_t.get_value() <= 2800 && line_m.get_value() >= 2750){
				std::cout << "Here2" << "\n";
				mtrDefs.intake_l->move(127);
				mtrDefs.intake_r->move(-127);
				mtrDefs.roller_b->move(-80);
				while(line_m.get_value() >= 2750 && detection_enabled) {
					if (!detection_enabled){
						return;
					}
					pros::Task::delay(10);
				}
				mtrDefs.roller_b->move(0);
				mtrDefs.roller_t->move(0);
				mtrDefs.intake_l->move(0);
				mtrDefs.intake_r->move(0);
				detection_enabled = false;
			}

			if (line_t.get_value() >= 2800 && line_m.get_value() < 2750){
				std::cout << "Here3" << "\n";
				mtrDefs.intake_l->move(127);
				mtrDefs.intake_r->move(-127);
				mtrDefs.roller_t->move(-80);
				mtrDefs.roller_b->move(-80);
				while(line_t.get_value() >= 2750 && detection_enabled) {
					if (!detection_enabled){
						return;
					}
					pros::Task::delay(10);
				}
				mtrDefs.roller_b->move(0);
				mtrDefs.roller_t->move(0);
				mtrDefs.intake_l->move(0);
				mtrDefs.intake_r->move(0);
				detection_enabled = false;
			}
		}
	}
}

void control(void* param) {
	
	while(true) {
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			detection_enabled = false;
			stopAll();
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
			detection_enabled = true;
			std::cout << detection_enabled;
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			detection_enabled = false;

			mtrDefs.intake_r->move(60);
			mtrDefs.intake_l->move(-60);
			pros::Task::delay(200);
			mtrDefs.intake_r->move(0);
			mtrDefs.intake_l->move(0);

			mtrDefs.roller_b->move(80);
			pros::Task::delay(100);
			mtrDefs.roller_b->move(0);

			pros::Task::delay(50);

			mtrDefs.roller_t->move(-127);
			pros::Task::delay(300);
			mtrDefs.roller_t->move(0);
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
			// detection_enabled = false;
			// mtrDefs.roller_t->move(-127);
			// pros::Task::delay(500);
			// mtrDefs.roller_b->move(-127);
			// pros::Task::delay(300);
			// mtrDefs.roller_b->move(0);
			// mtrDefs.roller_t->move(0);

			mtrDefs.intake_r->move(60);
			mtrDefs.intake_l->move(-60);
			pros::Task::delay(200);
			mtrDefs.intake_r->move(0);
			mtrDefs.intake_l->move(0);

			mtrDefs.roller_b->move(80);
			pros::Task::delay(200);
			mtrDefs.roller_b->move(15);
			
			mtrDefs.roller_t->move(-127);
			pros::Task::delay(300);
			mtrDefs.roller_t->move(0);

			pros::Task::delay(50);

			mtrDefs.roller_b->move(-127);
			mtrDefs.roller_t->move(-127);
			pros::Task::delay(300);
			mtrDefs.roller_b->move(0);
			pros::Task::delay(600);
			mtrDefs.roller_t->move(0);
		}
	}
}

void opcontrol() {
	pros::Task driveTask(drive);
	pros::Task intakeTask(intake);
	pros::Task rollerTask(rollers);
	pros::Task indexTask(index);
	pros::Task controlTask(control);
}

