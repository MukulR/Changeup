#include <stdlib.h>

#include "main.h"
#include "user_control.hpp"
#include "motor_defs.hpp"
#include "okapi/api.hpp"


//using namespace std::placeholders;

MotorDefs mtrDefs;

using namespace okapi;

std::shared_ptr<ChassisController> myChassis =
  ChassisControllerBuilder().withMotors({16, 19}, {2, 13}).withDimensions(AbstractMotor::gearset::blue, {{1.71_in, 11_in}, imev5BlueTPR}).build();

std::shared_ptr<AsyncMotionProfileController> profileController =
	AsyncMotionProfileControllerBuilder().withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    }).withOutput(myChassis).buildMotionProfileController();

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


const double SLOW_DOWN_DISTANCE = 0.75;

void driveRobot(double degrees, int speed){
	int delay_ms = speed; // delay units eq. to speed
	mtrDefs.left_mtr_t->tare_position();
	mtrDefs.right_mtr_t->tare_position();
	int setSpeed = speed;
	mtrDefs.left_mtr_t->move(setSpeed);
	mtrDefs.left_mtr_b->move(setSpeed);
	mtrDefs.right_mtr_t->move(setSpeed);
	mtrDefs.right_mtr_b->move(setSpeed);
	while(true){
		int degreesTraveled = (mtrDefs.left_mtr_t->get_position() + mtrDefs.right_mtr_t->get_position()) / 2;
		if (degreesTraveled >= degrees){
			mtrDefs.left_mtr_t->move(-setSpeed);
			mtrDefs.left_mtr_b->move(-setSpeed);
			mtrDefs.right_mtr_t->move(-setSpeed);
			mtrDefs.right_mtr_b->move(-setSpeed);
			pros::Task::delay(delay_ms);
			mtrDefs.left_mtr_t->move(0);
			mtrDefs.left_mtr_b->move(0);
			mtrDefs.right_mtr_t->move(0);
			mtrDefs.right_mtr_b->move(0);
			break;
		} else if (degreesTraveled >= degrees * SLOW_DOWN_DISTANCE){ // slow down after traveling 3/4 of the distance
			setSpeed = speed / 2;
			mtrDefs.left_mtr_t->move(setSpeed);
			mtrDefs.left_mtr_b->move(setSpeed);
			mtrDefs.right_mtr_t->move(setSpeed);
			mtrDefs.right_mtr_b->move(setSpeed);
		}
	}
}

void driveRobotBack(double degrees, int speed){
	int delay_ms = speed; // delay units eq. to speed
	mtrDefs.left_mtr_t->tare_position();
	mtrDefs.right_mtr_t->tare_position();
	int setSpeed = speed;
	mtrDefs.left_mtr_t->move(-setSpeed);
	mtrDefs.left_mtr_b->move(-setSpeed);
	mtrDefs.right_mtr_t->move(-setSpeed);
	mtrDefs.right_mtr_b->move(-setSpeed);
	while(true){
		int degreesTraveled = (abs(mtrDefs.left_mtr_t->get_position()) + abs(mtrDefs.right_mtr_t->get_position())) / 2;
		if (abs(degreesTraveled) >= abs(degrees)){
			mtrDefs.left_mtr_t->move(setSpeed);
			mtrDefs.left_mtr_b->move(setSpeed);
			mtrDefs.right_mtr_t->move(setSpeed);
			mtrDefs.right_mtr_b->move(setSpeed);
			pros::Task::delay(delay_ms);
			mtrDefs.left_mtr_t->move(0);
			mtrDefs.left_mtr_b->move(0);
			mtrDefs.right_mtr_t->move(0);
			mtrDefs.right_mtr_b->move(0);
			break;
		} else if (abs(degreesTraveled) >= abs(degrees) * SLOW_DOWN_DISTANCE){ // slow down after traveling 3/4 of the distance
			setSpeed = speed / 2;
			mtrDefs.left_mtr_t->move(-speed);
			mtrDefs.left_mtr_b->move(-speed);
			mtrDefs.right_mtr_t->move(-speed);
			mtrDefs.right_mtr_b->move(-speed);
		}
	}
}

void waitForCompletion() {
	while((abs(mtrDefs.right_mtr_b->get_position() - mtrDefs.right_mtr_b->get_target_position()) + 
        abs(mtrDefs.left_mtr_b->get_position() - mtrDefs.left_mtr_b->get_target_position()) +
        abs(mtrDefs.left_mtr_t->get_position() - mtrDefs.left_mtr_t->get_target_position()) +
        abs(mtrDefs.right_mtr_t->get_position() - mtrDefs.right_mtr_t->get_target_position())) > 6 * 4 /* Number of drive mtrs */ ) {
        pros::Task::delay(5);
    }
}

void turnWithoutSensors(double angle, int multiplier) {
	mtrDefs.left_mtr_t->tare_position();
	mtrDefs.right_mtr_t->tare_position();
	mtrDefs.left_mtr_b->tare_position();
	mtrDefs.right_mtr_b->tare_position();

	const double TURN_SCALE_FACTOR = 480.0 / 90.0;
	double scaled_angle = TURN_SCALE_FACTOR * angle;
    mtrDefs.left_mtr_t->move_relative(scaled_angle * multiplier, 200);
    mtrDefs.left_mtr_b->move_relative(scaled_angle * multiplier, 200);
    mtrDefs.right_mtr_t->move_relative(-scaled_angle * multiplier, 200);
    mtrDefs.right_mtr_b->move_relative(-scaled_angle * multiplier, 200);
}





void autonomous() {
	driveRobot(300, 80);
	pros::Task::delay(100);
	turnWithoutSensors(90, 1);
	waitForCompletion();
	pros::Task::delay(100);
	driveRobot(300, 80);
	
	
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */


void opcontrol() {
	/*
	UserControl uc;
	auto intakeFp = std::bind(&UserControl::intake, uc, (void*)NULL);
	auto rollersFp = std::bind(&UserControl::rollers, uc, (void*)NULL);
	auto driveFp = std::bind(&UserControl::drive, uc, (void*)NULL);
	*/
	
	pros::Task intakeTask(intake, &mtrDefs);
	pros::Task rollerTask(rollers, &mtrDefs);
	pros::Task driveTask(drive, &mtrDefs);
	pros::Task indexTask(index, &mtrDefs);
	pros::Task controlTask(control, &mtrDefs);
}
