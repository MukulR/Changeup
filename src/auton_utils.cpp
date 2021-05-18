#include <iostream>

#include "autonselection.hpp"
#include "auton_utils.hpp"
#include "motordefs.hpp"
#include "sensors.hpp"

using std::endl;

// TODO: Move to Sensors.hpp
pros::ADIAnalogIn line_top('D');
pros::ADIAnalogIn line_mid('B');
pros::ADIAnalogIn line_bot({{15, 'H'}});
pros::ADIAnalogIn line_drive_right('H');
pros::ADIAnalogIn line_drive_left('C');
pros::ADIDigitalIn limit_t('A');
pros::ADIDigitalIn intake_bumper('G');
pros::ADIUltrasonic ultrasonic('E', 'F');

bool indexingOneBall = false;

const int DISTANCE_THRESHOLD = 200;
const int DISTANCE_TO_FENCE = 125;

const int TURN_VOLTAGE = 50;
const int CORRECTION_TURN_VOLTAGE = 25;

const int TRANSLATE_VOLTAGE = 80;

AutonUtils::AutonUtils(MotorDefs* mtrDefs, Sensors* sensors) {
    this->mtrDefs = mtrDefs;
    this->sensors = sensors;
}

AutonUtils::~AutonUtils() {}

void AutonUtils::resetDriveEncoders(){
    mtrDefs->left_mtr_b->tare_position();
    mtrDefs->left_mtr_t->tare_position();
    mtrDefs->right_mtr_b->tare_position();
    mtrDefs->right_mtr_t->tare_position();
}

double AutonUtils::avgDriveEncoderValue() {
    return (fabs(mtrDefs->left_mtr_b->get_position()) + 
            fabs(mtrDefs->left_mtr_t->get_position()) + 
            fabs(mtrDefs->right_mtr_b->get_position()) + 
            fabs(mtrDefs->right_mtr_t->get_position())) / 4;
}

void AutonUtils::translate(int units, int voltage, double angle, bool braking) {
    double imu_correction = 0.0;

    // Initial imu rotation used for alignment
    double imu_initial = sensors->imu->get_heading();
    double right_err = 0;
    double left_err = 0;
    double leftVoltage;
    double rightVoltage;
    int dir;

    if(angle != -1.0) {
        // Subtract 3 since imu is off.
        imu_initial = angle + imu_correction;
    }

    // Reset drive encoders
    resetDriveEncoders();

    // Direction of movement
    int direction = abs(units) / units;
    double velocityScale = 4.1415926;

    // drive forward units
    while(avgDriveEncoderValue() < fabs(units)){
        right_err = determineError(sensors->imu->get_heading(), angle, 1);
        left_err = -right_err;

        if(right_err > 0.0) {
            dir = 1;
        } else {
            dir = -1;
        }

        if (voltage > 95) {
            leftVoltage = (direction * voltage) + 2 * (determineError(sensors->imu->get_heading(), imu_initial, dir) * velocityScale * dir);
            rightVoltage = (direction * voltage) - 2 * (determineError(sensors->imu->get_heading(), imu_initial, dir) * velocityScale * dir);
        } else {
            leftVoltage = (direction * voltage) + (determineError(sensors->imu->get_heading(), imu_initial, dir) * velocityScale * dir);
            rightVoltage = (direction * voltage) - (determineError(sensors->imu->get_heading(), imu_initial, dir) * velocityScale * dir);
        }

        setDriveVoltage(leftVoltage, rightVoltage);
        
        pros::delay(10);
    }

    // brake
    if (braking) {
        setDriveVoltage(-0.5 * TRANSLATE_VOLTAGE * direction, -0.5 * TRANSLATE_VOLTAGE * direction);
        pros::delay(50);
    }

    // set drive to neutral
    setDriveVoltage(0, 0);
}

void AutonUtils::goalTranslate(int units, bool parallelOuttake) {
    int direction = abs(units) / units;
    setDriveVoltage(direction * TRANSLATE_VOLTAGE, direction * TRANSLATE_VOLTAGE);
    
    // Reset drive encoders
    resetDriveEncoders();

    while(avgDriveEncoderValue() < fabs(units)){
        if (parallelOuttake && avgDriveEncoderValue() > fabs(0.2 * units)){
            startOuttake(mtrDefs);
        }
    }

    // brake
    setDriveVoltage(direction * -20, direction * -20);
    pros::delay(50);

    // set drive to neutral
    setDriveVoltage(0, 0);
    if (parallelOuttake) {
        stopIntakes(mtrDefs);
    }
}


void AutonUtils::translateWithDS() {
    setDriveVoltage(TRANSLATE_VOLTAGE, TRANSLATE_VOLTAGE);

    while ((sensors->distance_l->get() > DISTANCE_THRESHOLD && sensors->distance_r->get() > DISTANCE_THRESHOLD)
            || (sensors->distance_l->get() == 0 || sensors->distance_r->get() == 0)) {

    }

    setDriveVoltage(-20, -20);
    pros::Task::delay(50);
    setDriveVoltage(0, 0);
    std::cout << "Done" << std::endl;
}

void AutonUtils::setDriveVoltage(int leftVoltage, int rightVoltage) {
    *(mtrDefs->left_mtr_t) = (leftVoltage);
    *(mtrDefs->left_mtr_b) = (leftVoltage);
    *(mtrDefs->right_mtr_t) = (rightVoltage);
    *(mtrDefs->right_mtr_b) = (rightVoltage);
}

void AutonUtils::visionTranslate(int units, int speed, bool useLT, bool useBumper) {
    pros::vision_object_s_t obj;
    int des_left_coord;
    int turn_direction;

    double left_speed;
    double right_speed;
    double speed_correction;

    // Reset drive encoders
    resetDriveEncoders();
    while (useBumper ? !intake_bumper.get_value() : (useLT ? line_drive_right.get_value() > 550 : avgDriveEncoderValue() < fabs(units))) {
    // while (avgDriveEncoderValue() < fabs(units) || (useLT && line_drive_right.get_value() < 550) || (useBumper && intake_bumper.get_value())) {
        // Run-away robot prevention!
        if(useLT && (avgDriveEncoderValue() > fabs(units))) {
            break;
        }


        obj = sensors->vision->get_by_size(0);
        des_left_coord = (315 / 2) - (obj.width / 2);

        turn_direction = des_left_coord - obj.left_coord;
        turn_direction = fabs(turn_direction) / turn_direction;

        if(sensors->vision->get_object_count() <= 0 || obj.width < 20) {
            speed_correction = 0;
        } else {
            speed_correction = fabs(des_left_coord - obj.left_coord) / 2.0;
        }

        speed_correction = speed_correction * turn_direction;

        left_speed = speed + speed_correction;
        right_speed = speed - speed_correction;
        
        //std::cout << "LSPEED: " << left_speed << " RSPEED: " << right_speed << " dir " << turn_direction << "\n";

        setDriveVoltage(left_speed, right_speed);
        pros::Task::delay(10);
    }

    // brake
    setDriveVoltage(-50, -50);
    if(useLT || useBumper) {
        pros::delay(100);
    } else {
        pros::delay(50);
    }

    // set drive to neutral
    setDriveVoltage(0, 0);
}

void AutonUtils::signatureVisionTranslate(int units, int speed, bool useLT, bool useBumper, bool blue) {
    //pros::vision_object_s_t obj;

    const int RED_SIG_ID = 1;
    const int BLUE_SIG_ID = 2;
    pros::vision_signature_s_t RED_SIG = pros::Vision::signature_from_utility(RED_SIG_ID, 7057, 10137, 8597, -1205, 239, -483, 3.700, 0);
	pros::vision_signature_s_t BLUE_SIG = pros::Vision::signature_from_utility(BLUE_SIG_ID, -3449, -1285, -2367, 5405, 10409, 7907, 2.200, 0);
	// Make the vision sensor aware of these signatures
	sensors->vision->set_signature(RED_SIG_ID, &RED_SIG);
	sensors->vision->set_signature(BLUE_SIG_ID, &BLUE_SIG);

    int des_left_coord;
    int turn_direction;

    double left_speed;
    double right_speed;
    double speed_correction;

    // Reset drive encoders
    resetDriveEncoders();

    while (useBumper ? !intake_bumper.get_value() : (useLT ? line_drive_right.get_value() > 550 : avgDriveEncoderValue() < fabs(units))) {
        // Run-away robot prevention!
        if(useLT && (avgDriveEncoderValue() > fabs(units))) {
            break;
        }
        
        pros::vision_object_s_t obj = sensors->vision->get_by_sig(0, blue ? BLUE_SIG_ID : RED_SIG_ID);
        std::cout << (obj.signature) << std::endl;
        des_left_coord = (315 / 2) - (obj.width / 2);

        turn_direction = des_left_coord - obj.left_coord;
        turn_direction = fabs(turn_direction) / turn_direction;

        if(sensors->vision->get_object_count() <= 0 || obj.width < 20) {
            speed_correction = 0;
        } else {
            speed_correction = fabs(des_left_coord - obj.left_coord) / 2.0;
        }

        speed_correction = speed_correction * turn_direction;

        left_speed = speed + speed_correction;
        right_speed = speed - speed_correction;
        
        //std::cout << "LSPEED: " << left_speed << " RSPEED: " << right_speed << " dir " << turn_direction << "\n";

        setDriveVoltage(left_speed, right_speed);
        pros::Task::delay(10);
    }

    // brake
    setDriveVoltage(-50, -50);
    if(useLT || useBumper) {
        pros::delay(100);
    } else {
        pros::delay(50);
    }

    // set drive to neutral
    setDriveVoltage(0, 0);
}

void AutonUtils::setDriveSpeed(int leftSpeed, int rightSpeed) {
    mtrDefs->left_mtr_t->move(leftSpeed);
    mtrDefs->left_mtr_b->move(leftSpeed);
    mtrDefs->right_mtr_t->move(rightSpeed);
    mtrDefs->right_mtr_b->move(rightSpeed);
}

void AutonUtils::distanceTranslate(int speed, int unitsLeft) {
    setDriveSpeed(speed, speed);
    while (ultrasonic.get_value() > unitsLeft + 40) {
        std::cout << ultrasonic.get_value() / 10 << std::endl;
        pros::Task::delay(10);
    }
    setDriveSpeed(-speed/2, -speed/2);
    pros::Task::delay(100);
    setDriveSpeed(0, 0);
}

void AutonUtils::pidGlobalTurn(double angle) {
    double imu_cur = sensors->imu->get_heading();

    double angleTurningRight = std::fmod((360.0 - imu_cur + angle), 360.0);
    double angleTurningLeft = 360.0 - angleTurningRight;

    if (angleTurningLeft < angleTurningRight) {
        pidRotate(angle, -1);
    } else if (angleTurningRight <= angleTurningLeft) {
        pidRotate(angle, 1);
    }
}

void AutonUtils::pidRotate(double angle, int direction) {
    double speed;
    double imu_cur;
    double error = determineError(imu_cur, angle, direction);
    double prev_error = 0.0;

    double integral = 0.0;
    double integral_cap = 40.0;
    double derivative;

    // Used to determine when to exit the loop
    bool started = false;
    int iter = 10;
    double angle_to_start = 3.0;

    // PID CONSTANTS
    double kP = 3.2;
    double kI = 0.25;
    double kD = 40.0;

    while (true) {
        std::cout << sensors->imu->get_heading() << "\n";
        imu_cur = sensors->imu->get_heading();

        error = determineError(imu_cur, angle, direction);

        // If the error is within a threshold(angle_to_start) or the floor of the error is equal to angle_to_start,
        // start the countdown in order to preserve the sanity of humanity.
        if (fabs(error) < angle_to_start || (fabs(error) > angle_to_start && floor(fabs(error)) == angle_to_start)) {
            started = true;
        }

        if(iter == 0) {
            break;
        } else if (started) {
            iter--;
        }

        integral += error;
        derivative = error - prev_error;
        prev_error = error;

        if (integral > integral_cap) {
            integral = integral_cap;
        } else if (integral < -integral_cap) {
            integral = -integral_cap;
        }

        speed = error * kP + integral * kI + derivative * kD;
        setDriveSpeed(speed * direction, speed * -direction);

        pros::Task::delay(5);
    }
    setDriveSpeed(0, 0);
}

double AutonUtils::determineError(double imu_cur, double imu_desired, int direction) {
    double angleTurningRight = std::fmod((360.0 - imu_cur + imu_desired), 360.0);
    double angleTurningLeft = 360.0 - angleTurningRight;
    double error;

    if (direction == 1) {
        if (angleTurningRight < angleTurningLeft) {
            error = angleTurningRight;
        } else {
            error = -angleTurningLeft;
        }
    } else {
        if (angleTurningLeft < angleTurningRight) {
            error = angleTurningLeft;
        } else {
            error = -angleTurningRight;
        }
    }

    return error;
}


// TODO: 


void AutonUtils::filter() {
    mtrDefs->roller_b->move(-80);
    mtrDefs->roller_t->move(80);
    // pros::Task::delay(750);
    while (!ballAtBottom()) {}
    mtrDefs->roller_b->move(0);
    mtrDefs->roller_t->move(0);
    pros::Task::delay(50);
}

void AutonUtils::indexTwoBalls(void* param) {
    MotorDefs* mtrDefs = (MotorDefs*)param;
    while(indexTwoBallsTask->notify_take(true, TIMEOUT_MAX)) {
        mtrDefs->roller_t->move(-80);
        mtrDefs->roller_b->move(-80);
        while(!ballAtTop()) {

        }
        mtrDefs->roller_t->move(-15);
        while(!ballAtMid()) {

        }
        mtrDefs->roller_b->move(0);
    }
}

void AutonUtils::filterAndIndexTwoBalls(void* param) {
    MotorDefs* mtrDefs = (MotorDefs*)param;
    while(filterAndIndexTwoBallsTask->notify_take(true, TIMEOUT_MAX)) {
        // Filter until a ball is detected in the intake
        mtrDefs->roller_b->move(-127);
        mtrDefs->roller_t->move(127);
        while (!ballAtBottom()) {}
        mtrDefs->roller_b->move(0);
        mtrDefs->roller_t->move(0);

        // Start the rollers so that we can index the newly picked up ball.
        mtrDefs->roller_t->move(-80);
        mtrDefs->roller_b->move(-80);
        while(!ballAtTop()) {

        }
        mtrDefs->roller_t->move(-15);
        while(!ballAtMid()) {

        }
        mtrDefs->roller_b->move(0);
    }
}

void AutonUtils::filterAndIndexOneBall(void* param) {
    MotorDefs* mtrDefs = (MotorDefs*)param;
    while(filterAndIndexOneBallTask->notify_take(true, TIMEOUT_MAX)) {
        mtrDefs->roller_b->move(-127);
        mtrDefs->roller_t->move(127);
        while (!ballAtBottom()) {}
        mtrDefs->roller_b->move(0);
        mtrDefs->roller_t->move(0);

        // Start the rollers so that we can index the newly picked up ball.
        mtrDefs->roller_t->move(-80);
        mtrDefs->roller_b->move(-80);
        while(!ballAtTop()) {

        }
        stopRollers(mtrDefs);
        mtrDefs->roller_t->move(-15);
    }
}

void AutonUtils::filterAndIndexMid(void* param) {
    MotorDefs* mtrDefs = (MotorDefs*)param;
    while(filterAndIndexMidTask->notify_take(true, TIMEOUT_MAX)) {
        mtrDefs->roller_b->move(-127);
        mtrDefs->roller_t->move(127);
        while (!ballAtBottom()) {}
        mtrDefs->roller_b->move(0);
        mtrDefs->roller_t->move(0);

        // Start the rollers so that we can index the newly picked up ball.
        mtrDefs->roller_t->move(-80);
        mtrDefs->roller_b->move(-80);
        while(!ballAtMid()) {

        }
        stopRollers(mtrDefs);
    }
}

void AutonUtils::indexMid(void* param) {
    MotorDefs* mtrDefs = (MotorDefs*)param;
    while(indexMidTask->notify_take(true, TIMEOUT_MAX)) {
        // Start the rollers so that we can index the newly picked up ball.
        mtrDefs->roller_b->move(-80);
        while(!ballAtMid()) {

        }
        mtrDefs->roller_b->move(0);
    }
}

void AutonUtils::indexTwoBallsWithIntake(void* param) {
    MotorDefs* mtrDefs = (MotorDefs*)param;
    while(indexMidTask->notify_take(true, TIMEOUT_MAX)) {
        mtrDefs->intake_l->move(127);
        mtrDefs->intake_r->move(-127);
        

        mtrDefs->roller_t->move(-80);
        mtrDefs->roller_b->move(-80);
        while(!ballAtTop()) {

        }
        mtrDefs->roller_t->move(0);
        while(!ballAtMid()) {

        }
        mtrDefs->roller_b->move(0);

        mtrDefs->intake_l->move(0);
        mtrDefs->intake_r->move(0);
   }
}

void AutonUtils::indexOneTop(void* param) {
    MotorDefs* mtrDefs = (MotorDefs*)param;
     while(filterTask->notify_take(true, TIMEOUT_MAX)) {
         // Start the rollers so that we can index the newly picked up ball.
        mtrDefs->roller_t->move(-80);
        mtrDefs->roller_b->move(-80);
        while(!ballAtTop()) {

        }
        stopRollers(mtrDefs);
     }
}

void AutonUtils::indexTopBlocking(MotorDefs *mtrDefs) {
    mtrDefs->roller_t->move(-80);
    mtrDefs->roller_b->move(-80);
    while(!ballAtTop()) {

    }
    stopRollers(mtrDefs);
}


void AutonUtils::filter(void* param) {
    MotorDefs* mtrDefs = (MotorDefs*)param;
    // Todo: Make it so that filtering is only finished when
    // the middle line sensor is not blocked.
    while(filterTask->notify_take(true, TIMEOUT_MAX)) {
        mtrDefs->roller_b->move(-80);
        mtrDefs->roller_t->move(80);
        pros::Task::delay(750);
        mtrDefs->roller_b->move(0);
        mtrDefs->roller_t->move(0);
        pros::Task::delay(10);
    }
}


// ----------------------------------------

void AutonUtils::startIntakes(MotorDefs* mtrDefs) {
    mtrDefs->intake_l->move(127);
    mtrDefs->intake_r->move(-127);
}

void AutonUtils::startIntakesSlow(MotorDefs* mtrDefs) {
    mtrDefs->intake_l->move(40);
    mtrDefs->intake_r->move(-40);
}

void AutonUtils::startOuttake(MotorDefs* mtrDefs) {
    mtrDefs->intake_l->move(-80);
    mtrDefs->intake_r->move(80);
}

void AutonUtils::startOuttakeFast(MotorDefs* mtrDefs) {
    mtrDefs->intake_l->move(-127);
    mtrDefs->intake_r->move(127);
}

void AutonUtils::stopIntakes(MotorDefs* mtrDefs) {
    mtrDefs->intake_l->move(0);
    mtrDefs->intake_r->move(0);
}

void AutonUtils::startRollers(MotorDefs* mtrDefs) {
    mtrDefs->roller_t->move(-127);
	mtrDefs->roller_b->move(-127);
}

void AutonUtils::startRollersForDoubleShot(MotorDefs* mtrDefs) {
    mtrDefs->roller_t->move(-127);
    mtrDefs->roller_b->move(-80);
}

void AutonUtils::stopRollers(MotorDefs* mtrDefs) {
    mtrDefs->roller_t->move(0);
    mtrDefs->roller_b->move(0);
}

void AutonUtils::setIndexingOneBall(bool value) {
    indexingOneBall = value;
}

bool AutonUtils::ballAtTop() {
   return line_top.get_value() < 2800;
}

bool AutonUtils::ballAtMid() {
    return line_mid.get_value() < 2750;
}

bool AutonUtils::ballAtBottom() {
    return line_bot.get_value() < 2800;
}

bool AutonUtils::blueBallInFilteringPos(Sensors *sensors) {
    sensors->optical->set_led_pwm(100);
    bool isBlueBall = sensors->optical->get_hue() > 175.0 && sensors->optical->get_hue() < 290.0;
    sensors->optical->set_led_pwm(0);
    return isBlueBall;
}



void AutonUtils::oneShot() {
    mtrDefs->roller_t->move(-127);
    mtrDefs->roller_b->move(20);
    while (limit_t.get_value() && line_top.get_value() <= 2800) {
        pros::Task::delay(10);
    }
    pros::Task::delay(300);
    mtrDefs->roller_b->move(0);
    mtrDefs->roller_t->move(0);
} 

void AutonUtils::slowOneShot() {
    mtrDefs->roller_t->move(-100);
    while (limit_t.get_value() && line_top.get_value() <= 2800) {
        pros::Task::delay(10);
    }
    pros::Task::delay(300);
    mtrDefs->roller_b->move(0);
    mtrDefs->roller_t->move(0);
}

void AutonUtils::shootBalls(void* param) {
    // This function assumes that the balls in shooting positions are red.
    MotorDefs* mtrDefs = (MotorDefs*)param;
    while(shootBallsTask->notify_take(true, TIMEOUT_MAX)) {
        if (ballAtTop() || ballAtMid()) {
            if(!ballAtMid() && ballAtTop()) {
                // Oneshot, start the top roller to shoot
                mtrDefs->roller_t->move(-127);
                // Wait until the limit switch is released (while it is pressed, wait)
                while(limit_t.get_value()) {
                    // pros::Task::delay(2);
                }
                // Wait a little bit, then shut off power.
                pros::Task::delay(400);
                mtrDefs->roller_t->move(0);
            } else if (ballAtTop() && ballAtMid()) {
                // Twoshot
                startRollers(mtrDefs);
                // Wait until the first ball is shot out (While it is pressed, wait)
                // (Until the limit switch is released, wait)
                while(limit_t.get_value()) {
                    // pros::Task::delay(2);
                }
                // Wait until the second ball triggers the switch (while it is not pressed, wait)
                while(!limit_t.get_value()) {
                    // pros::Task::delay(2);
                }
                // Turn off the bottom roller so that blue balls do not get indexed.
                mtrDefs->roller_b->move(0);
                // Wait until the second ball releases the switch (while it is pressed, wait)
                while(limit_t.get_value()) {
                    // pros::Task::delay(2);
                }
                // The second ball has triggered the switch at this point
                // Wait a little bit until the ball has left the top spot, then shut off power.
                pros::Task::delay(400);
                stopRollers(mtrDefs);
            }
        }
        // shootBallsTask->notify_clear();
        // shootingDoneMutex.give();
        // pros::Task::delay(10);
    }
    std::cout << "Outside while." << std::endl;
}

void AutonUtils::cornerGoalSequence() {
    mtrDefs->roller_t->move(-107);
    pros::Task::delay(400);
    mtrDefs->roller_b->move(-80);
    startIntakes(mtrDefs);
    while (!ballAtBottom()) {
        
    }
    mtrDefs->roller_b->move(0);
    setDriveSpeed(-40, -40);
    pros::Task::delay(200);
    setDriveSpeed(40, 40);
    pros::Task::delay(200);
    setDriveSpeed(0, 0);
    
    while(!ballAtMid()) {

    }

    stopRollers(mtrDefs);

    startOuttake(mtrDefs);
    mtrDefs->roller_t->move(30);
    translate(-400, TRANSLATE_VOLTAGE);
    mtrDefs->roller_t->move(0);
    pros::Task::delay(300);
    stopIntakes(mtrDefs);
}

void AutonUtils::twoInOneOut(int moveBackDistance, double heading) {
    mtrDefs->roller_t->move(-100);
    pros::Task::delay(400);
    mtrDefs->roller_b->move(-80);
    
    // Wait until the limit switch is released (while it is pressed, wait)
    while(limit_t.get_value()) {
        
    }

    // Wait until the limit switch is pressed (while it is released, wait)
    while(!limit_t.get_value()) {
        
    }
   
    startIntakes(mtrDefs);
    while (!ballAtBottom()) {
        
    }

    stopIntakes(mtrDefs);
   
    while(!ballAtMid()) {

    }
    
    stopRollers(mtrDefs);

    // Ensure that the blue ball comes to the right spot to filter
    mtrDefs->roller_t->move(30);
    translate(moveBackDistance, TRANSLATE_VOLTAGE, heading);
    mtrDefs->roller_t->move(0);
}


void AutonUtils::nonCornerGoalSequence(int moveBackDistance, double heading) {
    startIntakes(mtrDefs);
    while (!ballAtBottom()) {
        
    }
    stopIntakes(mtrDefs);
    mtrDefs->roller_t->move(-100);
    mtrDefs->roller_b->move(-80);
    while(!ballAtMid()) {

    }
    stopRollers(mtrDefs);
    // Ensure that the blue ball comes to the right spot to filter
    mtrDefs->roller_t->move(30);
    translate(moveBackDistance, TRANSLATE_VOLTAGE, heading);
    mtrDefs->roller_t->move(0);
}

void AutonUtils::centerSequence() {
    // for (int i = 0; i < 2; i++) {
    //     translate(-300, 50, 0.0, false);
    //     pros::Task::delay(50);

    //     setDriveVoltage(50, 50);
    //     pros::Task::delay(500);
    //     setDriveVoltage(0, 0);
    // }

    // setDriveVoltage(0, 50);
    // mtrDefs->roller_t->move(-127);
    // mtrDefs->roller_b->move(-127);
    // startOuttake(mtrDefs);
    // pros::Task::delay(1000);
    // translate(-650, TRANSLATE_VOLTAGE, 0.0);
    // mtrDefs->roller_t->move(0);
    // mtrDefs->roller_b->move(0);

    setDriveVoltage(-10, -10);
    startRollers(mtrDefs);
    startIntakes(mtrDefs);
    pros::Task::delay(300);
    while(!ballAtTop()) {

    }
    stopRollers(mtrDefs);
    stopIntakes(mtrDefs);

    startIntakesSlow(mtrDefs);
    translate(-300, TRANSLATE_VOLTAGE, -1.0, false);


}