#ifndef _AUTONSELECTION_H_
#define _AUTONSELECTION_H_

extern int autonSelected;
extern bool redAlliance;

namespace pros {
    class Imu;
    class Task;
}

extern pros::Imu *inertial_sensor;

extern pros::Task *filterTask;
extern pros::Task *shootBallsTask;
extern pros::Task *backUpAndOuttakeTask;
extern pros::Task *moveForwardAndFilterTask;
extern pros::Task *indexMidTask;
extern pros::Task *indexTwoBallsTask;
extern pros::Task *filterAndIndexTwoBallsTask;
extern pros::Task *filterAndIndexOneBallTask;


#endif //_AUTONSELECTION_H_