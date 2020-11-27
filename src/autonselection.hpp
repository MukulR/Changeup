#ifndef _AUTONSELECTION_H_
#define _AUTONSELECTION_H_

extern int autonSelected;
extern bool redAlliance;

namespace pros {
    class Imu;
    class Task;
    class Vision;
    class Optical;
    class Distance;
}

extern pros::Imu *inertial_sensor;
extern pros::Vision *vision_sensor;
extern pros::Optical *optical_sensor;
extern pros::Distance *distance_sensor_l;
extern pros::Distance *distance_sensor_r;

extern pros::Task *indexTopTask;
extern pros::Task *indexMidTask;
extern pros::Task *filterTask;
extern pros::Task *shootBallsTask;
extern pros::Task *backUpAndOuttakeTask;
extern pros::Task *moveForwardAndFilterTask;
extern pros::Task *indexTask;


#endif //_AUTONSELECTION_H_