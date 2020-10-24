#ifndef _USER_CONTROL_H_
#define _USER_CONTROL_H_

void driveRobot(double degrees, int speed, void* param);
void driveRobotBack(double degrees, int speed, void* param);
void turnWithoutSensors(double degrees, void* param);

#endif //_USER_CONTROL_H_

