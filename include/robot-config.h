#include "vex.h"

using namespace vex;

extern brain Brain;
extern controller Controller;
// Sensors
// extern inertial Inertial;
extern rotation ForwardTracker;
extern gps LGPS;
extern gps RGPS;
extern DualGPS GPS;
extern optical IntakeSensor;
extern optical OutSensor;
extern optical MiddleSensor;
// Left Drive
extern motor LeftDriveA;
extern motor LeftDriveB;
extern motor LeftDriveC;
extern motor_group LeftDrive;
// Right Drive
extern motor RightDriveA;
extern motor RightDriveB;
extern motor RightDriveC;
extern motor_group RightDrive;
//Drivetrain a.k.a Chassis
extern smartdrive Drivetrain;
// Ramp
extern motor Ramp;
// Intake
extern motor Intake;
// Score
extern motor Score;
// Pneumatics
extern digital_out ScoreMiddle;
extern digital_out MatchLoader;
extern digital_out BallStopper;

//extern Drive chassis;

/*
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
