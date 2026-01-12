/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023 All rights reserved.                */
/*    Licensed under the MIT license.                                         */
/*                                                                            */
/*    Module:     ai_functions.cpp                                            */
/*    Author:     VEX Robotics Inc.                                           */
/*    Created:    11 August 2023                                              */
/*    Description:  Header for AI robot movement functions                    */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <vex.h>
#include <robot-config.h>
#include <vector>

// Minimum probability threshold for valid detections
#define MIN_DETECTION_PROBABILITY 0.70
// Camera configuration
#define CAMERA_WIDTH 640
#define CAMERA_CENTER_X 320
#define ALIGN_TOLERANCE 50  // pixels - can be adjusted
// Approach configuration
#define APPROACH_MAX_SPEED 50.0     // Maximum approach speed %
#define APPROACH_MIN_SPEED 15.0     // Minimum speed near target %
#define APPROACH_SLOWDOWN_DISTANCE 50.0  // Distance in cm to start slowing down

#define MAX_OBJECT_THRESHOLD 6 // Maximum number of balls


enum OBJECT {
    BallBlue,
    BallRed
};

using namespace vex;

// Calculates the distance to a given target (x, y)
double distanceTo(double target_x, double target_y);

// Moves the robot to a specified position and orientation
void moveToPosition(double target_x, double target_y, double target_theta);

// Finds a target object based on the specified type
DETECTION_OBJECT findTarget(OBJECT type);

// Drives to the closest specified object
void goToObject(OBJECT type);

// Turns the robot to a specific angle with given tolerance and speed
void turnTo(double angle, int tolerance, int speed);

// Drives the robot in a specified heading for a given distance and speed
void driveFor(int heading, double distance, int speed);

void runIntake(vex::directionType dir);
void runIntake(vex::directionType dir, int rotations, bool driveForward);

void stopIntake();

// Check captured ball color using optical sensor hue.
// Returns true if matches desiredColor; false if wrong color (function will eject the ball).
bool checkCapturedBallColor(OBJECT desiredColor);

void goToGoal();

void findAndPickupBall(OBJECT ballType = OBJECT::BallBlue, double approachDistance = 10.0);

void tunepidtest();

void stopDrivetrain();

bool checkAndStopIfBallCaptured();

// Search and find ball based on color
bool FindObject(OBJECT ballType, float maxRotation = 360.0, float searchSpeed = 30.0);

// Allign ball with camera center
bool AlignToObject(OBJECT ballType, float alignSpeed = 20.0, float timeout = 5000.0);

// Function declaration
bool ApproachObject(OBJECT ballType, float stopDistance = 15.0);

bool ScoreBall();

// Find, collect, and score a ball
bool CollectAndScore(OBJECT ballType);

// Optical sensor with masking
bool getOpticalReading();
bool detectOpticalRisingEdge();
void ignoreCurrentBall();
void detectNextBall();

// Emergency stop
void emergencyStop();

int collectMultipleBalls(OBJECT ballType, int targetCount);
static double normalizeAngle180(double a);
// Ramp / storage counting helpers (user can update count externally if desired)
int getRampBallCount();
void setRampBallCount(int count);
void incrementRampBallCount(int delta = 1);

// Select top-K detections by score (same scoring as selectBestBall)
std::vector<DETECTION_OBJECT> selectTopBalls(const std::vector<DETECTION_OBJECT> &detections, double robot_x, double robot_y, double robot_heading, int k);

// Score on specific goals on specific side
void ScoreLGB1();
void ScoreLGB2();
void ScoreLGR1();
void ScoreLGR2();
void ScoreMUB();
void ScoreMUR();
void ScoreMLB();
void ScoreMLR();

// Display GPS Info
void displayGPSPosition();
void debugGPSHeadings();

// Find Goal Coordinates
void findGoalCoordinates();

void initialize();

void testWaypoints();

// Select the best detected ball from a list of detections.
// - `detections`: vector of DETECTION_OBJECT returned by Jetson
// - `robot_x`, `robot_y`: robot position in cm
// - `robot_heading`: robot heading in degrees
// Returns the best DETECTION_OBJECT (default object if none found)
DETECTION_OBJECT selectBestBall(const std::vector<DETECTION_OBJECT> &detections, double robot_x, double robot_y, double robot_heading);

// Test A* path planning and waypoint following.
// Prompts user to input target coordinates (via Controller display/input).
// Gets current GPS position, computes obstacle-free path using A* to the target,
// then drives the robot along the computed waypoints.
// Can be called from usercontrol() for testing path planning before integration.
void testPathPlanning();

// Test pneumatic pistons: initializes to safe state and toggles each piston twice.
void pistontest();

// Isolation macro: autonomous routine for isolating and scoring balls in the corner.
void Isolation(OBJECT ballColor);

// Interaction macro: autonomous routine for interaction period.
void Interaction(OBJECT ballColor);

// Parking macro: navigate to closer of two alliance-specific coordinates and park.
void Parking(OBJECT ballColor);