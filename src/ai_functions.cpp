/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023 All rights reserved.                */
/*    Licensed under the MIT license.                                         */
/*                                                                            */
/*    Module:     ai_functions.cpp                                            */
/*    Author:     VEX Robotics Inc.                                           */
/*    Created:    11 August 2023                                              */
/*    Description:  Helper movement functions for VEX AI program              */
/*                                                                            */
/*----------------------------------------------------------------------------*/


#include "vex.h"
#include "ai_functions.h"
#include "field_map.h"
#include "astar.h"
#include <string>
#include <iostream>
#include <vector>
#include <algorithm>
#include <robot-config.h>
using namespace vex;
using namespace std;
using std::cout;

// Disambiguate wait() to vex::wait()
#define wait vex::wait

digital_out ScoreMiddle = digital_out(Brain.ThreeWirePort.A);
digital_out MatchLoader = digital_out(Brain.ThreeWirePort.B);
digital_out BallStopper = digital_out(Brain.ThreeWirePort.C);

// Calculates the distance to the coordinates from the current robot position
double distanceTo(double target_x, double target_y){
    double distance = sqrt(pow((target_x - GPS.xPosition()), 2) + pow((target_y - GPS.yPosition()), 2));
    return distance;
}

// Calculates the bearing to drive to the target coordinates in a straight line aligned with global coordinate/heading system.
double calculateBearing(double currX, double currY, double targetX, double targetY) {
    // Calculate the difference in coordinates
    double dx = targetX - currX;
    double dy = targetY - currY;

    // Calculate the bearing in radians
    double bearing_rad = atan2(dy, dx);

    // Convert to degrees
    double bearing_deg = bearing_rad * 180 / M_PI;

    // Normalize to the range 0 to 360
    if (bearing_deg < 0) {
        bearing_deg += 360;
    }

    // Convert from mathematical to navigation coordinates
    bearing_deg = fmod(90 - bearing_deg, 360);
    if (bearing_deg < 0) {
        bearing_deg += 360;
    }

    return bearing_deg;
}

// Turns the robot to face the angle specified, taking into account a tolerance and speed of turn.
void turnTo(double angle/*, int tolerance, int speed*/){
    chassis.turn_to_angle(angle);
    // double current_heading = GPS.heading();
    // double angle_to_turn = angle - current_heading;

    // // Normalize the angle to the range [-180, 180]
    // while (angle_to_turn > 180) angle_to_turn -= 360;
    // while (angle_to_turn < -180) angle_to_turn += 360;

    // // Determine the direction to turn (left or right)
    // turnType direction = angle_to_turn > 0 ? turnType::right : turnType::left;
    // // Drivetrain.turn(direction, speed, velocityUnits::pct);
    // chassis.turn_to_angle(angle);
    // while (1) {
    
    //     current_heading = GPS.heading();
    //     // Check if the current heading is within a tolerance of degrees to the target
    //     if (current_heading > (angle - tolerance) && current_heading < (angle + tolerance)) {
    //         break;
    //     }

    // }
    // Drivetrain.stop();
}

// Moves the robot toward the target at the specificed heading, for a distance at a given speed.
void driveFor(int heading, double distance, int speed){
    // Determine the smallest degree of turn
    double angle_to_turn = heading - GPS.heading();
    while (angle_to_turn > 180) angle_to_turn -= 360;
    while (angle_to_turn < -180) angle_to_turn += 360;

    // Decide whether to move forward or backward
    // Allos for a 5 degree margin of error that defaults to forward
    directionType direction = fwd;
    if (std::abs(angle_to_turn) > 105) {
        angle_to_turn += angle_to_turn > 0 ? -180 : 180;
        direction = directionType::rev;
    } else if (std::abs(angle_to_turn) < 75) {
        angle_to_turn += angle_to_turn > 0 ? 180 : -180;
        direction = directionType::fwd;
    }

    // Drivetrain.driveFor(direction, distance, vex::distanceUnits::cm, speed, velocityUnits::pct);
    if (direction == directionType::rev) {
        chassis.drive_distance(-distance);
    } else {
        chassis.drive_distance(distance);
    }
}

// Method that moves to a given (x,y) position and a desired target theta to finish movement facing
void moveToPosition(double target_x, double target_y, double target_theta = -1) {
    // Calculate the angle to turn to face the target
    double initialHeading = calculateBearing(GPS.xPosition(), GPS.yPosition(), target_x, target_y);
    // Turn to face the target
    //turnTo(intialHeading, 3, 10);
    // Drivetrain.turnToHeading(initialHeading, rotationUnits::deg, 10, velocityUnits::pct);
    chassis.turn_to_angle(initialHeading);
    double distance = distanceTo(target_x, target_y);
    // Move to the target, only 30% of total distance to account for error
    driveFor(initialHeading, distance*0.3, 30);

    // Recalculate the heading and distance to the target
    double heading = calculateBearing(GPS.xPosition(), GPS.yPosition(), target_x, target_y);
    //turnTo(heading, 3, 10);
    // Drivetrain.turnToHeading(heading, rotationUnits::deg, 10, velocityUnits::pct);
    chassis.turn_to_angle(heading);
    distance = distanceTo(target_x, target_y);
    // Move to the target, completing the remaining distance
    driveFor(heading, distance, 20);

    // Turn to the final target heading if specified, otherwise use current heading
    // if (target_theta == -1){
    //     target_theta = GPS.heading();
    // }
    //turnTo(target_theta, 2, 10);
    // Drivetrain.turnToHeading(target_theta, rotationUnits::deg, 10, velocityUnits::pct);
    //chassis.turn_to_angle(target_theta);
}

// Function to find the target object based on type and return its record
DETECTION_OBJECT findTarget(OBJECT type){
    DETECTION_OBJECT target;
    static AI_RECORD local_map;
    jetson_comms.get_data(&local_map);
    double lowestDist = 1000000;
    // Iterate through detected objects to find the closest target of the specified type
    for(int i = 0; i < local_map.detectionCount; i++) {
        double distance = distanceTo(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y);
        if (distance < lowestDist && local_map.detections[i].classID == (int) type) {
            target = local_map.detections[i];
            lowestDist = distance;
        }
    }
    return target;
}

// Function to drive to an object based on detection
void goToObject(OBJECT type){
    DETECTION_OBJECT target = findTarget(type);
    // If no target found, turn and try to find again
    if (target.mapLocation.x == 0 && target.mapLocation.y == 0){
        //Drivetrain.turnFor(45, rotationUnits::deg, 50, velocityUnits::pct);
        // Drivetrain.turn(turnType::left);
        chassis.turn_to_angle(GPS.heading() + 45);
        wait(2, sec);
        // Drivetrain.stop();
        LeftDrive.stop();
        RightDrive.stop();
        target = findTarget(type);
    }
    // Move to the detected target's position
    moveToPosition(target.mapLocation.x*100, target.mapLocation.y*100);
}

void runIntake(vex::directionType dir) {
    Intake.spin(fwd);
    Ramp.spin(fwd);
    Score.spin(fwd);
}

void runIntake(vex::directionType dir, int rotations, bool driveForward = false) {
    Ramp.spinFor(vex::directionType::rev, 12, vex::rotationUnits::rev, false);
    Score.spinFor(vex::directionType::rev, 12, vex::rotationUnits::rev, !driveForward);
    if (driveForward)
        chassis.drive_distance(70);
        // Drivetrain.driveFor(70, vex::distanceUnits::cm, 40, velocityUnits::pct);
}

void stopIntake() {
    Ramp.stop();
    Score.stop();
}

// Check captured ball color using optical sensor hue.
// Returns true if matches desiredColor; false if wrong color (function will eject the ball).
bool checkCapturedBallColor(OBJECT desiredColor) {
    // Run intake to pull ball in, wait for optical sensor to detect it
    Intake.setVelocity(100, percent);
    Intake.spin(vex::directionType::fwd);

    timer t;
    t.clear();
    const int DETECT_TIMEOUT_MS = 2000; // timeout if ball not detected
    while (!IntakeSensor.isNearObject() && t.time() < DETECT_TIMEOUT_MS) {
        wait(20, msec);
    }

    // If not detected, stop intake and return false
    if (!IntakeSensor.isNearObject()) {
        Intake.stop();
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1,1);
        Controller.Screen.print("No ball detected");
        return false;
    }

    // Small settle then read hue
    wait(120, msec);
    int h = IntakeSensor.hue();
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1,1);
    Controller.Screen.print("Hue: %d", h);

    // Broad hue thresholds (tune on-field)
    bool isBlue = (h >= 140 && h <= 260);
    bool isRed  = (h <= 25 || h >= 330);

    if ((desiredColor == OBJECT::BallBlue && isBlue) || (desiredColor == OBJECT::BallRed && isRed)) {
        // Right color: stop intake and keep ball
        Intake.stop();
        Controller.Screen.setCursor(2,1);
        Controller.Screen.print("Right alliance color");
        return true;
    } else {
        // Wrong color: eject by running ramp+score to push ball out
        Controller.Screen.setCursor(2,1);
        Controller.Screen.print("Wrong color - eject");
        Intake.stop();
        Ramp.setVelocity(100, percent);
        Score.setVelocity(100, percent);
        Ramp.spin(vex::directionType::fwd);
        Score.spin(vex::directionType::fwd);
        wait(1000, msec);
        Ramp.stop();
        Score.stop();
        return false;
    }
}

void goToGoal() {
    int closestGoalX = 0;
    int closestGoalY = 0;
    int heading = 0;

    if (distanceTo(122, 0) < distanceTo(-122, 0)) {
        closestGoalX = 122;
        heading = 90;
    } else {
        closestGoalX = -122;
        heading = 270;
    }
    if (distanceTo(0, 122) < distanceTo(0, -122)) {
        closestGoalY = 122;
    } else {
        closestGoalY = -122;
    }

    moveToPosition(closestGoalX, closestGoalY, heading);

}

/**
 * Stops the drivetrain motors immediately.
 * Clean, reusable function for emergency stops.
 */
void stopDrivetrain() {
    chassis.drive_with_voltage(0, 0);
}

/**
 * Checks if optical sensor detects a ball in the intake.
 * If detected, stops both drivetrain and intake immediately.
 * 
 * @return true if ball detected and stopped, false if no ball
 */
bool checkAndStopIfBallCaptured() {
    if (IntakeSensor.isNearObject()) {
        // Ball detected! Emergency stop everything
        stopDrivetrain();
        Ramp.stop();
        
        // Show success message
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Ball Secured!");
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Optical Detected");
        
        return true;
    }
    
    return false;  // No ball detected
}

/**
 * Global variables for optical sensor masking
 */
static bool ignoreSensorReading = false;
static bool opticalPreviousState = false;

/**
 * Get the optical sensor reading with masking support.
 * If ignoring is enabled, always returns FALSE regardless of physical sensor.
 */
bool getOpticalReading() {
    if (ignoreSensorReading) {
        return false;  // FORCE FALSE - ball doesn't exist!
    }
    return IntakeSensor.isNearObject();  // Actual sensor reading
}

/**
 * Detects rising edge on optical sensor using masked reading.
 */
bool detectOpticalRisingEdge() {
    bool currentState = getOpticalReading();  // Use masked reading
    
    // Check for rising edge
    bool edge = (currentState && !opticalPreviousState);
    
    // Update previous state
    opticalPreviousState = currentState;
    
    return edge;
}

/**
 * Start ignoring the current ball at the sensor.
 * After calling this, getOpticalReading() returns FALSE even if ball is present.
 * This makes the ball "invisible" to edge detection.
 */
void ignoreCurrentBall() {
    ignoreSensorReading = true;
    
    // Force both previous and current to FALSE
    opticalPreviousState = false;
}

/**
 * Stop ignoring sensor (allow detection of next ball).
 */
void detectNextBall() {
    ignoreSensorReading = false;
}

// -------------------- Ramp count helpers --------------------
// Tracks how many balls are currently stored in the ramp/stacker.
// This is an estimate counter that can be updated by other code when
// balls are loaded/unloaded. Default 0.
static int rampBallCount = 0;

int getRampBallCount() {
    return rampBallCount;
}

void setRampBallCount(int count) {
    if (count < 0) count = 0;
    if (count > MAX_OBJECT_THRESHOLD) count = MAX_OBJECT_THRESHOLD;
    rampBallCount = count;
}

void incrementRampBallCount(int delta) {
    setRampBallCount(rampBallCount + delta);
}

// Normalize angle to [-180,180]
static double normalizeAngle180(double a) {
    while (a > 180.0) a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
}

// -------------------- Top-K selection helper --------------------
// Score detections using same heuristic as selectBestBall and return top K.
std::vector<DETECTION_OBJECT> selectTopBalls(const std::vector<DETECTION_OBJECT> &detections, double robot_x, double robot_y, double robot_heading, int k) {
    std::vector<std::pair<double, DETECTION_OBJECT>> scored;
    FieldMap fm;
    fm.populateStandardField();

    for (const auto &d : detections) {
        if (d.probability < MIN_DETECTION_PROBABILITY) continue;
        double tx = d.mapLocation.x * 100.0;
        double ty = d.mapLocation.y * 100.0;
        double dx = tx - robot_x;
        double dy = ty - robot_y;
        double dist = sqrt(dx*dx + dy*dy);
        double target_bearing = calculateBearing(robot_x, robot_y, tx, ty);
        double heading_err = fabs(normalizeAngle180(target_bearing - robot_heading));
        bool clear = !fm.lineIntersectsObstacles(robot_x, robot_y, tx, ty);

        double prob_score = d.probability;
        double dist_score = 1.0 - std::min(dist / 1000.0, 1.0);
        double angle_score = 1.0 - (heading_err / 180.0);
        double clear_score = clear ? 1.0 : 0.0;

        const double w_prob = 0.40;
        const double w_dist = 0.30;
        const double w_angle = 0.20;
        const double w_clear = 0.10;

        double score = w_prob * prob_score + w_dist * dist_score + w_angle * angle_score + w_clear * clear_score;
        scored.push_back({score, d});
    }

    // sort descending
    std::sort(scored.begin(), scored.end(), [](const std::pair<double, DETECTION_OBJECT> &a, const std::pair<double, DETECTION_OBJECT> &b){ return a.first > b.first; });

    std::vector<DETECTION_OBJECT> out;
    for (size_t i = 0; i < scored.size() && (int)out.size() < k; ++i) {
        out.push_back(scored[i].second);
    }
    return out;
}

/**
 * Emergency stop with hard braking.
 */
void emergencyStop() {
    chassis.drive_with_voltage(0, 0);
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
    Ramp.stop(hold);
    Intake.stop(hold);
}

/**
 * Searches for a ball by rotating the robot and scanning with Jetson camera.
 * Robot turns clockwise while checking for valid ball detections.
 * Properly handles rotation across 360° boundary.
 * 
 * @param ballType The type of ball to search for (BallBlue or BallRed)
 * @param maxRotation Maximum degrees to rotate during search (default 360°)
 * @param searchSpeed Rotation speed as percentage (default 30%)
 * @return true if ball found, false if not found after maxRotation
 */
bool FindObject(OBJECT ballType, float maxRotation, float searchSpeed) {
    // Map OBJECT enum to Jetson classID
    // classID: 0 = BallBlue, 1 = BallRed
    int targetClassID = (ballType == OBJECT::BallBlue) ? 0 : 1;
    // -----------------------------------------------
    // Get starting heading
    float startHeading = chassis.get_absolute_heading();
    float previousHeading = startHeading;
    float totalRotation = 0.0;  // Accumulate total rotation
    // -----------------------------------------------
    // Convert search speed percentage to voltage
    float turnVoltage = (searchSpeed / 100.0) * 12.0;
    // -----------------------------------------------
    // Storage for Jetson data
    AI_RECORD local_map;
    // -----------------------------------------------
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Searching...");
    // -----------------------------------------------
    // Start rotating clockwise
    // -----------------------------------------------
    // Use STEP & DWELL method for reliable detection
    // Rotate in small steps, pause at each step to let Jetson process
    // -----------------------------------------------
    float stepSize = 15.0;  // Rotate 15° at a time
    int stepsPerRotation = (int)(maxRotation / stepSize);
    // -----------------------------------------------
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Searching...");
    // -----------------------------------------------
    for (int step = 0; step < stepsPerRotation; step++) {
        // Calculate target angle for this step
        float targetAngle = startHeading + (step * stepSize);
        
        // Normalize to 0-360 range
        if (targetAngle >= 360) targetAngle -= 360;
        if (targetAngle < 0) targetAngle += 360;
        // -----------------------------------------------
        // Rotate to target angle
        chassis.turn_to_angle(targetAngle);
        // -----------------------------------------------
        // DWELL: Pause to let robot settle and Jetson process
        wait(150, msec);
        // -----------------------------------------------
        // Check detections multiple times during dwell
        for (int check = 0; check < 3; check++) {
            jetson_comms.get_data(&local_map);
            // -----------------------------------------------
            // Check all detections in current frame
            for (int i = 0; i < local_map.detectionCount; i++) {
                DETECTION_OBJECT detection = local_map.detections[i];
                // -----------------------------------------------
                // Check if this is the correct ball type with sufficient confidence
                if (detection.classID == targetClassID && 
                    detection.probability >= MIN_DETECTION_PROBABILITY) {
                    // -----------------------------------------------
                    // Ball found! Stop the robot
                    chassis.drive_with_voltage(0, 0);
                    // -----------------------------------------------
                    Controller.Screen.clearScreen();
                    Controller.Screen.setCursor(1, 1);
                    Controller.Screen.print("Ball Found!");
                    Controller.Screen.setCursor(2, 1);
                    Controller.Screen.print("Prob: %.0f%%", detection.probability * 100);
                    Controller.Screen.setCursor(3, 1);
                    Controller.Screen.print("At step %d", step);
                    // -----------------------------------------------
                    wait(500, msec);
                    return true;
                }
            }
            // -----------------------------------------------
            wait(33, msec);  // Wait one frame (~30 FPS)
        }
        // -----------------------------------------------
        // Update display with current progress
        float currentRotation = step * stepSize;
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Searching...");
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Step: %d/%d", step + 1, stepsPerRotation);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("Detects: %d", local_map.detectionCount);
    }
    // -----------------------------------------------
    // Search complete, ball not found
    chassis.drive_with_voltage(0, 0);
    // -----------------------------------------------
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Ball Not Found");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Searched 360 deg");
    // -----------------------------------------------
    return false;
}

// // Normalize angle to [-180,180]
// static double normalizeAngle180(double a) {
//     while (a > 180.0) a -= 360.0;
//     while (a < -180.0) a += 360.0;
//     return a;
// }

// Select the best detection from a list using a simple heuristic:
// score = w_prob*prob + w_dist*(1 - dist/1000) + w_angle*(1 - angle_err/180) + w_clear*(clear?1:0)
DETECTION_OBJECT selectBestBall(const std::vector<DETECTION_OBJECT> &detections, double robot_x, double robot_y, double robot_heading) {
    DETECTION_OBJECT best;
    bool any = false;
    double bestScore = -1e9;

    // prepare a FieldMap to check line-of-sight (uses populateStandardField defaults)
    FieldMap fm;
    fm.populateStandardField();

    for (const auto &d : detections) {
        // ignore low-confidence detections
        if (d.probability < MIN_DETECTION_PROBABILITY) continue;

        // mapLocation appears to be in meters in other code; convert to cm
        double tx = d.mapLocation.x * 100.0;
        double ty = d.mapLocation.y * 100.0;

        double dx = tx - robot_x;
        double dy = ty - robot_y;
        double dist = sqrt(dx*dx + dy*dy); // cm

        // bearing from robot to target in our global convention
        double target_bearing = calculateBearing(robot_x, robot_y, tx, ty);
        double heading_err = fabs(normalizeAngle180(target_bearing - robot_heading));

        // obstacle check
        bool clear = !fm.lineIntersectsObstacles(robot_x, robot_y, tx, ty);

        // scoring components
        double prob_score = d.probability; // 0..1
        double dist_score = 1.0 - std::min(dist / 1000.0, 1.0); // prefer closer (10m cutoff)
        double angle_score = 1.0 - (heading_err / 180.0); // 1.0 if aligned, 0 if opposite
        double clear_score = clear ? 1.0 : 0.0;

        // weights (tunable)
        const double w_prob = 0.40;
        const double w_dist = 0.30;
        const double w_angle = 0.20;
        const double w_clear = 0.10;

        double score = w_prob * prob_score + w_dist * dist_score + w_angle * angle_score + w_clear * clear_score;

        if (!any || score > bestScore) {
            best = d;
            bestScore = score;
            any = true;
        }
    }

    return best;
}

/** AlignToObject()-4
 * Aligns robot to center a detected object in camera view.
 * Assumes object is already visible in camera (use FindObject first if needed).
 * Uses adaptive speed control and "close enough" logic to avoid oscillation.
 * 
 * @param ballType The type of ball to align to (BallBlue or BallRed)
 * @param maxAlignSpeed Maximum rotation speed percentage (default 30%)
 * @param timeout Maximum time in ms to attempt alignment (default 5000ms)
 * @return true if aligned successfully, false if object lost or timeout
 */
bool AlignToObject(OBJECT ballType, float maxAlignSpeed, float timeout) {
    // Map OBJECT enum to Jetson classID
    int targetClassID = (ballType == OBJECT::BallBlue) ? 0 : 1;
    // -----------------------------------------------
    // Storage for Jetson data
    AI_RECORD local_map;
    // -----------------------------------------------
    // ===== CHECK IF ALREADY ALIGNED FIRST =====
    jetson_comms.get_data(&local_map);
    for (int i = 0; i < local_map.detectionCount; i++) {
        DETECTION_OBJECT detection = local_map.detections[i];
        if (detection.classID == targetClassID && 
            detection.probability >= MIN_DETECTION_PROBABILITY) {
            int initialX = detection.screenLocation.x;
            int initialError = initialX - CAMERA_CENTER_X;
            // -----------------------------------------------
            // Already aligned! Return immediately
            if (abs(initialError) <= ALIGN_TOLERANCE) {
                Controller.Screen.clearScreen();
                Controller.Screen.setCursor(1, 1);
                Controller.Screen.print("Already Aligned!");
                Controller.Screen.setCursor(2, 1);
                Controller.Screen.print("Error: %d", initialError);
                wait(300, msec);
                return true;
            }
            break;
        }
    }
    // -----------------------------------------------
    // Start timer for timeout
    timer alignTimer;
    alignTimer.clear();
    // -----------------------------------------------
    bool objectFound = false;
    int objectX = 0;
    int lostFrameCount = 0;
    const int MAX_LOST_FRAMES = 5;
    // -----------------------------------------------
    // "Close enough" tracking
    int timeCloseToAligned = 0;
    const int CLOSE_THRESHOLD = 80;  // Increased from 60 - more forgiving
    const int CLOSE_TIME_NEEDED = 200;  // Reduced from 300ms - faster accept
    // -----------------------------------------------
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Aligning...");
    // -----------------------------------------------
    while (alignTimer.time() < timeout) {
        jetson_comms.get_data(&local_map);
        objectFound = false;
        // -----------------------------------------------
        for (int i = 0; i < local_map.detectionCount; i++) {
            DETECTION_OBJECT detection = local_map.detections[i];
            // -----------------------------------------------
            if (detection.classID == targetClassID && 
                detection.probability >= MIN_DETECTION_PROBABILITY) {
                objectFound = true;
                objectX = detection.screenLocation.x;
                lostFrameCount = 0;
                break;
            }
        }
        // -----------------------------------------------
        if (!objectFound) {
            lostFrameCount++;
            int maxAllowedLostFrames = (alignTimer.time() < 1000) ? 20 : MAX_LOST_FRAMES;
            // -----------------------------------------------
            if (lostFrameCount > maxAllowedLostFrames) {
                chassis.drive_with_voltage(0, 0);
                // -----------------------------------------------
                Controller.Screen.clearScreen();
                Controller.Screen.setCursor(1, 1);
                Controller.Screen.print("Object Lost!");
                Controller.Screen.setCursor(2, 1);
                Controller.Screen.print("Frames: %d", lostFrameCount);
                // -----------------------------------------------
                return false;
            }
            // -----------------------------------------------
            chassis.drive_with_voltage(0, 0);
            wait(20, msec);
            continue;
        }
        // -----------------------------------------------
        int error = objectX - CAMERA_CENTER_X;
        // -----------------------------------------------
        // Perfectly aligned
        if (abs(error) <= ALIGN_TOLERANCE) {
            chassis.drive_with_voltage(0, 0);
            // -----------------------------------------------
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Aligned!");
            Controller.Screen.setCursor(2, 1);
            Controller.Screen.print("X:%d Error:%d", objectX, error);
            // -----------------------------------------------
            return true;
        }
        // -----------------------------------------------
        // Close enough tracking
        if (abs(error) <= CLOSE_THRESHOLD) {
            timeCloseToAligned += 20;
            // -----------------------------------------------
            if (timeCloseToAligned >= CLOSE_TIME_NEEDED) {
                chassis.drive_with_voltage(0, 0);
                // -----------------------------------------------
                Controller.Screen.clearScreen();
                Controller.Screen.setCursor(1, 1);
                Controller.Screen.print("Close Enough!");
                Controller.Screen.setCursor(2, 1);
                Controller.Screen.print("X:%d Error:%d", objectX, error);
                // -----------------------------------------------
                return true;
            }
        } else {
            timeCloseToAligned = 0;
        }
        // -----------------------------------------------
        // === FIXED SPEED CALCULATION ===
        float speedScale = (float)abs(error) / (CAMERA_WIDTH / 2.0);
        // -----------------------------------------------
        // INCREASED minimum speed from 6% to 20%
        float minSpeed = 20.0;  // Motors actually move now!
        float turnSpeed = minSpeed + (speedScale * (maxAlignSpeed - minSpeed));
        // -----------------------------------------------
        // Less aggressive slowdown when close
        if (abs(error) < 80) {
            turnSpeed = turnSpeed * 0.75;  // 75% instead of 60%
            if (turnSpeed < 15.0) {  // Minimum 15% instead of 5%
                turnSpeed = 15.0;
            }
        }
        // -----------------------------------------------
        if (turnSpeed > maxAlignSpeed) {
            turnSpeed = maxAlignSpeed;
        }
        // -----------------------------------------------
        float turnVoltage = (turnSpeed / 100.0) * 12.0;
        // -----------------------------------------------
        if (error > 0) {
            chassis.drive_with_voltage(turnVoltage, -turnVoltage);
        } else {
            chassis.drive_with_voltage(-turnVoltage, turnVoltage);
        }
        // -----------------------------------------------
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Aligning...");
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Error:%d Speed:%.1f", error, turnSpeed);
        // -----------------------------------------------
        wait(20, msec);
    }
    // -----------------------------------------------
    chassis.drive_with_voltage(0, 0);
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Align Timeout!");
    // -----------------------------------------------
    return false;
}

// ApproachObject()-7
bool ApproachObject(OBJECT ballType, float stopDistance) {
    int targetClassID = (ballType == OBJECT::BallBlue) ? 0 : 1;
    // -----------------------------------------------
    AI_RECORD local_map;
    int lostFrameCount = 0;
    const int MAX_LOST_FRAMES = 200;
    bool objectFound = false;
    float currentDepth = 0.0;
    float lastKnownDepth = 1.0;
    // -----------------------------------------------
    // Enable detection for this ball
    detectNextBall();
    // -----------------------------------------------
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Approaching...");
    // -----------------------------------------------
    while (true) {
        // CHECK FOR RISING EDGE: ball entered intake optical sensor
        if (detectOpticalRisingEdge()) {
            // stop drivetrain and intake
            emergencyStop();
            // increment internal ramp counter
            incrementRampBallCount(1);
            // pulse the ramp/score slightly to move ball up past mechanical stop
            Ramp.setVelocity(80, percent);
            Score.setVelocity(80, percent);
            Ramp.spin(fwd);
            Score.spin(fwd);
            wait(300, msec);
            Ramp.stop(hold);
            Score.stop(hold);

            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Ball Secured!");
            // small settle
            wait(200, msec);
            return true;
        }
        // -----------------------------------------------
        // Jetson detection
        jetson_comms.get_data(&local_map);
        objectFound = false;
        int ballX = 0;
        // -----------------------------------------------
        for (int i = 0; i < local_map.detectionCount; i++) {
            DETECTION_OBJECT detection = local_map.detections[i];
            // -----------------------------------------------
            if (detection.classID == targetClassID && 
                detection.probability >= MIN_DETECTION_PROBABILITY) {
                objectFound = true;
                currentDepth = detection.depth;
                lastKnownDepth = currentDepth;
                ballX = detection.screenLocation.x;
                lostFrameCount = 0;
                break;
            }
        }
        // -----------------------------------------------
        if (!objectFound) {
            lostFrameCount++;
            float lastDistanceCm = lastKnownDepth * 100.0;
            // -----------------------------------------------
            if (lastDistanceCm <= 30.0 || 
                (lostFrameCount > 15 && lastDistanceCm <= 50.0)) {
                emergencyStop();
                return true;
            }
            // -----------------------------------------------
            if (lostFrameCount > MAX_LOST_FRAMES) {
                emergencyStop();
                return false;
            }
            // -----------------------------------------------
            currentDepth = lastKnownDepth;
            wait(5, msec);
            continue;
        }
        // -----------------------------------------------
        float distanceCm = currentDepth * 100.0;
        // -----------------------------------------------
        // Calculate base speed
        float driveSpeed;
        if (distanceCm > APPROACH_SLOWDOWN_DISTANCE) {
            driveSpeed = APPROACH_MAX_SPEED;
        } else {
            float speedRange = APPROACH_MAX_SPEED - APPROACH_MIN_SPEED;
            float speedScale = distanceCm / APPROACH_SLOWDOWN_DISTANCE;
            driveSpeed = APPROACH_MIN_SPEED + (speedScale * speedRange);
            if (driveSpeed > APPROACH_MAX_SPEED) driveSpeed = APPROACH_MAX_SPEED;
            if (driveSpeed < APPROACH_MIN_SPEED) driveSpeed = APPROACH_MIN_SPEED;
        }
        // -----------------------------------------------
        // GENTLE STEERING CORRECTION
        int offset = ballX - CAMERA_CENTER_X;
        // -----------------------------------------------
        // Deadzone - don't correct small errors
        const int STEERING_DEADZONE = 30;  // Ignore errors < 30 pixels
        float steeringCorrection = 0.0;
        
        if (abs(offset) > STEERING_DEADZONE) {
            // Much smaller gain - 0.003 instead of 0.02
            float steeringGain = 0.003;
            steeringCorrection = offset * steeringGain;
            
            // Limit maximum correction to 1.5 volts
            if (steeringCorrection > 1.5) steeringCorrection = 1.5;
            if (steeringCorrection < -1.5) steeringCorrection = -1.5;
        }
        // -----------------------------------------------
        // Apply steering
        float baseVoltage = (driveSpeed / 100.0) * 12.0;
        float leftVoltage = baseVoltage + steeringCorrection;
        float rightVoltage = baseVoltage - steeringCorrection;
        // -----------------------------------------------
        // Limit voltages
        if (leftVoltage > 12.0) leftVoltage = 12.0;
        if (leftVoltage < -12.0) leftVoltage = -12.0;
        if (rightVoltage > 12.0) rightVoltage = 12.0;
        if (rightVoltage < -12.0) rightVoltage = -12.0;
        // -----------------------------------------------
        chassis.drive_with_voltage(leftVoltage, rightVoltage);
        // -----------------------------------------------
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Approaching...");
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Dist: %.1fcm", distanceCm);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("Off:%d Corr:%.2f", offset, steeringCorrection);
        // -----------------------------------------------
        wait(5, msec);
    }
}

/**
 * Scores the ball that's currently in the intake.
 * Runs intake/scoring mechanism to deposit ball.
 * 
 * @return true if scored successfully
 */
bool ScoreBall() {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Scoring...");
    // -----------------------------------------------
    // Run scoring mechanism (adjust direction/time as needed)
    Ramp.spin(vex::directionType::rev);  // Or forward, depending on your mechanism
    Score.spin(fwd);   // Run scoring motor
    // -----------------------------------------------
    wait(1000, msec);  // Run for 1 second to score
    // -----------------------------------------------
    Ramp.stop();
    Score.stop();
    // -----------------------------------------------
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Ball Scored!");
    // -----------------------------------------------
    return true;
}

/**
 * Complete sequence: Find → Align → Approach → Collect → Score
 * 
 * @param ballType Ball color to collect (BallBlue or BallRed)
 * @return true if entire sequence successful
 */
bool CollectAndScore(OBJECT ballType) {
    // STEP 1: Find
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Finding ball...");
    // -----------------------------------------------
    bool found = FindObject(ballType, 360.0, 30.0);
    if (!found) {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Ball Not Found");
        return false;
    }
    // -----------------------------------------------
    wait(300, msec);
    // -----------------------------------------------
    // STEP 2: Align
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Aligning...");
    // -----------------------------------------------
    bool aligned = AlignToObject(ballType, 30.0, 5000.0);
    if (!aligned) {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Align Failed");
        return false;
    }
    // -----------------------------------------------
    wait(300, msec);
    // -----------------------------------------------
    // STEP 3: Approach & Collect
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Collecting...");
    // -----------------------------------------------
    Ramp.spin(fwd);
    bool collected = ApproachObject(ballType, 15.0);
    // Intake stopped by optical sensor or ApproachObject
    // -----------------------------------------------
    if (!collected) {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Collection Failed");
        return false;
    }
    // -----------------------------------------------
    wait(500, msec);
    // -----------------------------------------------
    // STEP 4: Score (if you have a goal location)
    bool scored = ScoreBall();
    // -----------------------------------------------
    if (scored) {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("SUCCESS!");
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Ball Scored");
        return true;
    }
    // -----------------------------------------------
    return false;
}

/**
 * Master function to collect multiple balls automatically.
 * Handles the complete sequence: Find → Align → Approach → Collect
 * Updates counter in real-time and ignores previous balls.
 * 
 * @param ballType Ball color to collect (BallBlue or BallRed)
 * @param targetCount Number of balls to collect (1-10)
 * @return Number of balls successfully collected
 */
int collectMultipleBalls(OBJECT ballType, int targetCount) {
    // Respect the requested targetCount but clamp to reasonable bounds
    if (targetCount < 1) targetCount = 1;
    if (targetCount > MAX_OBJECT_THRESHOLD) targetCount = MAX_OBJECT_THRESHOLD;
    // Validate input
    if (targetCount < 1) targetCount = 1;
    if (targetCount > 10) targetCount = 10;
    // -----------------------------------------------
    // Display start message
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Starting...");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Target: %d balls", targetCount);
    wait(1000, msec);
    // -----------------------------------------------
    int ballsCollected = 0;
    // -----------------------------------------------
    // Main collection loop
    for (int i = 0; i < targetCount; i++) {
        // Show current status
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Ball %d of %d", i+1, targetCount);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Collected: %d/%d", ballsCollected, targetCount);
        wait(500, msec);
        // -----------------------------------------------
        // ========== STEP 1: FIND ==========
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Finding ball %d...", i+1);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Collected: %d/%d", ballsCollected, targetCount);
        // -----------------------------------------------
        bool found = FindObject(ballType, 360.0, 30.0);
        // -----------------------------------------------
        if (!found) {
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Ball Not Found!");
            Controller.Screen.setCursor(2, 1);
            Controller.Screen.print("Collected: %d/%d", ballsCollected, targetCount);
            Controller.rumble("---");
            wait(3000, msec);
            break;  // Exit loop
        }
        // -----------------------------------------------
        wait(300, msec);
        // -----------------------------------------------
        // ========== STEP 2: ALIGN ==========
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Aligning ball %d...", i+1);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Collected: %d/%d", ballsCollected, targetCount);
        // -----------------------------------------------
        bool aligned = AlignToObject(ballType, 30.0, 5000.0);
        // -----------------------------------------------
        if (!aligned) {
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Align Failed!");
            Controller.Screen.setCursor(2, 1);
            Controller.Screen.print("Collected: %d/%d", ballsCollected, targetCount);
            Controller.rumble("--");
            wait(3000, msec);
            break;  // Exit loop
        }
        // -----------------------------------------------
        wait(300, msec);
        // -----------------------------------------------
        // ========== STEP 3: APPROACH & COLLECT ==========
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Grabbing ball %d...", i+1);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Collected: %d/%d", ballsCollected, targetCount);
        // -----------------------------------------------
        Intake.setVelocity(100, percent);
        Intake.spin(fwd);  // Start intake
        wait(1000, msec);
        bool collected = ApproachObject(ballType, 15.0);
        // Intake stopped by emergencyStop() in ApproachObject
        // -----------------------------------------------
        if (!collected) {
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Approach Failed!");
            Controller.Screen.setCursor(2, 1);
            Controller.Screen.print("Collected: %d/%d", ballsCollected, targetCount);
            Controller.rumble("--");
            wait(3000, msec);
            break;  // Exit loop
        }
        // -----------------------------------------------
        // ========== SUCCESS! INCREMENT COUNTER ==========
        ballsCollected++;
        // -----------------------------------------------
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Ball %d Secured!", i+1);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Collected: %d/%d", ballsCollected, targetCount);
        Controller.rumble(".");  // Success vibration
        wait(500, msec);
        // -----------------------------------------------
        // ========== IGNORE THIS BALL FOR NEXT DETECTION ==========
        ignoreCurrentBall();
        // -----------------------------------------------
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Ball %d ignored", i+1);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Collected: %d/%d", ballsCollected, targetCount);
        wait(500, msec);
        // -----------------------------------------------
        // ========== PULSE RAMP TO MOVE BALLS UP ==========
        if (ballsCollected < targetCount) {
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Stacking...");
            // -----------------------------------------------
            // Pulse RAMP only
            Ramp.setVelocity(70, percent);
            Ramp.spin(fwd);
            wait(250, msec);  // Tune this!
            Ramp.stop(hold);
            // -----------------------------------------------
            wait(300, msec);
        }
        // Check if we're done
        if (ballsCollected >= targetCount) {
            break;
        }
        // -----------------------------------------------
        // Brief pause before next ball
        wait(500, msec);
    }
    // -----------------------------------------------
    // ========== FINAL REPORT ==========
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("COMPLETE!");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Collected: %d/%d", ballsCollected, targetCount);
    // -----------------------------------------------
    if (ballsCollected == targetCount) {
        Controller.rumble(".");  // Full success
    } else {
        Controller.rumble("--");  // Partial success
    }
    // -----------------------------------------------
    wait(3000, msec);
    // -----------------------------------------------
    return ballsCollected;  // Return count for caller to use if needed
}

// Score on Long Goal 1 Blue side
void ScoreLGB1(){
    // This routine: find -> align -> pickup -> face waypoint -> drive to waypoint -> face goal -> back up -> score
    Controller.Screen.clearScreen();
    Controller.Screen.print("Scoring...LGB1");
    wait(200, msec);
    // Waypoint coordinates for Long Goal Blue 1 (from usercontrol originally)
    double waypoint_x_cm = 121.92; // cm
    double waypoint_y_cm = 110.92; // cm
    double face_deg = 90.0; // final facing heading
    double back_distance_cm = 60.0; // back up after facing

    // Emergency stop and prepare
    emergencyStop();
    wait(200, msec);

    // Find the ball by rotating
    bool found = FindObject(OBJECT::BallBlue, 360.0, 30.0);
    if(!found){
        Controller.Screen.clearScreen();
        Controller.Screen.print("Ball Not Found");
        wait(800, msec);
        return;
    }

    // After a successful search decide how many balls to pick based on ramp capacity
    const int RAMP_CAPACITY = MAX_OBJECT_THRESHOLD; // typically 6
    int currentRamp = getRampBallCount();
    int availableSpace = RAMP_CAPACITY - currentRamp;
    if (availableSpace <= 0) {
        Controller.Screen.clearScreen();
        Controller.Screen.print("Ramp Full");
        wait(300, msec);
    } else {
        Controller.Screen.clearScreen();
        Controller.Screen.print("Will collect %d balls", availableSpace);
        wait(200, msec);

        int collectedNow = 0;
        const float STOP_DISTANCE_CM = 12.0f; // approach stop distance

        // Loop: pick best visible ball, approach and capture, repeat until full or none left
        while (collectedNow < availableSpace) {
            // Refresh detections
            AI_RECORD scan_map;
            jetson_comms.get_data(&scan_map);
            std::vector<DETECTION_OBJECT> scan_dets;
            for (int i = 0; i < scan_map.detectionCount; i++) scan_dets.push_back(scan_map.detections[i]);

            DETECTION_OBJECT target = selectBestBall(scan_dets, GPS.xPosition(), GPS.yPosition(), GPS.heading());

            // If no valid target found, attempt a quick search. If still none, stop trying.
            if (target.mapLocation.x == 0 && target.mapLocation.y == 0) {
                bool reacquired = FindObject(OBJECT::BallBlue, 180.0, 30.0);
                if (!reacquired) break;
                wait(200, msec);
                continue; // will re-loop and fetch detections again
            }

            // Turn to face chosen detection via GPS
            double tx = target.mapLocation.x * 100.0;
            double ty = target.mapLocation.y * 100.0;
            double target_bearing = calculateBearing(GPS.xPosition(), GPS.yPosition(), tx, ty);
            chassis.set_heading(GPS.heading());
            chassis.turn_to_angle(target_bearing);
            wait(150, msec);

            // Align to object visually
            OBJECT cls = (target.classID == 0) ? OBJECT::BallBlue : OBJECT::BallRed;
            bool alignedNow = AlignToObject(cls, 25.0, 3000.0);
            if (!alignedNow) {
                // try next best
                continue;
            }

            // Approach and capture
            detectNextBall();
            runIntake(vex::directionType::rev);
            bool captured = ApproachObject(OBJECT::BallBlue, STOP_DISTANCE_CM);
            stopIntake();
            if (captured) {
                incrementRampBallCount(1);
                collectedNow++;
                ignoreCurrentBall();
                // small settle
                wait(200, msec);
            } else {
                // couldn't capture this one; try another
                wait(200, msec);
                continue;
            }
        }
        Controller.Screen.clearScreen();
        Controller.Screen.print("Collected %d", collectedNow);
        wait(300, msec);
    }
    // Use current GPS to face waypoint then drive forward to it
    double curr_x = GPS.xPosition();
    double curr_y = GPS.yPosition();
    double curr_heading = GPS.heading();
    if (!((curr_x == 0.0 && curr_y == 0.0) || isnan(curr_x) || isnan(curr_y) || isnan(curr_heading))) {
        double target_bearing = calculateBearing(curr_x, curr_y, waypoint_x_cm, waypoint_y_cm);
        chassis.set_heading(curr_heading);
        chassis.turn_to_angle(target_bearing);
        wait(200, msec);
        double dx = waypoint_x_cm - curr_x;
        double dy = waypoint_y_cm - curr_y;
        double dist_cm = sqrt(dx*dx + dy*dy);
        double dist_in = dist_cm / 2.54;
        Controller.Screen.clearScreen();
        Controller.Screen.print("Driving fwd: %.1f cm", dist_cm);
        chassis.drive_distance(dist_in);
        wait(200, msec);
    } else {
        // Fallback: move using moveToPosition if GPS bad
        moveToPosition(waypoint_x_cm, waypoint_y_cm);
    }
    // Ensure full stop and settle at waypoint
    emergencyStop();
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
    wait(300, msec);

    // Turn to final face heading and back up
    double stop_h = GPS.heading();
    chassis.set_heading(stop_h);
    chassis.turn_to_angle(face_deg);
    wait(300, msec);
    double back_in = back_distance_cm / 2.54;
    chassis.drive_distance(-back_in);
    wait(300, msec);

    // Score - run ramp and score motors
    Intake.setVelocity(100, percent);
    Ramp.setVelocity(100, percent);
    Score.setVelocity(100, percent);
    Intake.spin(fwd);
    Ramp.spin(fwd);
    Score.spin(fwd);
    wait(10000, msec);
    // Intake.stop();
    // Ramp.stop();
    // Score.stop();
    Controller.Screen.clearScreen();
    Controller.Screen.print("Scored!");
}

// Score on Long Goal 2 Blue side (GPS with NULL and NaN filter)
void ScoreLGB2() {
    // Mirror of ScoreLGB1 but for the (-,-) waypoint
    Controller.Screen.clearScreen();
    Controller.Screen.print("Scoring...LGB2");
    wait(200, msec);
    // Waypoint coordinates mirrored from LGB1 -> (+,-)
    double waypoint_x_cm = 121.92; // cm
    double waypoint_y_cm = -110.92; // cm
    double face_deg = 90.0; // final facing heading
    double back_distance_cm = 60.0; // back up after facing

    // Emergency stop and prepare
    emergencyStop();
    wait(200, msec);

    // Find the ball by rotating
    bool found = FindObject(OBJECT::BallBlue, 360.0, 30.0);
    if(!found){
        Controller.Screen.clearScreen();
        Controller.Screen.print("Ball Not Found");
        wait(800, msec);
        return;
    }

    // After a successful search decide how many balls to pick based on ramp capacity
    const int RAMP_CAPACITY = MAX_OBJECT_THRESHOLD; // typically 6
    int currentRamp = getRampBallCount();
    int availableSpace = RAMP_CAPACITY - currentRamp;
    if (availableSpace <= 0) {
        Controller.Screen.clearScreen();
        Controller.Screen.print("Ramp Full");
        wait(300, msec);
    } else {
        Controller.Screen.clearScreen();
        Controller.Screen.print("Will collect %d balls", availableSpace);
        wait(200, msec);

        int collectedNow = 0;
        const float STOP_DISTANCE_CM = 12.0f; // approach stop distance

        // Loop: pick best visible ball, approach and capture, repeat until full or none left
        while (collectedNow < availableSpace) {
            // Refresh detections
            AI_RECORD scan_map;
            jetson_comms.get_data(&scan_map);
            std::vector<DETECTION_OBJECT> scan_dets;
            for (int i = 0; i < scan_map.detectionCount; i++) scan_dets.push_back(scan_map.detections[i]);

            DETECTION_OBJECT target = selectBestBall(scan_dets, GPS.xPosition(), GPS.yPosition(), GPS.heading());

            // If no valid target found, attempt a quick search. If still none, stop trying.
            if (target.mapLocation.x == 0 && target.mapLocation.y == 0) {
                bool reacquired = FindObject(OBJECT::BallBlue, 180.0, 30.0);
                if (!reacquired) break;
                wait(200, msec);
                continue; // will re-loop and fetch detections again
            }

            // Turn to face chosen detection via GPS
            double tx = target.mapLocation.x * 100.0;
            double ty = target.mapLocation.y * 100.0;
            double target_bearing = calculateBearing(GPS.xPosition(), GPS.yPosition(), tx, ty);
            chassis.set_heading(GPS.heading());
            chassis.turn_to_angle(target_bearing);
            wait(150, msec);

            // Align to object visually
            OBJECT cls = (target.classID == 0) ? OBJECT::BallBlue : OBJECT::BallRed;
            bool alignedNow = AlignToObject(cls, 25.0, 3000.0);
            if (!alignedNow) {
                // try next best
                continue;
            }

            // Approach and capture
            detectNextBall();
            runIntake(vex::directionType::rev);
            bool captured = ApproachObject(OBJECT::BallBlue, STOP_DISTANCE_CM);
            stopIntake();
            if (captured) {
                // check color using optical sensor hue; only count if correct alliance color
                bool rightColor = checkCapturedBallColor(cls);
                if (rightColor) {
                    incrementRampBallCount(1);
                    collectedNow++;
                    ignoreCurrentBall();
                    // small settle
                    wait(200, msec);
                } else {
                    // wrong color was ejected; try next ball
                    wait(300, msec);
                    continue;
                }
            } else {
                // couldn't capture this one; try another
                wait(200, msec);
                continue;
            }
        }
        Controller.Screen.clearScreen();
        Controller.Screen.print("Collected %d", collectedNow);
        wait(300, msec);
    }
    // Use current GPS to face waypoint then drive forward to it
    double curr_x = GPS.xPosition();
    double curr_y = GPS.yPosition();
    double curr_heading = GPS.heading();
    if (!((curr_x == 0.0 && curr_y == 0.0) || isnan(curr_x) || isnan(curr_y) || isnan(curr_heading))) {
        double target_bearing = calculateBearing(curr_x, curr_y, waypoint_x_cm, waypoint_y_cm);
        chassis.set_heading(curr_heading);
        chassis.turn_to_angle(target_bearing);
        wait(200, msec);
        double dx = waypoint_x_cm - curr_x;
        double dy = waypoint_y_cm - curr_y;
        double dist_cm = sqrt(dx*dx + dy*dy);
        double dist_in = dist_cm / 2.54;
        Controller.Screen.clearScreen();
        Controller.Screen.print("Driving fwd: %.1f cm", dist_cm);
        chassis.drive_distance(dist_in);
        wait(200, msec);
    } else {
        // Fallback: move using moveToPosition if GPS bad
        moveToPosition(waypoint_x_cm, waypoint_y_cm);
    }
    // Ensure full stop and settle at waypoint
    emergencyStop();
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
    wait(300, msec);

    // Turn to final face heading and back up
    double stop_h = GPS.heading();
    chassis.set_heading(stop_h);
    chassis.turn_to_angle(face_deg);
    wait(300, msec);
    double back_in = back_distance_cm / 2.54;
    chassis.drive_distance(-back_in);
    wait(300, msec);

    // Score - run ramp and score motors
    Intake.setVelocity(100, percent);
    Ramp.setVelocity(100, percent);
    Score.setVelocity(100, percent);
    Intake.spin(fwd);
    Ramp.spin(fwd);
    Score.spin(fwd);
    wait(10000, msec);
    // Intake.stop();
    // Ramp.stop();
    // Score.stop();
    Controller.Screen.clearScreen();
    Controller.Screen.print("Scored!");
}

// Score on Long Goal 1 Red side
void ScoreLGR1(){
    // Mirror of ScoreLGB1 for (-,+) waypoint
    Controller.Screen.clearScreen();
    Controller.Screen.print("Scoring...LGR1");
    wait(200, msec);
    // Waypoint coordinates for Long Goal Red 1
    double waypoint_x_cm = -121.92; // cm
    double waypoint_y_cm = 110.92; // cm
    double face_deg = 90.0; // final facing heading
    double back_distance_cm = 60.0; // back up after facing

    // Emergency stop and prepare
    emergencyStop();
    wait(200, msec);

    // Find the ball by rotating
    bool found = FindObject(OBJECT::BallBlue, 360.0, 30.0);
    if(!found){
        Controller.Screen.clearScreen();
        Controller.Screen.print("Ball Not Found");
        wait(800, msec);
        return;
    }

    // After a successful search decide how many balls to pick based on ramp capacity
    const int RAMP_CAPACITY = MAX_OBJECT_THRESHOLD; // typically 6
    int currentRamp = getRampBallCount();
    int availableSpace = RAMP_CAPACITY - currentRamp;
    if (availableSpace <= 0) {
        Controller.Screen.clearScreen();
        Controller.Screen.print("Ramp Full");
        wait(300, msec);
    } else {
        Controller.Screen.clearScreen();
        Controller.Screen.print("Will collect %d balls", availableSpace);
        wait(200, msec);

        int collectedNow = 0;
        const float STOP_DISTANCE_CM = 12.0f; // approach stop distance

        // Loop: pick best visible ball, approach and capture, repeat until full or none left
        while (collectedNow < availableSpace) {
            // Refresh detections
            AI_RECORD scan_map;
            jetson_comms.get_data(&scan_map);
            std::vector<DETECTION_OBJECT> scan_dets;
            for (int i = 0; i < scan_map.detectionCount; i++) scan_dets.push_back(scan_map.detections[i]);

            DETECTION_OBJECT target = selectBestBall(scan_dets, GPS.xPosition(), GPS.yPosition(), GPS.heading());

            // If no valid target found, attempt a quick search. If still none, stop trying.
            if (target.mapLocation.x == 0 && target.mapLocation.y == 0) {
                bool reacquired = FindObject(OBJECT::BallBlue, 180.0, 30.0);
                if (!reacquired) break;
                wait(200, msec);
                continue; // will re-loop and fetch detections again
            }

            // Turn to face chosen detection via GPS
            double tx = target.mapLocation.x * 100.0;
            double ty = target.mapLocation.y * 100.0;
            double target_bearing = calculateBearing(GPS.xPosition(), GPS.yPosition(), tx, ty);
            chassis.set_heading(GPS.heading());
            chassis.turn_to_angle(target_bearing);
            wait(150, msec);

            // Align to object visually
            OBJECT cls = (target.classID == 0) ? OBJECT::BallBlue : OBJECT::BallRed;
            bool alignedNow = AlignToObject(cls, 25.0, 3000.0);
            if (!alignedNow) {
                // try next best
                continue;
            }

            // Approach and capture
            detectNextBall();
            runIntake(vex::directionType::rev);
            bool captured = ApproachObject(OBJECT::BallBlue, STOP_DISTANCE_CM);
            stopIntake();
            if (captured) {
                // check color using optical sensor hue; only count if correct alliance color
                bool rightColor = checkCapturedBallColor(cls);
                if (rightColor) {
                    incrementRampBallCount(1);
                    collectedNow++;
                    ignoreCurrentBall();
                    // small settle
                    wait(200, msec);
                } else {
                    // wrong color was ejected; try next ball
                    wait(300, msec);
                    continue;
                }
            } else {
                // couldn't capture this one; try another
                wait(200, msec);
                continue;
            }
        }
        Controller.Screen.clearScreen();
        Controller.Screen.print("Collected %d", collectedNow);
        wait(300, msec);
    }
    // Use current GPS to face waypoint then drive forward to it
    double curr_x = GPS.xPosition();
    double curr_y = GPS.yPosition();
    double curr_heading = GPS.heading();
    if (!((curr_x == 0.0 && curr_y == 0.0) || isnan(curr_x) || isnan(curr_y) || isnan(curr_heading))) {
        double target_bearing = calculateBearing(curr_x, curr_y, waypoint_x_cm, waypoint_y_cm);
        chassis.set_heading(curr_heading);
        chassis.turn_to_angle(target_bearing);
        wait(200, msec);
        double dx = waypoint_x_cm - curr_x;
        double dy = waypoint_y_cm - curr_y;
        double dist_cm = sqrt(dx*dx + dy*dy);
        double dist_in = dist_cm / 2.54;
        Controller.Screen.clearScreen();
        Controller.Screen.print("Driving fwd: %.1f cm", dist_cm);
        chassis.drive_distance(dist_in);
        wait(200, msec);
    } else {
        // Fallback: move using moveToPosition if GPS bad
        moveToPosition(waypoint_x_cm, waypoint_y_cm);
    }
    // Ensure full stop and settle at waypoint
    emergencyStop();
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
    wait(300, msec);

    // Turn to final face heading and back up
    double stop_h = GPS.heading();
    chassis.set_heading(stop_h);
    chassis.turn_to_angle(face_deg);
    wait(300, msec);
    double back_in = back_distance_cm / 2.54;
    chassis.drive_distance(-back_in);
    wait(300, msec);

    // Score - run ramp and score motors
    Intake.setVelocity(100, percent);
    Ramp.setVelocity(100, percent);
    Score.setVelocity(100, percent);
    Intake.spin(fwd);
    Ramp.spin(fwd);
    Score.spin(fwd);
    wait(10000, msec);
    // Intake.stop();
    // Ramp.stop();
    // Score.stop();
    Controller.Screen.clearScreen();
    Controller.Screen.print("Scored!");
}

// Score on Long Goal 2 Red side
void ScoreLGR2(){
    // Mirror of ScoreLGB1 for (-,-) waypoint
    Controller.Screen.clearScreen();
    Controller.Screen.print("Scoring...LGR2");
    wait(200, msec);
    // Waypoint coordinates for Long Goal Red 2
    double waypoint_x_cm = -121.92; // cm
    double waypoint_y_cm = -110.92; // cm
    double face_deg = 90.0; // final facing heading
    double back_distance_cm = 60.0; // back up after facing

    // Emergency stop and prepare
    emergencyStop();
    wait(200, msec);

    // Find the ball by rotating
    bool found = FindObject(OBJECT::BallBlue, 360.0, 30.0);
    if(!found){
        Controller.Screen.clearScreen();
        Controller.Screen.print("Ball Not Found");
        wait(800, msec);
        return;
    }

    // After a successful search decide how many balls to pick based on ramp capacity
    const int RAMP_CAPACITY = MAX_OBJECT_THRESHOLD; // typically 6
    int currentRamp = getRampBallCount();
    int availableSpace = RAMP_CAPACITY - currentRamp;
    if (availableSpace <= 0) {
        Controller.Screen.clearScreen();
        Controller.Screen.print("Ramp Full");
        wait(300, msec);
    } else {
        Controller.Screen.clearScreen();
        Controller.Screen.print("Will collect %d balls", availableSpace);
        wait(200, msec);

        int collectedNow = 0;
        const float STOP_DISTANCE_CM = 12.0f; // approach stop distance

        // Loop: pick best visible ball, approach and capture, repeat until full or none left
        while (collectedNow < availableSpace) {
            // Refresh detections
            AI_RECORD scan_map;
            jetson_comms.get_data(&scan_map);
            std::vector<DETECTION_OBJECT> scan_dets;
            for (int i = 0; i < scan_map.detectionCount; i++) scan_dets.push_back(scan_map.detections[i]);

            DETECTION_OBJECT target = selectBestBall(scan_dets, GPS.xPosition(), GPS.yPosition(), GPS.heading());

            // If no valid target found, attempt a quick search. If still none, stop trying.
            if (target.mapLocation.x == 0 && target.mapLocation.y == 0) {
                bool reacquired = FindObject(OBJECT::BallBlue, 180.0, 30.0);
                if (!reacquired) break;
                wait(200, msec);
                continue; // will re-loop and fetch detections again
            }

            // Turn to face chosen detection via GPS
            double tx = target.mapLocation.x * 100.0;
            double ty = target.mapLocation.y * 100.0;
            double target_bearing = calculateBearing(GPS.xPosition(), GPS.yPosition(), tx, ty);
            chassis.set_heading(GPS.heading());
            chassis.turn_to_angle(target_bearing);
            wait(150, msec);

            // Align to object visually
            OBJECT cls = (target.classID == 0) ? OBJECT::BallBlue : OBJECT::BallRed;
            bool alignedNow = AlignToObject(cls, 25.0, 3000.0);
            if (!alignedNow) {
                // try next best
                continue;
            }

            // Approach and capture
            detectNextBall();
            runIntake(vex::directionType::rev);
            bool captured = ApproachObject(OBJECT::BallBlue, STOP_DISTANCE_CM);
            stopIntake();
            if (captured) {
                // check color using optical sensor hue; only count if correct alliance color
                bool rightColor = checkCapturedBallColor(cls);
                if (rightColor) {
                    incrementRampBallCount(1);
                    collectedNow++;
                    ignoreCurrentBall();
                    // small settle
                    wait(200, msec);
                } else {
                    // wrong color was ejected; try next ball
                    wait(300, msec);
                    continue;
                }
            } else {
                // couldn't capture this one; try another
                wait(200, msec);
                continue;
            }
        }
        Controller.Screen.clearScreen();
        Controller.Screen.print("Collected %d", collectedNow);
        wait(300, msec);
    }
    // Use current GPS to face waypoint then drive forward to it
    double curr_x = GPS.xPosition();
    double curr_y = GPS.yPosition();
    double curr_heading = GPS.heading();
    if (!((curr_x == 0.0 && curr_y == 0.0) || isnan(curr_x) || isnan(curr_y) || isnan(curr_heading))) {
        double target_bearing = calculateBearing(curr_x, curr_y, waypoint_x_cm, waypoint_y_cm);
        chassis.set_heading(curr_heading);
        chassis.turn_to_angle(target_bearing);
        wait(200, msec);
        double dx = waypoint_x_cm - curr_x;
        double dy = waypoint_y_cm - curr_y;
        double dist_cm = sqrt(dx*dx + dy*dy);
        double dist_in = dist_cm / 2.54;
        Controller.Screen.clearScreen();
        Controller.Screen.print("Driving fwd: %.1f cm", dist_cm);
        chassis.drive_distance(dist_in);
        wait(200, msec);
    } else {
        // Fallback: move using moveToPosition if GPS bad
        moveToPosition(waypoint_x_cm, waypoint_y_cm);
    }
    // Ensure full stop and settle at waypoint
    emergencyStop();
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
    wait(300, msec);

    // Turn to final face heading and back up
    double stop_h = GPS.heading();
    chassis.set_heading(stop_h);
    chassis.turn_to_angle(face_deg);
    wait(300, msec);
    double back_in = back_distance_cm / 2.54;
    chassis.drive_distance(-back_in);
    wait(300, msec);

    // Score - run ramp and score motors
    Intake.setVelocity(100, percent);
    Ramp.setVelocity(100, percent);
    Score.setVelocity(100, percent);
    Intake.spin(fwd);
    Ramp.spin(fwd);
    Score.spin(fwd);
    wait(10000, msec);
    // Intake.stop();
    // Ramp.stop();
    // Score.stop();
    Controller.Screen.clearScreen();
    Controller.Screen.print("Scored!");
}

// Score on Middle Upper Goal Blue side
void ScoreMUB(){
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Scoring...MUB");
    // -----------------------------------------------
    chassis.drive_to_point(48,48);
    wait(500, msec);
    chassis.turn_to_angle(90);
    wait(500, msec);
    chassis.drive_to_point(30,48);
    wait(500, msec);
    // Run scoring mechanism (adjust direction/time as needed)
    Ramp.spin(vex::directionType::rev);  // Or forward, depending on your mechanism
    Score.spin(fwd);   // Run scoring motor
    // -----------------------------------------------
    wait(3000, msec);  // Run for 3 seconds to score
    // -----------------------------------------------
    Ramp.stop();
    Score.stop();
    // -----------------------------------------------
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Ball Scored!");
    // -----------------------------------------------
}

/**
 * Displays current GPS position and heading on controller screen
 * Useful for finding waypoint coordinates
 */
/**
 * Displays STABILIZED GPS position by averaging multiple readings
 * Takes 10 readings over 2 seconds for accuracy
 */
void displayGPSPosition() {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Reading GPS...");
    // Take multiple readings and average them
    const int NUM_READINGS = 10;
    double sum_x = 0;
    double sum_y = 0;
    double sum_h = 0;
    for(int i = 0; i < NUM_READINGS; i++) {
        sum_x += GPS.xPosition();
        sum_y += GPS.yPosition();
        sum_h += GPS.heading();
        wait(200, msec);  // Wait between readings
    }
    // Calculate averages
    double avg_x = sum_x / NUM_READINGS;
    double avg_y = sum_y / NUM_READINGS;
    double avg_h = sum_h / NUM_READINGS;
    double quality = GPS.quality();
    // Display averaged values
    Controller.Screen.clearScreen();
    // Controller.Screen.setCursor(1, 1);
    // Controller.Screen.print("=== GPS (AVG) ===");
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("X: %.2f cm", avg_x);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Y: %.2f cm", avg_y);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("H: %.2f deg", avg_h);
    // Controller.Screen.setCursor(5, 1);
    // Controller.Screen.print("Q: %.1f", quality);
    // // Also print individual sensor quality for debugging
    // Controller.Screen.setCursor(6, 1);
    // Controller.Screen.print("L:%d R:%d", 
    //     GPS.Left_GPS.quality(), 
    //     GPS.Right_GPS.quality());
}

void debugGPSHeadings() {
    while(!Controller.ButtonA.pressing()) {
        double left_h = GPS.Left_GPS.heading();
        double right_h = GPS.Right_GPS.heading();
        double avg_h = GPS.heading();
        double imu_h = chassis.Gyro.heading();
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("LEFT: %.1f", left_h);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("RIGHT: %.1f", right_h);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("AVG: %.1f", avg_h);
        // Controller.Screen.setCursor(4, 1);
        // Controller.Screen.print("IMU: %.1f", imu_h);
        wait(200, msec);
    }
}

void findGoalCoordinates() {
    // Average 20 readings for stability
    const int SAMPLES = 20;
    double sum_x = 0, sum_y = 0;
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Reading GPS...");
    for(int i = 0; i < SAMPLES; i++) {
        sum_x += GPS.xPosition();
        sum_y += GPS.yPosition();
        wait(100, msec);
    }
    double avg_x = sum_x / SAMPLES;
    double avg_y = sum_y / SAMPLES;
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("GOAL COORDS:");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("X: %.2f cm", avg_x);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Y: %.2f cm", avg_y);
    // Keep showing for 30 seconds
    wait(10000, msec);
}

void initialize() {
    // Set starting position: (-48, 15.5), heading 0
    chassis.set_coordinates(-48, 15.5, 0);
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1,1);
    Controller.Screen.print("Position Set");
    wait(500, msec);
    
    // Drive forward 10 inches
    Controller.Screen.setCursor(2,1);
    Controller.Screen.print("Driving 10in...");
    chassis.drive_distance(10);
}

void testWaypoints() {
    // interactive: Up/Down adjust X, Left/Right adjust Y, A=run, B=cancel
    double wp_x = 121.92;
    double wp_y = 121.92;

    Controller.Screen.clearScreen();
    Controller.Screen.print("Test Waypoint");
    wait(200, msec);

    while (true) {
        Controller.Screen.setCursor(1,1);
        Controller.Screen.print("X: %.2f cm", wp_x);
        Controller.Screen.setCursor(2,1);
        Controller.Screen.print("Y: %.2f cm", wp_y);
        Controller.Screen.setCursor(3,1);
        Controller.Screen.print("A=Run  B=Cancel");

        if (Controller.ButtonUp.pressing()) { wp_x += 1.0; wait(150, msec); }
        if (Controller.ButtonDown.pressing()) { wp_x -= 1.0; wait(150, msec); }
        if (Controller.ButtonRight.pressing()) { wp_y += 1.0; wait(150, msec); }
        if (Controller.ButtonLeft.pressing()) { wp_y -= 1.0; wait(150, msec); }

        if (Controller.ButtonA.pressing()) { waitUntil(!Controller.ButtonA.pressing()); break; }
        if (Controller.ButtonB.pressing()) { waitUntil(!Controller.ButtonB.pressing()); Controller.Screen.clearScreen(); Controller.Screen.print("Cancelled"); wait(400, msec); return; }

        wait(50, msec);
    }

    // Read start GPS
    double start_x = GPS.xPosition();
    double start_y = GPS.yPosition();
    double start_h = GPS.heading();
    if ((start_x == 0.0 && start_y == 0.0) || isnan(start_x) || isnan(start_y) || isnan(start_h)) {
        Controller.Screen.clearScreen();
        Controller.Screen.print("Bad GPS - abort");
        wait(800, msec);
        return;
    }

    double bearing = calculateBearing(start_x, start_y, wp_x, wp_y);
    double dx = wp_x - start_x;
    double dy = wp_y - start_y;
    double dist_cm = sqrt(dx*dx + dy*dy);
    double dist_in = dist_cm / 2.54;

    // Turn to waypoint and drive forward
    chassis.set_heading(start_h);
    Controller.Screen.clearScreen(); Controller.Screen.print("Turn: %.1f", bearing);
    chassis.turn_to_angle(bearing);
    wait(150, msec);

    Controller.Screen.clearScreen(); Controller.Screen.print("Drive: %.1f cm", dist_cm);
    chassis.drive_distance(dist_in);

    // settle, stop, and report
    wait(300, msec);
    emergencyStop(); LeftDrive.stop(hold); RightDrive.stop(hold);
    wait(200, msec);

    double final_x = GPS.xPosition();
    double final_y = GPS.yPosition();
    double final_h = GPS.heading();
    double err = sqrt(pow(final_x - wp_x, 2) + pow(final_y - wp_y, 2));

    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1,1);
    Controller.Screen.print("Final X: %.2f", final_x);
    Controller.Screen.setCursor(2,1);
    Controller.Screen.print("Final Y: %.2f", final_y);
    Controller.Screen.setCursor(3,1);
    Controller.Screen.print("H: %.2f Err: %.2fcm", final_h, err);

    cout << "testWaypoints result: final(" << final_x << "," << final_y << ") H=" << final_h << " err=" << err << "cm\n";
    wait(3000, msec);
}

// Test A* path planning and waypoint following
void testPathPlanning() {
    Controller.Screen.clearScreen();
    Controller.Screen.print("A* Path Test");
    wait(200, msec);
    
    // Initialize target coordinates (can be adjusted with arrows)
    double target_x = 0.0;  // cm
    double target_y = 0.0;   // cm
    
    // Allow user to adjust target coordinates dynamically
    bool coordinatesLocked = false;
    
    while (!coordinatesLocked) {
        // Display current target coordinates
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("X: %.1f", target_x);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Y: %.1f", target_y);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("A=Lock");
        
        // Up arrow: increase X
        if (Controller.ButtonUp.pressing()) {
            target_x += 1.0;
        }
        
        // Down arrow: decrease X
        if (Controller.ButtonDown.pressing()) {
            target_x -= 1.0;
        }
        
        // Left arrow: decrease Y
        if (Controller.ButtonLeft.pressing()) {
            target_y -= 1.0;
        }
        
        // Right arrow: increase Y
        if (Controller.ButtonRight.pressing()) {
            target_y += 1.0;
        }
        
        // Button A: lock coordinates and start pathfinding
        if (Controller.ButtonA.pressing()) {
            waitUntil(!Controller.ButtonA.pressing());
            coordinatesLocked = true;
            wait(200, msec);
        }
        
        wait(20, msec);
    }
    
    // Coordinates locked - now get current position and start pathfinding
    Controller.Screen.clearScreen();
    Controller.Screen.print("Getting GPS...");
    wait(300, msec);
    
    double curr_x = GPS.xPosition();
    double curr_y = GPS.yPosition();
    double curr_h = GPS.heading();
    
    if ((curr_x == 0.0 && curr_y == 0.0) || isnan(curr_x) || isnan(curr_y) || isnan(curr_h)) {
        Controller.Screen.clearScreen();
        Controller.Screen.print("Bad GPS - abort");
        wait(800, msec);
        return;
    }
    
    // Display current position and target
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Current: %.1f,%.1f", curr_x, curr_y);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Target: %.1f,%.1f", target_x, target_y);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Planning...");
    wait(500, msec);
    
    // Create and populate FieldMap with obstacles
    FieldMap fieldMap;
    fieldMap.populateStandardField();
    
    // Call A* to find path
    // Increase grid resolution to reduce waypoint count (coarser grid)
    std::vector<astar::Point> path = astar::findPath(
        fieldMap,
        curr_x, curr_y,
        target_x, target_y,
        30.0,   // resolution_cm (was 5.0)
        12.0,   // robot_radius_cm
        3.0     // extra_margin_cm
    );
    
    // Check if path was found
    if (path.empty()) {
        Controller.Screen.clearScreen();
        Controller.Screen.print("No path found!");
        wait(2000, msec);
        return;
    }
    
    // Print entire path to console BEFORE robot moves
    cout << "===== A* PATH PLANNED =====\n";
    cout << "Start: (" << curr_x << "," << curr_y << ") H=" << curr_h << "\n";
    cout << "Target: (" << target_x << "," << target_y << ")\n";
    cout << "Total Waypoints: " << path.size() << "\n";
    cout << "Full Path:\n";
    for (size_t i = 0; i < path.size(); i++) {
        cout << "  WP" << i << ": (" << path[i].first << "," << path[i].second << ")\n";
    }
    cout << "===========================\n";
    
    // Display path info on controller in requested order
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Path planned");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("List all waypoints");
    wait(700, msec);
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Waypoints: %d", (int)path.size());
    wait(1000, msec);
    
    // Follow the path waypoint by waypoint
    for (size_t i = 0; i < path.size(); i++) {
        double wp_x = path[i].first;
        double wp_y = path[i].second;
        
        // Get current position
        double robot_x = GPS.xPosition();
        double robot_y = GPS.yPosition();
        double robot_h = GPS.heading();
        
        if ((robot_x == 0.0 && robot_y == 0.0) || isnan(robot_x) || isnan(robot_y)) {
            cout << "Bad GPS at WP" << i << ", breaking\n";
            break;
        }
        
        // Calculate bearing to waypoint
        double bearing = calculateBearing(robot_x, robot_y, wp_x, wp_y);
        double dx = wp_x - robot_x;
        double dy = wp_y - robot_y;
        double dist_cm = sqrt(dx*dx + dy*dy);
        
        // Skip if already at waypoint
        if (dist_cm < 5.0) {
            cout << "Waypoint " << (int)(i+1) << " (" << wp_x << "," << wp_y << ") reached\n";
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Waypoint %d/%d (%.1f, %.1f) reached", (int)(i+1), (int)path.size(), wp_x, wp_y);
            continue;
        }
        
        // Display waypoint info
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Moving to WP %d/%d", (int)(i+1), (int)path.size());
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Dist: %.1f cm", dist_cm);
        
        // Turn and drive to waypoint
        double dist_in = dist_cm / 2.54;
        chassis.set_heading(robot_h);
        chassis.turn_to_angle(bearing);
        //wait(20, msec);
        chassis.drive_distance(dist_in);
        //wait(20, msec);
        
        // Print waypoint reached to terminal and controller
        cout << "Waypoint " << (int)(i+1) << " (" << wp_x << "," << wp_y << ") reached\n";
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Waypoint %d/%d (%.1f, %.1f) reached", (int)(i+1), (int)path.size(), wp_x, wp_y);
    }
    
    // Final stop and report
    emergencyStop();
    LeftDrive.stop(hold);
    RightDrive.stop(hold);
    wait(300, msec);
    
    double final_x = GPS.xPosition();
    double final_y = GPS.yPosition();
    double final_h = GPS.heading();
    double err = sqrt(pow(final_x - target_x, 2) + pow(final_y - target_y, 2));
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Arrived!");
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Pos: %.1f,%.1f", final_x, final_y);
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("Error: %.1f cm", err);
    
    wait(3000, msec);
}

/**
 * Test function for pneumatic pistons
 * Initializes pistons and toggles each one twice with 1000ms intervals
 */
void pistontest(){
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("PISTON TEST START");
    wait(1000, msec);
    
    // ===== Initialize pneumatics to safe state =====
    MatchLoader.set(false);   // RETRACTED
    ScoreMiddle.set(true);    // EXTENDED
    BallStopper.set(false);   // RETRACTED
    wait(1000, msec);
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Testing MatchLoader");
    
    // ===== Test MatchLoader (toggle twice) =====
    // Toggle 1: RETRACTED -> EXTENDED
    MatchLoader.set(true);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("State: EXTENDED");
    wait(1000, msec);
    
    // Toggle 2: EXTENDED -> RETRACTED
    MatchLoader.set(false);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("State: RETRACTED");
    wait(1000, msec);
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Testing ScoreMiddle");
    
    // ===== Test ScoreMiddle (toggle twice) =====
    // Toggle 1: EXTENDED -> RETRACTED
    ScoreMiddle.set(false);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("State: RETRACTED");
    wait(1000, msec);
    
    // Toggle 2: RETRACTED -> EXTENDED
    ScoreMiddle.set(true);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("State: EXTENDED");
    wait(1000, msec);
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Testing BallStopper");
    
    // ===== Test BallStopper (toggle twice) =====
    // Toggle 1: RETRACTED -> EXTENDED
    BallStopper.set(true);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("State: EXTENDED");
    wait(1000, msec);
    
    // Toggle 2: EXTENDED -> RETRACTED
    BallStopper.set(false);
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("State: RETRACTED");
    wait(1000, msec);
    
    // ===== Return to safe state =====
    MatchLoader.set(false);
    ScoreMiddle.set(true);
    BallStopper.set(false);
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("PISTON TEST COMPLETE");
    wait(2000, msec);
}
// ============= ISOLATION MACRO AND HELPERS =============

/**
 * Helper: Drive forward at full speed for a specified distance.
 * @param distanceInches Distance to drive in inches
 */
void isolationDriveForward(double distanceInches) {
    chassis.drive_distance(distanceInches);
}

/**
 * Helper: Drive backward (reverse) for a specified distance.
 * @param distanceInches Distance to back up in inches
 */
void isolationDriveBackward(double distanceInches) {
    chassis.drive_distance(-distanceInches);
}

/**
 * Helper: Turn to face a specific heading.
 * @param heading Target heading in degrees
 */
void isolationTurnToFace(double heading) {
    chassis.turn_to_angle(heading);
}

/**
 * Helper: Navigate to a specific coordinate using GPS.
 * @param targetX Target X coordinate in cm
 * @param targetY Target Y coordinate in cm
 */
void isolationMoveToCoordinate(double targetX, double targetY) {
    double curr_x = GPS.xPosition();
    double curr_y = GPS.yPosition();
    double curr_h = GPS.heading();
    
    if ((curr_x == 0.0 && curr_y == 0.0) || isnan(curr_x) || isnan(curr_y) || isnan(curr_h)) {
        return;  // Bad GPS, skip
    }
    
    double bearing = calculateBearing(curr_x, curr_y, targetX, targetY);
    double dx = targetX - curr_x;
    double dy = targetY - curr_y;
    double dist_cm = sqrt(dx*dx + dy*dy);
    double dist_in = dist_cm / 2.54;
    
    // Turn and drive to target
    chassis.set_heading(curr_h);
    chassis.turn_to_angle(bearing);
    wait(100, msec);
    chassis.drive_distance(dist_in);
    wait(100, msec);
}

/**
 * Helper: Run Intake, Ramp, and Score motors in forward direction.
 */
void isolationRunMotorsForward() {
    Intake.setVelocity(40, percent);
    Ramp.setVelocity(40, percent);
    Score.setVelocity(40, percent);
    Intake.spin(fwd);
    Ramp.spin(fwd);
    Score.spin(fwd);
}

/**
 * Helper: Run Intake, Ramp, and Score motors in reverse direction.
 */
void isolationRunMotorsReverse() {
    Intake.setVelocity(40, percent);
    Ramp.setVelocity(40, percent);
    Score.setVelocity(40, percent);
    Intake.spin(vex::directionType::rev);
    Ramp.spin(vex::directionType::rev);
    Score.spin(vex::directionType::rev);
}

/**
 * Helper: Stop Intake, Ramp, and Score motors.
 */
void isolationStopMotors() {
    Intake.stop();
    Ramp.stop();
    Score.stop();
}

/**
 * ISOLATION MACRO
 * Autonomous routine for isolating and scoring balls in the corner.
 * 
 * @param ballColor Color of ball alliance (OBJECT::BallBlue or OBJECT::BallRed)
 */
void Isolation(OBJECT ballColor) {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("ISOLATION START");
    wait(200, msec);

    // Clock the macro so we can bail before 15s field cutoff
    timer isolationTimer;
    isolationTimer.clear();

    // Set drivetrain to 50% speed (~5V out of 12V) for controlled collection
    chassis.set_drive_constants(5, 1.5, 0, 10, 0);
    
    // Use shorter timeouts for faster execution within 15s isolation window
    // Format: (settle_error_inches, settle_time_ms, timeout_ms)
    chassis.set_drive_exit_conditions(1.5, 200, 2000);  // 2s timeout per move
    chassis.set_turn_exit_conditions(1, 200, 1500);     // 1.5s timeout per turn
    
    // ===== Initialize counters with edge detection =====
    int intakeSensorCount = 0;      // Count balls entering ramp
    int outSensorCount = 0;         // Count balls passing out
    bool intakeSensorWasDetecting = false;  // Track previous state
    bool outSensorWasDetecting = false;     // Track previous state
    
    // ===== STEP 0: Determine target waypoint based on alliance =====
    // BLUE: (+,-), RED: (-,+)
    double targetX = (ballColor == OBJECT::BallRed) ? -120.0 : 120.0;
    double targetY = (ballColor == OBJECT::BallRed) ? 120.0 : -120.0;
    
    // ===== INITIAL STATE: Set pneumatics =====
    // Piston extended = true, retracted = false
    MatchLoader.set(false);   // RETRACTED
    ScoreMiddle.set(true);    // EXTENDED (stays extended the whole time)
    BallStopper.set(false);   // RETRACTED
    
    // ===== STEP 1: Get current location and navigate to target =====
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("STEP 1: Moving...");
    
    isolationMoveToCoordinate(targetX, targetY);
    wait(300, msec);
    
    // ===== STEP 2: Turn to face goal (90° BLUE, 270° RED) =====
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("STEP 2: Turning...");
    
    double goalFacing = (ballColor == OBJECT::BallRed) ? 270.0 : 90.0;
    isolationTurnToFace(goalFacing);
    wait(300, msec);
    
    // ===== STEP 3: Back up into goal =====
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("STEP 3: Backing up...");
    
    isolationDriveBackward(20);
    wait(300, msec);
    
    // ===== PHASE 1: COLLECT BALLS =====
    // ===== STEP 4: BallStopper EXTENDED, motors spin (60% speed) =====
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("STEP 4: Motors on...");
    
    BallStopper.set(true);   // EXTENDED - will stay extended until first ball passes
    Intake.setVelocity(60, percent);
    Ramp.setVelocity(60, percent);
    Score.setVelocity(60, percent);
    Intake.spin(fwd);
    Ramp.spin(fwd);
    Score.spin(fwd);
    wait(300, msec);

    // ===== STEP 5: MatchLoader EXTENDED, robot moves FORWARD =====
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("STEP 5: Loading...");
    
    MatchLoader.set(true);   // EXTENDED
    wait(100, msec);
    isolationDriveForward(40);
    wait(200, msec);
    
    // ===== STEP 6: Collect balls - wait for first ball to pass OutSensor =====
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("STEP 6: Waiting for ball...");
    
    // Wait until first ball passes OutSensor, then retract BallStopper
    // Cap the wait to 1.5s to keep us under time
    double collectStartTime = isolationTimer.time(sec);
    while (outSensorCount < 1 && (isolationTimer.time(sec) - collectStartTime) < 1.5) {
        bool outSensorDetecting = OutSensor.isNearObject();
        
        // Stop Score motor when OutSensor sees an object
        if (outSensorDetecting) {
            Score.stop();
        }
        
        if (outSensorDetecting && !outSensorWasDetecting) {
            outSensorCount++;
            intakeSensorCount++;  // Track collected balls for scoring phase
            if (outSensorCount == 1) {
                Controller.Screen.clearScreen();
                Controller.Screen.setCursor(1, 1);
                Controller.Screen.print("First ball detected!");
                wait(50, msec);  // Brief wait for ball to pass through
                BallStopper.set(false);  // RETRACTED after first ball passes
                Controller.Screen.clearScreen();
                Controller.Screen.setCursor(1, 1);
                Controller.Screen.print("BallStopper retracted!");
            }
        }
        outSensorWasDetecting = outSensorDetecting;
        wait(10, msec);
    }
    
    // If timeout reached without detecting ball, retract BallStopper anyway
    if (outSensorCount < 1) {
        BallStopper.set(false);
    }
    
    // Motors keep running, collect all balls
    // wait(150, msec);
    
    // ===== STEP 7: Back up to goal and retract MatchLoader =====
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("STEP 7: Backing up...");
    
    isolationDriveBackward(35);
    wait(200, msec);
    MatchLoader.set(false);  // RETRACTED
    wait(100, msec);
    
    // ===== PHASE 2: SCORE BALLS =====
    // ===== STEP 8: BallStopper EXTENDED again, prepare for scoring (40% speed) =====
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("STEP 8: Scoring prep...");
    
    BallStopper.set(true);   // EXTENDED
    Intake.setVelocity(40, percent);
    Ramp.setVelocity(40, percent);
    Score.setVelocity(40, percent);
    Intake.spin(fwd);
    Ramp.spin(fwd);
    Score.spin(fwd);
    wait(150, msec);
    
    // ===== STEP 9: Score balls - loop while outSensorCount < intakeSensorCount and both < MAX =====
    outSensorCount = 0;  // Reset counter for scoring phase
    int ballNumber = 0;  // Track which ball we're processing
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("STEP 9: Scoring (%d)", intakeSensorCount);
    
    // Scoring loop exits if timers run long to preserve the 15s window
    double scoreStartTime = isolationTimer.time(sec);
    while (outSensorCount < intakeSensorCount && intakeSensorCount < MAX_OBJECT_THRESHOLD && (isolationTimer.time(sec) - scoreStartTime) < 5.0) {
        bool outSensorDetecting = OutSensor.isNearObject();
        
        // Detect ball passing with edge detection (rising edge)
        if (outSensorDetecting && !outSensorWasDetecting) {
            outSensorCount++;
            ballNumber++;
            
            // Get the color of the current ball
            double ballHue = OutSensor.hue();
            bool isBlueBall = (ballHue >= 180.0 && ballHue <= 240.0);  // Blue hue range
            bool isRedBall = (ballHue >= 0.0 && ballHue <= 30.0) || (ballHue >= 330.0 && ballHue <= 360.0);  // Red hue range
            
            // Determine if we should throw this ball
            bool shouldThrow = false;
            if (ballColor == OBJECT::BallBlue) {
                // BLUE alliance: throw RED balls, keep BLUE balls
                shouldThrow = isRedBall;
            } else {
                // RED alliance: throw BLUE balls, keep RED balls
                shouldThrow = isBlueBall;
            }
            
            if (shouldThrow) {
                // THROW IT
                Controller.Screen.clearScreen();
                Controller.Screen.setCursor(1, 1);
                Controller.Screen.print("Throwing ball %d", ballNumber);
                
                // Move forward 10 inches
                isolationDriveForward(10);
                wait(100, msec);
                
                // Face 0 degrees to throw the ball
                isolationTurnToFace(0.0);
                wait(100, msec);
                
                // Motors run for 500ms to eject the ball
                isolationRunMotorsForward();
                wait(200, msec);
                
                // Stop motors
                isolationStopMotors();
                wait(50, msec);
                
                // Turn back to goal angle
                isolationTurnToFace(goalFacing);
                wait(100, msec);
                
                // Back up 10 inches into goal
                isolationDriveBackward(10);
                wait(50, msec);
                
                // Restart motors for next ball
                Intake.spin(fwd);
                Ramp.spin(fwd);
                Score.spin(fwd);
            } else {
                // Keep this ball - pass through (motors keep running)
                Controller.Screen.clearScreen();
                Controller.Screen.setCursor(1, 1);
                Controller.Screen.print("Keeping ball %d", ballNumber);
                // Motors keep running
            }
        }
        
        outSensorWasDetecting = outSensorDetecting;
        wait(10, msec);
    }
    
    // ===== STEP 10: All balls processed - move forward and stop =====
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("STEP 10: Finalizing...");
    
    isolationDriveForward(10);
    wait(100, msec);
    isolationStopMotors();
    wait(100, msec);

    // Restore drivetrain speed to default (full speed)
    chassis.set_drive_constants(10, 1.5, 0, 10, 0);
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("ISOLATION DONE");
    wait(100, msec);
}

// ============= INTERACTION MACRO =============

/**
 * INTERACTION MACRO (PLACEHOLDER)
 * Autonomous routine for interaction period.
 * Will be implemented with specific interaction logic.
 * 
 * @param ballColor Color of ball alliance (OBJECT::BallBlue or OBJECT::BallRed)
 */
void Interaction(OBJECT ballColor) {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("INTERACTION START");
    wait(1000, msec);

    // Parking override helper: stop everything, then park
    auto park_now = [&]() {
        Intake.stop();
        Ramp.stop();
        Score.stop();
        LeftDrive.stop(brakeType::hold);
        RightDrive.stop(brakeType::hold);
        wait(200, msec);
        Parking(ballColor);
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("INTERACTION DONE");
    };

    // Interaction runtime guard using Brain system timer (ms)
    double interactionStartMs = Brain.Timer.system();
    
    // ===== CHECK SENSORS FOR WRONG COLOR BALLS & EJECT =====
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Checking sensors...");
    
    Intake.setVelocity(60, percent);
    Ramp.setVelocity(60, percent);
    Score.setVelocity(60, percent);
    
    bool wrongBallDetected = false;
    
    // Check IntakeSensor
    if (IntakeSensor.isNearObject()) {
        double intakeHue = IntakeSensor.hue();
        bool intakeSeesBlue = (intakeHue >= 180.0 && intakeHue <= 240.0);
        bool intakeSeesRed = (intakeHue >= 0.0 && intakeHue <= 30.0) || (intakeHue >= 330.0 && intakeHue <= 360.0);
        
        bool wrongColor = false;
        if (ballColor == OBJECT::BallBlue) {
            wrongColor = intakeSeesRed;  // Blue alliance: red is wrong
        } else {
            wrongColor = intakeSeesBlue;  // Red alliance: blue is wrong
        }
        
        if (wrongColor) {
            wrongBallDetected = true;
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Wrong ball in Intake!");
        }
    }
    
    // Check MiddleSensor
    if (MiddleSensor.isNearObject()) {
        double middleHue = MiddleSensor.hue();
        bool middleSeesBlue = (middleHue >= 180.0 && middleHue <= 240.0);
        bool middleSeesRed = (middleHue >= 0.0 && middleHue <= 30.0) || (middleHue >= 330.0 && middleHue <= 360.0);
        
        bool wrongColor = false;
        if (ballColor == OBJECT::BallBlue) {
            wrongColor = middleSeesRed;  // Blue alliance: red is wrong
        } else {
            wrongColor = middleSeesBlue;  // Red alliance: blue is wrong
        }
        
        if (wrongColor) {
            wrongBallDetected = true;
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Wrong ball in Middle!");
        }
    }
    
    // Check OutSensor
    if (OutSensor.isNearObject()) {
        double outHue = OutSensor.hue();
        bool outSeesBlue = (outHue >= 180.0 && outHue <= 240.0);
        bool outSeesRed = (outHue >= 0.0 && outHue <= 30.0) || (outHue >= 330.0 && outHue <= 360.0);
        
        bool wrongColor = false;
        if (ballColor == OBJECT::BallBlue) {
            wrongColor = outSeesRed;  // Blue alliance: red is wrong
        } else {
            wrongColor = outSeesBlue;  // Red alliance: blue is wrong
        }
        
        if (wrongColor) {
            wrongBallDetected = true;
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Wrong ball in Out!");
        }
    }
    
    // If wrong ball detected, run motors in reverse to eject it
    if (wrongBallDetected) {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Ejecting wrong ball...");
        
        Intake.spin(vex::directionType::rev);
        Ramp.spin(vex::directionType::rev);
        Score.spin(vex::directionType::rev);
        
        // Run reverse until no balls are detected (max 3 seconds)
        timer ejectTimer;
        ejectTimer.clear();
        while (ejectTimer.time(sec) < 3.0) {
            bool anyBallDetected = IntakeSensor.isNearObject() || MiddleSensor.isNearObject() || OutSensor.isNearObject();
            if (!anyBallDetected) {
                Controller.Screen.clearScreen();
                Controller.Screen.setCursor(1, 1);
                Controller.Screen.print("Wrong ball ejected!");
                wait(300, msec);
                break;
            }
            wait(50, msec);
        }
        
        Intake.stop();
        Ramp.stop();
        Score.stop();
    }
    
    // Set drivetrain to 50% speed for Interaction and tighten exit conditions
    chassis.set_drive_constants(5, 1.5, 0, 10, 0);
    chassis.set_drive_exit_conditions(1.5, 200, 2000);
    chassis.set_turn_exit_conditions(1, 200, 1500);
    
    // ===== STEP 1: Determine initial interaction zone =====
    // BLUE: (120,-60) or (120,60), RED: (-120,-60) or (-120,60)
    double curr_x = GPS.xPosition();
    double curr_y = GPS.yPosition();
    
    // Determine which zone to go to (closer of two options)
    double zone_x, zone_y, facing_angle;
    if (ballColor == OBJECT::BallBlue) {
        double dist1 = sqrt(pow(120.0 - curr_x, 2) + pow(-60.0 - curr_y, 2));
        double dist2 = sqrt(pow(120.0 - curr_x, 2) + pow(60.0 - curr_y, 2));
        zone_x = 120.0;
        zone_y = (dist1 <= dist2) ? -60.0 : 60.0;
        facing_angle = 270.0;  // Face left (270°)
    } else {
        double dist1 = sqrt(pow(-120.0 - curr_x, 2) + pow(-60.0 - curr_y, 2));
        double dist2 = sqrt(pow(-120.0 - curr_x, 2) + pow(60.0 - curr_y, 2));
        zone_x = -120.0;
        zone_y = (dist1 <= dist2) ? -60.0 : 60.0;
        facing_angle = 90.0;  // Face right (90°)
    }
    
    // Navigate to the interaction zone directly (no A*)
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Going to zone...");
    isolationMoveToCoordinate(zone_x, zone_y);
    wait(500, msec);
    
    // ===== LOOP UNTIL SCORE OR 80s TIMEOUT (no Timer1/2/3 dependency) =====
    bool scored = false;
    while (!scored) {
        double elapsed = (Brain.Timer.system() - interactionStartMs) / 1000.0;
        double remaining = 80.0 - elapsed;

        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("LEFT: %.1f sec", remaining);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Finding ball...");
        wait(300, msec);

        if (remaining <= 0.0) {
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Going to Park");
            wait(300, msec);
            park_now();
            return;
        }

        // ===== STEP 2: Face the right direction and scan for balls =====
        isolationTurnToFace(facing_angle);
        wait(500, msec);

        // Initialize motors for ball pickup
        Intake.setVelocity(60, percent);
        Ramp.setVelocity(60, percent);
        Score.setVelocity(60, percent);

        // ===== STEP 3: Ball collection - collect 1 ball =====
        int blocksCollected = 0;
        setRampBallCount(0);  // Reset ramp counter for this cycle

        // Extend BallStopper to hold the ball during collection
        BallStopper.set(false);
        wait(200, msec);

        while (blocksCollected < 1) {
        double loopElapsed = (Brain.Timer.system() - interactionStartMs) / 1000.0;
        if (loopElapsed >= 80.0) { park_now(); return; }
        // Get closest ball of the correct color
        DETECTION_OBJECT closestBall = findTarget(ballColor);
        
        if (closestBall.depth > 0) {
            // Ball found - navigate to it using A*
            double distanceCm = closestBall.depth * 100.0;  // Convert meters to cm
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Ball found! Dist: %.1f", distanceCm);
            
            // Get ball location
            double curr_x = GPS.xPosition();
            double curr_y = GPS.yPosition();
            double ball_x = closestBall.mapLocation.x * 100.0;  // Convert to cm
            double ball_y = closestBall.mapLocation.y * 100.0;  // Convert to cm
            
            // Build field map and reject balls that sit inside obstacle geometry
            FieldMap ballFieldMap;
            ballFieldMap.populateStandardField();
            if (ballFieldMap.isPointInObstacle(ball_x, ball_y)) {
                // Skip this ball and search for another
                Controller.Screen.clearScreen();
                Controller.Screen.setCursor(1, 1);
                Controller.Screen.print("Ball in obstacle, skipping");
                ignoreCurrentBall();
                continue;
            }
            
            // Start motors before navigating
            Intake.spin(fwd);
            Ramp.spin(fwd);
            Score.spin(fwd);
            
            // Use A* to navigate to the ball
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("Planning path to ball...");
            
            std::vector<astar::Point> ballPath = astar::findPath(
                ballFieldMap,
                curr_x, curr_y,
                ball_x, ball_y,
                20.0,   // resolution_cm (denser grid to hug obstacles)
                24.0,   // robot_radius_cm (inflate to reduce scraping)
                15.0    // extra_margin_cm (larger safety buffer)
            );
            
            if (!ballPath.empty()) {
                bool capturedDuringPath = false;
                bool needReplan = false;
                for (size_t i = 0; i < ballPath.size(); i++) {
                    double wp_x = ballPath[i].first;
                    double wp_y = ballPath[i].second;
                    double robot_x = GPS.xPosition();
                    double robot_y = GPS.yPosition();
                    double bearing = calculateBearing(robot_x, robot_y, wp_x, wp_y);
                    double dx = wp_x - robot_x;
                    double dy = wp_y - robot_y;
                    double dist_cm = sqrt(dx*dx + dy*dy);
                    if (dist_cm < 5.0) continue;

                    // If segment to next waypoint crosses an obstacle, abort and replan
                    if (ballFieldMap.lineIntersectsObstacles(robot_x, robot_y, wp_x, wp_y)) {
                        needReplan = true;
                        break;
                    }

                    chassis.turn_to_angle(bearing);
                    wait(30, msec);
                    chassis.drive_distance(dist_cm / 2.54);
                    wait(30, msec);

                    // Check if ball was captured while following path
                    if (detectOpticalRisingEdge()) {
                        emergencyStop();
                        incrementRampBallCount(1);
                        capturedDuringPath = true;
                        break;
                    }
                    
                    // Check MiddleSensor for wrong color ball (color sensing)
                    if (MiddleSensor.isNearObject()) {
                        double sensorHue = MiddleSensor.hue();
                        bool sensorSeesBlue = (sensorHue >= 180.0 && sensorHue <= 240.0);
                        bool sensorSeesRed = (sensorHue >= 0.0 && sensorHue <= 30.0) || (sensorHue >= 330.0 && sensorHue <= 360.0);
                        
                        bool wrongColor = false;
                        if (ballColor == OBJECT::BallBlue) {
                            wrongColor = sensorSeesRed;  // Blue alliance sees red ball
                        } else {
                            wrongColor = sensorSeesBlue;  // Red alliance sees blue ball
                        }
                        
                        if (wrongColor) {
                            // Wrong color detected - eject it
                            Controller.Screen.clearScreen();
                            Controller.Screen.setCursor(1, 1);
                            Controller.Screen.print("Wrong color! Ejecting...");
                            
                            // Slow down the robot
                            chassis.set_drive_constants(2.5, 1.5, 0, 10, 0);  // Half speed
                            
                            // Retract ScoreMiddle to let ball fall out
                            ScoreMiddle.set(false);
                            wait(500, msec);
                            
                            // Extend ScoreMiddle again
                            ScoreMiddle.set(true);
                            wait(200, msec);
                            
                            // Resume normal speed
                            chassis.set_drive_constants(5, 1.5, 0, 10, 0);  // Back to normal
                            
                            Controller.Screen.clearScreen();
                            Controller.Screen.setCursor(1, 1);
                            Controller.Screen.print("Ejected, continuing...");
                        }
                    }
                }

                if (needReplan) {
                    // Skip to next loop iteration to compute a new path
                    continue;
                }

                if (!capturedDuringPath) {
                    // Final approach using vision alignment and optical confirmation
                    bool alignedNow = AlignToObject(ballColor, 25.0, 3000.0);
                    if (alignedNow) {
                        detectNextBall();
                        runIntake(vex::directionType::rev);
                        const float STOP_DISTANCE_CM = 12.0f;
                        bool captured = ApproachObject(ballColor, STOP_DISTANCE_CM);
                        stopIntake();
                        if (captured) {
                            incrementRampBallCount(1);
                            blocksCollected++;
                            Controller.Screen.clearScreen();
                            Controller.Screen.setCursor(1, 1);
                            Controller.Screen.print("Ball collected!");
                            ignoreCurrentBall();
                            // settle
                            wait(200, msec);
                        } else {
                            // Not captured, continue search: ignore this ball and look for another
                            Controller.Screen.clearScreen();
                            Controller.Screen.setCursor(1, 1);
                            Controller.Screen.print("Capture failed, searching...");
                            ignoreCurrentBall();
                        }
                    } else {
                        Controller.Screen.clearScreen();
                        Controller.Screen.setCursor(1, 1);
                        Controller.Screen.print("Align failed, searching...");
                        // Ignore this ball and search again
                        ignoreCurrentBall();
                    }
                } else {
                    // Captured during path
                    blocksCollected++;
                    Controller.Screen.clearScreen();
                    Controller.Screen.setCursor(1, 1);
                    Controller.Screen.print("Ball collected!");
                    // settle
                    wait(200, msec);
                }
            } else {
                Controller.Screen.clearScreen();
                Controller.Screen.setCursor(1, 1);
                Controller.Screen.print("No path to ball - searching next");
                // Ignore this current detection so next loop considers a different ball
                ignoreCurrentBall();
                // Do a brief search sweep to find alternative candidates
                FindObject(ballColor, 90.0, 25.0);
            }
            wait(500, msec);
        } else {
            // No ball found - search around
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("No ball found");
            
            // Turn slightly to search
            chassis.turn_to_angle(facing_angle + 30);
            wait(200, msec);
            chassis.turn_to_angle(facing_angle);
            wait(200, msec);
        }
    }  // END OF COLLECTION LOOP - blocksCollected should be 1 now

    // If no ball was collected (timeout/override), try again until 80s limit
    if (blocksCollected < 1) {
        continue;  // Loop back to search again or exit on timeout
    }
    
    // Retract BallStopper after collection
    BallStopper.set(false);
    wait(200, msec);
    
    // ===== STEP 4: When 1 block collected, go to closest scoring goal =====
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Ball collected! Going to goal...");
    wait(500, msec);
    
    // Stop motors
    Intake.stop();
    Ramp.stop();
    Score.stop();
    wait(200, msec);
    
    // Determine which of 4 goals is closest
    double goal_A_x = 120.0;   // Goal A: +,+
    double goal_A_y = 120.0;
    double goal_B_x = 120.0;   // Goal B: +,-
    double goal_B_y = -120.0;
    double goal_C_x = -120.0;  // Goal C: -,+
    double goal_C_y = 120.0;
    double goal_D_x = -120.0;  // Goal D: -,-
    double goal_D_y = -120.0;
    
    curr_x = GPS.xPosition();
    curr_y = GPS.yPosition();
    
    double distA = sqrt(pow(goal_A_x - curr_x, 2) + pow(goal_A_y - curr_y, 2));
    double distB = sqrt(pow(goal_B_x - curr_x, 2) + pow(goal_B_y - curr_y, 2));
    double distC = sqrt(pow(goal_C_x - curr_x, 2) + pow(goal_C_y - curr_y, 2));
    double distD = sqrt(pow(goal_D_x - curr_x, 2) + pow(goal_D_y - curr_y, 2));
    
    double minDist = distA;
    int closestGoal = 1;  // A
    
    if (distB < minDist) { minDist = distB; closestGoal = 2; }  // B
    if (distC < minDist) { minDist = distC; closestGoal = 3; }  // C
    if (distD < minDist) { minDist = distD; closestGoal = 4; }  // D
    
    double target_goal_x, target_goal_y;
    bool faceRight = false;  // For determining 90° or 270°
    
    if (closestGoal == 1) {  // A: +,+
        target_goal_x = goal_A_x;
        target_goal_y = goal_A_y;
        faceRight = true;  // 90°
    } else if (closestGoal == 2) {  // B: +,-
        target_goal_x = goal_B_x;
        target_goal_y = goal_B_y;
        faceRight = true;  // 90°
    } else if (closestGoal == 3) {  // C: -,+
        target_goal_x = goal_C_x;
        target_goal_y = goal_C_y;
        faceRight = false;  // 270°
    } else {  // D: -,-
        target_goal_x = goal_D_x;
        target_goal_y = goal_D_y;
        faceRight = false;  // 270°
    }
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Going to Goal %c...", (char)('A' + closestGoal - 1));
    wait(500, msec);
    
    isolationMoveToCoordinate(target_goal_x, target_goal_y);
    wait(500, msec);
    
    // ===== STEP 5: Score the ball =====
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Scoring...");
    
    // Turn to face goal
    double goalFacing = faceRight ? 90.0 : 270.0;
    isolationTurnToFace(goalFacing);
    wait(300, msec);
    
    // Retract BallStopper before backing up
    BallStopper.set(false);
    wait(200, msec);
    
    // Run motors forward to score
    Intake.setVelocity(60, percent);
    Ramp.setVelocity(60, percent);
    Score.setVelocity(60, percent);
    Intake.spin(fwd);
    Ramp.spin(fwd);
    Score.spin(fwd);
    wait(500, msec);
    
    // Drive backward to score
    isolationDriveBackward(20);
    wait(200, msec);
    
    // Extend BallStopper after backing up and keep motors running
    BallStopper.set(true);
    wait(200, msec);
    
    // Motors continue running for a bit longer
    wait(300, msec);
    
    // Stop motors
    Intake.stop();
    Ramp.stop();
    Score.stop();
    wait(200, msec);
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Ball scored!");
    wait(300, msec);

    // Watch OutSensor for a scored ball (quick exit)
    timer scoreWatch;
    scoreWatch.clear();
    bool outSeen = false;
    while (scoreWatch.time(msec) < 2000) {  // up to 2s
        if (OutSensor.isNearObject()) {
            outSeen = true;
            break;
        }
        wait(50, msec);
    }

    // If scored (OutSensor saw a ball), park immediately
    if (outSeen) {
        park_now();
        return;
    }

    // Loop continues if not scored and under 80s
    }

    // ===== HANDOFF TO PARKING (scored or hit 80s limit) =====
    park_now();
    return;
}

// ============= PARKING MACRO =============

/**
 * PARKING MACRO
 * Navigates to the closer of two possible parking coordinates (based on alliance color) and parks.
 * 
 * @param ballColor Alliance color (OBJECT::BallBlue or OBJECT::BallRed)
 */
void Parking(OBJECT ballColor) {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("PARKING START");
    Controller.rumble(".-.");
    wait(500, msec);
    
    // ===== SINGLE PARKING COORDINATE =====
    double target_x = -90.0;
    double target_y = 0.0;
    double finalHeading = 90.0;
    double final_park_x = 160.0;
    double final_park_y = 0.0;
    
    // ===== STEP 1: Get current position =====
    double curr_x = GPS.xPosition();
    double curr_y = GPS.yPosition();
    double curr_h = GPS.heading();
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("GPS: %.1f,%.1f", curr_x, curr_y);
    wait(500, msec);
    
    if ((curr_x == 0.0 && curr_y == 0.0) || isnan(curr_x) || isnan(curr_y) || isnan(curr_h)) {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Bad GPS - Using defaults");
        Controller.rumble("--");
        wait(1000, msec);
        // Use default position instead of aborting
        curr_x = ballColor == OBJECT::BallRed ? -115.0 : 115.0;
        curr_y = 0.0;
        curr_h = 0.0;
    }

    // ===== STEP 2: Report target =====
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Parking to %.1f,%.1f", target_x, target_y);
    wait(300, msec);

    // Restore drivetrain speed to default (50%)
    chassis.set_drive_constants(5, 1.5, 0, 10, 0);

    isolationMoveToCoordinate(-90,0); // move close to parking point
    wait(500, msec);
    //isolationDriveBackward(30);  // move into parking point
    isolationTurnToFace(90);  // face correct direction
    chassis.set_drive_constants(12, 1.5, 0, 10, 0);
    wait(500, msec);
    isolationDriveBackward(70);  // move into parking point


    // // ===== STEP 4: Use A* to navigate to chosen coordinate =====
    // Controller.Screen.clearScreen();
    // Controller.Screen.setCursor(1, 1);
    // Controller.Screen.print("Planning path...");
    
    // FieldMap fieldMap;
    // fieldMap.populateStandardField();
    
    // std::vector<astar::Point> path = astar::findPath(
    //     fieldMap,
    //     curr_x, curr_y,
    //     target_x, target_y,
    //     60.0,   // resolution_cm (increased from 30 to reduce waypoints)
    //     15.0,   // robot_radius_cm (increased from 12 to simplify path)
    //     5.0     // extra_margin_cm (increased from 3 for more clearance)
    // );
    
    // if (path.empty()) {
    //     Controller.Screen.clearScreen();
    //     Controller.Screen.setCursor(1, 1);
    //     Controller.Screen.print("No path found!");
    //     wait(2000, msec);
    //     return;
    // }
    
    // Controller.Screen.clearScreen();
    // Controller.Screen.setCursor(1, 1);
    // Controller.Screen.print("Following A* path...");
    // wait(500, msec);
    
    // // Follow path step-by-step: turn to each waypoint, drive, advance
    // for (size_t i = 0; i < path.size(); i++) {
    //     double wp_x = path[i].first;
    //     double wp_y = path[i].second;
        
    //     // Get current position
    //     double robot_x = GPS.xPosition();
    //     double robot_y = GPS.yPosition();
    //     double robot_h = GPS.heading();
        
    //     if ((robot_x == 0.0 && robot_y == 0.0) || isnan(robot_x) || isnan(robot_y)) {
    //         cout << "Bad GPS at WP" << i << ", breaking\n";
    //         break;
    //     }
        
    //     // Calculate bearing to waypoint
    //     double bearing = calculateBearing(robot_x, robot_y, wp_x, wp_y);
    //     double dx = wp_x - robot_x;
    //     double dy = wp_y - robot_y;
    //     double dist_cm = sqrt(dx*dx + dy*dy);
        
    //     // Skip if already at waypoint
    //     if (dist_cm < 5.0) {
    //         cout << "Waypoint " << (int)(i+1) << " (" << wp_x << "," << wp_y << ") reached\n";
    //         continue;
    //     }
        
    //     // Display waypoint info
    //     Controller.Screen.clearScreen();
    //     Controller.Screen.setCursor(1, 1);
    //     Controller.Screen.print("WP %d/%d", (int)(i+1), (int)path.size());
    //     Controller.Screen.setCursor(2, 1);
    //     Controller.Screen.print("Dist: %.1f cm", dist_cm);
        
    //     // Turn and drive to waypoint
    //     double dist_in = dist_cm / 2.54;
    //     chassis.turn_to_angle(bearing);
    //     wait(30, msec);
    //     chassis.drive_distance(dist_in);
    //     wait(50, msec);

    //     // Corrective pass if we missed by more than 15cm
    //     double robot_x2 = GPS.xPosition();
    //     double robot_y2 = GPS.yPosition();
    //     double err_cm = sqrt(pow(wp_x - robot_x2, 2) + pow(wp_y - robot_y2, 2));
    //     int retry = 0;
    //     while (err_cm > 15.0 && retry < 2) {
    //         double bearing2 = calculateBearing(robot_x2, robot_y2, wp_x, wp_y);
    //         double dist_cm2 = sqrt(pow(wp_x - robot_x2, 2) + pow(wp_y - robot_y2, 2));
    //         double dist_in2 = dist_cm2 / 2.54;
    //         chassis.turn_to_angle(bearing2);
    //         wait(30, msec);
    //         chassis.drive_distance(dist_in2);
    //         wait(50, msec);
    //         robot_x2 = GPS.xPosition();
    //         robot_y2 = GPS.yPosition();
    //         err_cm = sqrt(pow(wp_x - robot_x2, 2) + pow(wp_y - robot_y2, 2));
    //         retry++;
    //     }
        
    //     // Print waypoint reached (or best-effort if within tolerance)
    //     cout << "Waypoint " << (int)(i+1) << " (" << wp_x << "," << wp_y << ") reached";
    //     cout << " | err: " << err_cm << "cm\n";
    //     Controller.Screen.clearScreen();
    //     Controller.Screen.setCursor(1, 1);
    //     Controller.Screen.print("Waypoint %d/%d reached", (int)(i+1), (int)path.size());
    // }
    
    // wait(200, msec);
    
    // // ===== STEP 5: Turn to final heading (270°) =====
    // isolationTurnToFace(finalHeading);
    // wait(200, msec);
    
    // // ===== STEP 6: Drive backward to (160,0) =====
    // double curr_park_x = GPS.xPosition();
    // double curr_park_y = GPS.yPosition();
    // double backup_dist_cm = sqrt(pow(final_park_x - curr_park_x, 2) + pow(final_park_y - curr_park_y, 2));
    // double backup_dist_in = backup_dist_cm / 2.54;
    
    // Controller.Screen.clearScreen();
    // Controller.Screen.setCursor(1, 1);
    // Controller.Screen.print("Parking %.1f in...", backup_dist_in);
    
    // chassis.drive_distance(-backup_dist_in);  // Negative = reverse
    // wait(200, msec);
    
    // // ===== STEP 7: Stop =====
    // emergencyStop();
    // LeftDrive.stop(hold);
    // RightDrive.stop(hold);
    
    // Controller.Screen.clearScreen();
    // Controller.Screen.setCursor(1, 1);
    // Controller.Screen.print("PARKING COMPLETE");
    // wait(1000, msec);
}