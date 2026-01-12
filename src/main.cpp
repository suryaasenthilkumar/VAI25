/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Mon Aug 31 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "ai_functions.h"

using namespace vex;

brain Brain;
controller Controller;
// Robot configuration code.
// Sensors
// inertial Inertial = inertial(PORT5);
rotation ForwardTracker = rotation(PORT3);
gps LGPS = gps(PORT2, 10.8, -2.5, distanceUnits::cm, 270);
gps RGPS = gps(PORT4, -10.8, -2.5, distanceUnits::cm, 90);
// DualGPS GPS = DualGPS(LGPS, RGPS, Inertial, vex::distanceUnits::cm);
optical IntakeSensor = optical(PORT14);
optical OutSensor = optical(PORT6);
optical MiddleSensor = optical(PORT7);
// Left Drive
motor LeftDriveA = motor(PORT20, ratio6_1, true);
motor LeftDriveB = motor(PORT19, ratio6_1, true);
motor LeftDriveC = motor(PORT21, ratio6_1, true);
motor_group LeftDrive = motor_group(LeftDriveA, LeftDriveB, LeftDriveC);
// Right Drive
motor RightDriveA = motor(PORT11, ratio6_1, false);
motor RightDriveB = motor(PORT10, ratio6_1, false);
motor RightDriveC = motor(PORT9, ratio6_1, false);
motor_group RightDrive = motor_group(RightDriveA, RightDriveB, RightDriveC);
//Drivetrain a.k.a Chassis
// smartdrive Drivetrain = smartdrive(LeftDrive, RightDrive, Inertial, 3.25, 0.75, 360);
// Intake
motor Ramp = motor(PORT17, ratio18_1, true);
motor Intake = motor(PORT15, ratio18_1, true);
// Score
motor Score = motor(PORT13, ratio18_1, false);

// digital_out ScoreMiddle = digital_out(Brain.ThreeWirePort.A);
// digital_out MatchLoader = digital_out(Brain.ThreeWirePort.B);
// digital_out BallStopper = digital_out(Brain.ThreeWirePort.C);

// ===== STUCK DETECTION AND RECOVERY SYSTEM =====
// Global state for stuck detection
bool stuckDetectionActive = false;  // Enable during interaction period
bool robotIsStuck = false;          // Flag when stuck detected
double lastGPSX = 0.0;
double lastGPSY = 0.0;
timer stuckTimer;

/**
 * Recovery function: Back up 15 inches when stuck
 */
void performStuckRecovery() {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("STUCK! Recovering...");
    Controller.rumble(".");
    
    // Stop all motors
    LeftDrive.stop();
    RightDrive.stop();
    wait(200, msec);
    
    // Back up 15 inches
    chassis.drive_distance(-15);
    wait(500, msec);
    
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Recovery complete");
    wait(500, msec);
    
    // Reset stuck flag
    robotIsStuck = false;
    stuckTimer.clear();
}

/**
 * Stuck detection monitoring thread
 * Runs continuously and checks if robot hasn't moved for 5 seconds
 */
int stuckDetectionThread() {
    while (true) {
        if (stuckDetectionActive) {
            // Get current GPS position
            double currentX = GPS.xPosition();
            double currentY = GPS.yPosition();
            
            // Check if position changed
            double deltaX = fabs(currentX - lastGPSX);
            double deltaY = fabs(currentY - lastGPSY);
            bool positionChanged = (deltaX > 2.0 || deltaY > 2.0);  // 2cm threshold
            
            // Only position change resets timer (ignore wheel spin while stuck)
            if (positionChanged) {
                stuckTimer.clear();
                lastGPSX = currentX;
                lastGPSY = currentY;
            }
            // If not moving for 5 seconds, trigger stuck recovery
            else if (stuckTimer.time(sec) >= 5.0 && !robotIsStuck) {
                robotIsStuck = true;
                performStuckRecovery();
            }
        }
        
        wait(100, msec);  // Check every 100ms
    }
    return 0;
}


// A global instance of competition
competition Competition;

// create instance of jetson class to receive location and other
// data from the Jetson nano
//
ai::jetson  jetson_comms;

Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
//ZERO_TRACKER_NO_ODOM,
TANK_ONE_FORWARD_ROTATION,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(LeftDriveA, LeftDriveB, LeftDriveC),

//Right Motors:
motor_group(RightDriveA, RightDriveB, RightDriveC),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT5,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.75,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
3,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
-2,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
1,

//Sideways tracker diameter (reverse to make the direction switch):
-2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
5.5

);
DualGPS GPS = DualGPS(LGPS, RGPS, chassis.Gyro, vex::distanceUnits::cm);

/*----------------------------------------------------------------------------*/
// Create a robot_link on PORT1 using the unique name robot_32456_1
// The unique name should probably incorporate the team number
// and be at least 12 characters so as to generate a good hash
//
// The Demo is symetrical, we send the same data and display the same status on both
// manager and worker robots
// Comment out the following definition to build for the worker robot
#define  MANAGER_ROBOT    1

#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link       link( PORT15, "robot_32456_1", linkType::manager );
#else
#pragma message("building for the worker")
ai::robot_link       link( PORT10, "robot_32456_1", linkType::worker );
#endif

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Auto_Isolation Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous isolation  */
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void auto_Isolation(void) {
  // Calibrate GPS Sensor (no timer gating)
  GPS.calibrate();

  // Enable stuck detection for Isolation run
  stuckDetectionActive = true;
  stuckTimer.clear();
  lastGPSX = GPS.xPosition();
  lastGPSY = GPS.yPosition();

  // Run Isolation once; rely on field period for 15s cutoff
  Isolation(OBJECT::BallRed);

  // Disable stuck detection after Isolation
  stuckDetectionActive = false;
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                        Auto_Interaction Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous interaction*/
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void auto_Interaction(void) {
  // Enable stuck detection for interaction period
  stuckDetectionActive = true;
  stuckTimer.clear();
  lastGPSX = GPS.xPosition();
  lastGPSY = GPS.yPosition();

  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("AUTO_INTERACTION START");
  Controller.rumble("-");
  wait(1000, msec);

  // Run Parking directly
  Parking(OBJECT::BallRed);

  // Disable stuck detection after Interaction
  stuckDetectionActive = false;
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          AutonomousMain Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*---------------------------------------------------------------------------*/

bool firstAutoFlag = true;

void autonomousMain(void) {
  // ..........................................................................
  // The first time we enter this function we will launch our Isolation routine
  // When the field goes disabled after the isolation period this task will die
  // When the field goes enabled for the second time this task will start again
  // and we will enter the interaction period. 
  // ..........................................................................
  // chassis.turn_to_angle(180);
  if(firstAutoFlag) {
    firstAutoFlag = false;  // Set flag BEFORE running, not after
    auto_Isolation();
  }
  else {
    auto_Interaction();
  }
}

void usercontrol(void){
  // CALIBRATE GPS FIRST
  static bool calibrated = false;
  if(!calibrated) {
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Calibrating GPS...");
    GPS.calibrate();
    waitUntil(!GPS.isCalibrating());
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("Ready!");
    calibrated = true;

    // Set starting position and drive forward
    // initialize();
  }
  // initialize();
  while(true) {
    // Manual drive control
    chassis.control_arcade(); 
    // ButtonA: Test optical sensor
    if(Controller.ButtonA.pressing()){
      waitUntil(!Controller.ButtonA.pressing());
      Controller.Screen.clearScreen();
      Controller.Screen.setCursor(1, 1);
      if(IntakeSensor.isNearObject()) {
        Controller.Screen.print("BLOCKED!");
      } else {
        Controller.Screen.print("CLEAR!");
      }
      wait(2000, msec);
    }
    // ButtonB: find, pick up, navigate to goal and score (all-in-one)
    if(Controller.ButtonB.pressing()){
      waitUntil(!Controller.ButtonB.pressing());
      ScoreLGB1();
    }
    // ButtonX: Collect 6 RED balls (full capacity)
    if(Controller.ButtonX.pressing()){
      waitUntil(!Controller.ButtonX.pressing());
      collectMultipleBalls(OBJECT::BallRed, 6);
    }
    wait(20, msec);
    // ButtonR1: findGoalCoordinates()
    if(Controller.ButtonR1.pressing()){
      waitUntil(!Controller.ButtonR1.pressing());
      findGoalCoordinates();
      // wait(10000, msec);
    }
    // ButtonUp: check captured ball color (no driving)
    if(Controller.ButtonUp.pressing()){
      waitUntil(!Controller.ButtonUp.pressing());
      // ensure drivetrain is not moving
      chassis.drive_with_voltage(0,0);
      // run color check; use alliance color = Blue for this test
      checkCapturedBallColor(OBJECT::BallBlue);
      wait(500, msec);
    }
    // ButtonL1: testWaypoints()
    if(Controller.ButtonL1.pressing()){
      waitUntil(!Controller.ButtonL1.pressing());
      testWaypoints();
      wait(2000, msec);
    }
    // ButtonDown: Test A* path planning
    if(Controller.ButtonDown.pressing()){
      waitUntil(!Controller.ButtonDown.pressing());
      testPathPlanning();
      wait(500, msec);
    }
    // ButtonLeft: Test Isolation macro (Blue alliance)
    if(Controller.ButtonLeft.pressing()){
      waitUntil(!Controller.ButtonLeft.pressing());
      Isolation(OBJECT::BallBlue);
      wait(500, msec);
    }
    // ButtonRight: Test Parking macro (Blue alliance)
    if(Controller.ButtonRight.pressing()){
      waitUntil(!Controller.ButtonRight.pressing());
      Parking(OBJECT::BallBlue);
      wait(500, msec);
    }
    // ButtonR2: Test Interaction macro (Blue alliance)
    if(Controller.ButtonR2.pressing()){
      waitUntil(!Controller.ButtonR2.pressing());
      Interaction(OBJECT::BallBlue);
      wait(500, msec);
    }
    // ButtonL2: Test pneumatic pistons
    if(Controller.ButtonL2.pressing()){
      waitUntil(!Controller.ButtonL2.pressing());
      pistontest();
      wait(500, msec);
    }
    // ButtonY: Display GPS Info
    if(Controller.ButtonY.pressing()){
      waitUntil(!Controller.ButtonX.pressing());
      displayGPSPosition();
      wait(10000, msec);  // Show for 10 seconds
    }
  }
}

int main() {

  // local storage for latest data from the Jetson Nano
  static AI_RECORD local_map;

  // Run at about 15Hz
  int32_t loop_time = 33;

  // CALIBRATE INERTIAL FIRST
  // Inertial.calibrate();
  chassis.Gyro.calibrate();
  waitUntil(!chassis.Gyro.isCalibrating());
  
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  //chassis.set_turn_constants(6, 0.2, 0, 1.5, 5);
  chassis.set_turn_constants(12, .4, .03, 3, 15);

  chassis.set_swing_constants(6, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
  
  // ============ WAIT FOR JETSON TO BE READY ============
  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("Wait for Jetson...");

  AI_RECORD test_map;
  int attempts = 0;
  while(attempts < 60) {  // 60 attempts = 6 seconds
      jetson_comms.get_data(&test_map);
      if(jetson_comms.get_packets() > 10) {  // Got at least 10 packets
          break;
      }
      wait(100, msec);
      attempts++;
  }

  Controller.Screen.clearScreen();
  Controller.Screen.setCursor(1, 1);
  Controller.Screen.print("Jetson Ready!");
  Controller.Screen.setCursor(2, 1);
  Controller.Screen.print("Packets: %d", jetson_comms.get_packets());
  wait(1000, msec);
  // =====================================================

  // start the status update display
  thread t1(dashboardTask);
  
  // start the stuck detection thread
  thread stuckThread(stuckDetectionThread);

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomousMain);
  Competition.drivercontrol(usercontrol);

  // print through the controller to the terminal (vexos 1.0.12 is needed)
  // As USB is tied up with Jetson communications we cannot use
  // printf for debug.  If the controller is connected
  // then this can be used as a direct connection to USB on the controller
  // when using VEXcode.
  //
  //FILE *fp = fopen("/dev/serial2","wb");
  this_thread::sleep_for(loop_time);

  // ===== Initialize pneumatics to safe state =====
  MatchLoader.set(false);   // RETRACTED
  ScoreMiddle.set(true);    // EXTENDED
  BallStopper.set(false);   // RETRACTED
  wait(200, msec);

  Ramp.setVelocity(30, percent);
  Score.setVelocity(30, percent);

  while(1) {
      // get last map data
      jetson_comms.get_data( &local_map );

      // set our location to be sent to partner robot
      link.set_remote_location( local_map.pos.x, local_map.pos.y, local_map.pos.az, local_map.pos.status );

      // fprintf(fp, "%.2f %.2f %.2f\n", local_map.pos.x, local_map.pos.y, local_map.pos.az)

      // request new data    
      // NOTE: This request should only happen in a single task.    
      jetson_comms.request_map();

      // Allow other tasks to run
      this_thread::sleep_for(loop_time);
  }
}