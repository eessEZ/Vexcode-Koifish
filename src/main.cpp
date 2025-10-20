#include "vex.h"

using namespace vex;
competition Competition;

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors. In VEXcode Pro V5, you can do this using the graphical          */
/*  configurer port icon at the top right. In the VSCode extension, you'll   */
/*  need to go to robot-config.cpp and robot-config.h and create the         */
/*  motors yourself by following the style shown. All motors must be         */
/*  properly reversed, meaning the drive should drive forward when all       */
/*  motors spin forward.                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
/*---------------------------------------------------------------------------*/

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
ZERO_TRACKER_NO_ODOM,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(LeftFront, LeftMiddle, LeftBack),

//Right Motors:
motor_group(RightFront, RightMiddle, RightBack),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT12,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.75,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
357,

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

int current_auton_selection =1;
bool auto_started = false;
// Driver state mirrored from Python config
bool is_tank_drive = true; // Set tank drive as default
bool is_reverse_arcade = false;
int drive_speed = 100;
bool pneumatics_extended = false; // placeholder, no direct solenoid mapped
bool pneumatics2_extended = false; // second pneumatics state

bool prev_toggle_combo = false;
bool prev_speed_toggle = false;
bool prev_pneumatics_press = false;
bool prev_pneumatics2_press = false;
bool prev_reverse_arcade_combo = false;

/**
 * Function before autonomous. It prints the current auton number on the screen
 * and tapping the screen cycles the selected auton by 1. Add anything else you
 * may need, like resetting pneumatic components. You can rename these autons to
 * be more descriptive, if you like.
 */

void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // IMMEDIATELY retract pneumatics when program starts
  Solenoid.set(false);        // Force Port H pneumatics retracted
  Solenoid2.set(true);        // Force Port A pneumatics ON by default
  wait(50, msec);             // Brief delay to ensure command processes
  
  // Calibrate gyro sensor
  InertialSensor.calibrate();
  while (InertialSensor.isCalibrating()) {
    wait(10, msec);
  }
  wait(500, msec);
  
  default_constants();
  
  // Initialize pneumatics to retracted position (safety default)
  Solenoid.set(false);        // L2 pneumatics - retracted
  Solenoid2.set(true);        // A pneumatics - ON by default
  pneumatics_extended = false;
  pneumatics2_extended = true;
  
  // Ensure pneumatics are fully retracted with a small delay
  wait(100, msec);
  Solenoid.set(false);        // Double-check Port H pneumatics retracted
  Solenoid2.set(true);        // Double-check Port A pneumatics ON

  while(!auto_started){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "JAR Template v1.2.0");
    Brain.Screen.printAt(5, 40, "Battery Percentage:");
    Brain.Screen.printAt(5, 60, "%d", Brain.Battery.capacity());
    Brain.Screen.printAt(5, 80, "Chassis Heading Reading:");
    Brain.Screen.printAt(5, 100, "%f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5, 120, "Selected Auton:");
    switch(current_auton_selection){
      case 0:
        Brain.Screen.printAt(5, 140, "Auton 1");
        break;
      case 1:
        Brain.Screen.printAt(5, 140, "Auton 2");
        break;
      case 2:
        Brain.Screen.printAt(5, 140, "Auton 3");
        break;
      case 3:
        Brain.Screen.printAt(5, 140, "Auton 4");
        break;
      case 4:
        Brain.Screen.printAt(5, 140, "Auton 5");
        break;
      case 5:
        Brain.Screen.printAt(5, 140, "Auton 6");
        break;
      case 6:
        Brain.Screen.printAt(5, 140, "Auton 7");
        break;
      case 7:
        Brain.Screen.printAt(5, 140, "Auton 8");
        break;
    }
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
      current_auton_selection ++;
    } else if (current_auton_selection == 8){
      current_auton_selection = 0;
    }
    wait(10, msec);
  }
}

/**
 * Auton function, which runs the selected auton. Case 0 is the default,
 * and will run in the brain screen goes untouched during preauton. Replace
 * drive_test(), for example, with your own auton function you created in
 * autons.cpp and declared in autons.h.
 */

void autonomous(void) {
  auto_started = true;
  
  // Set drive motors to HOLD mode for precise autonomous movements
  LeftFront.setStopping(hold);
  LeftMiddle.setStopping(hold);
  LeftBack.setStopping(hold);
  RightFront.setStopping(hold);
  RightMiddle.setStopping(hold);
  RightBack.setStopping(hold);
  
  switch(current_auton_selection){ 
    case 0: 
    right_side_auton();
    
    break;
    case 1:
    left_side_auton();
       
    break;



    case 2:
      {
        chassis.drive_distance(24);

      }
      break;
    case 3:
      skills_auton();
      break;
    case 4:
      chassis.drive_distance(13);
      Intake1.spin(fwd, 100, pct);
      Intake2.stop();
      Intake2.setStopping(hold);
      chassis.turn_to_angle(25);
      // Second drive_distance(13) at 40% speed
      chassis.set_drive_constants(4, 1.0, 0.5, 6, 0);
      chassis.drive_distance(16);
      chassis.set_drive_constants(8, 1.0, 0.5, 6, 0);
      wait(0.5, sec);
      // Restore normal speed for turns
      Intake1.stop();
      Intake2.stop();
      chassis.turn_to_angle(0);

      chassis.drive_distance(-22);
      chassis.turn_to_angle(90);
      chassis.drive_distance(27);
      chassis.turn_to_angle(180);
      chassis.drive_distance(-18);
      chassis.turn_to_angle(180);
      // intake forward for 2 seconds
      Intake1.spin(fwd, 100, pct);
      Intake2.spin(fwd, 100, pct);
      wait(1.5, sec);
      Intake1.stop();
      Intake2.stop();
      Intake2.setStopping(hold);
      
      // Activate pneumatics
      Solenoid.set(true);
      pneumatics_extended = true;
      Intake1.spin(fwd, 100, pct);
      //drive to the matchload at 50% speed
      chassis.set_drive_constants(4, 1, 0.5, 6, 0);
      chassis.drive_distance(30);
      chassis.turn_to_angle(180);
      chassis.set_drive_constants(8, 1, 0.5, 6, 0);
      // Restore normal speed
      wait(0.4,sec);
      Intake1.stop();
      //drive back
      chassis.drive_distance(-30);
      // Deactivate pneumatics
      Solenoid.set(false);
      pneumatics_extended = false;
      Intake1.spin(fwd, 100, pct);
      Intake2.spin(fwd, 100, pct);
      wait(1.5,sec);
      chassis.drive_distance(10);
      
      chassis.drive_distance(-10);
      
      chassis.drive_distance(5);

      break;
    case 5:
      odom_test();
      break;
    case 6:
      tank_odom_test();
      break;
    case 7:
      holonomic_odom_test();
      break;
 }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // Set drive motors back to COAST mode for smooth user control
  LeftFront.setStopping(coast);
  LeftMiddle.setStopping(coast);
  LeftBack.setStopping(coast);
  RightFront.setStopping(coast);
  RightMiddle.setStopping(coast);
  RightBack.setStopping(coast);
  
  // Reset pneumatics when entering user control
  Solenoid.set(false);        // L2 pneumatics - retracted
  Solenoid2.set(true);        // A pneumatics - ON by default
  pneumatics_extended = false;
  pneumatics2_extended = true;
  
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    // === Intake Control ===
    if (Controller1.ButtonR1.pressing()){
      Intake1.spin(fwd, 100, pct);
      Intake2.spin(fwd, 100, pct);
      Intake2.setStopping(coast);
    } else if (Controller1.ButtonR2.pressing()){
      // intake1 forward only
      Intake1.spin(fwd, 100, pct);
      Intake2.stop();
      Intake2.setStopping(hold);
    } else if (Controller1.ButtonL1.pressing()){
      Intake1.spin(reverse, 100, pct);
      Intake2.spin(reverse, 100, pct);
      Intake2.setStopping(coast);
    } else {
      Intake1.stop();
      Intake2.stop();
      Intake2.setStopping(coast);
    }

    // === Drive Mode Toggle (Y + Right) ===
    bool toggle_combo = Controller1.ButtonY.pressing() && Controller1.ButtonRight.pressing();
    if (toggle_combo && !prev_toggle_combo){
      is_tank_drive = !is_tank_drive;
      is_reverse_arcade = false; // Reset reverse arcade when switching modes
      Brain.Screen.clearScreen();
      if (is_tank_drive) Brain.Screen.print("Drive Mode: Tank"); else Brain.Screen.print("Drive Mode: Arcade");
    }
    prev_toggle_combo = toggle_combo;

    // === Reverse Arcade Toggle (Up + X) ===
    bool reverse_arcade_combo = Controller1.ButtonUp.pressing() && Controller1.ButtonX.pressing();
    if (reverse_arcade_combo && !prev_reverse_arcade_combo){
      is_reverse_arcade = !is_reverse_arcade;
      is_tank_drive = false; // Switch to arcade mode when enabling reverse arcade
      Brain.Screen.clearScreen();
      if (is_reverse_arcade) Brain.Screen.print("Drive Mode: Reverse Arcade"); else Brain.Screen.print("Drive Mode: Arcade");
    }
    prev_reverse_arcade_combo = reverse_arcade_combo;

    // === Speed Toggle (B) ===
    bool curr_speed_toggle = Controller1.ButtonB.pressing();
    if (curr_speed_toggle && !prev_speed_toggle){
      drive_speed = (drive_speed == 100) ? 50 : 100;
      float scale = (float)drive_speed / 100.0f;
      chassis.set_driver_scale(scale);
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(5,20, "Speed: %d%%", drive_speed);
    }
    prev_speed_toggle = curr_speed_toggle;

    // === Drive Control ===
    int left_speed = 0;
    int right_speed = 0;
    
    // === Forward Movement (Down Button) ===
    if (Controller1.ButtonDown.pressing()) {
      // Move straight forward at 80% speed
      int forward_speed = (int)(80 * drive_speed / 100.0);
      left_speed = forward_speed;
      right_speed = forward_speed;
    } else {
      // Normal joystick control
      if (is_tank_drive){
        left_speed = (int)(deadband(Controller1.Axis3.position(), 5) * drive_speed / 100.0);
        right_speed = (int)(deadband(Controller1.Axis2.position(), 5) * drive_speed / 100.0);
      } else if (is_reverse_arcade) {
        // Reverse arcade: Axis4 (left/right) for left/right, Axis2 (left/right) for forward/back
        int fwd = (int)deadband(Controller1.Axis2.position(), 5);
        int turn = (int)deadband(Controller1.Axis4.position(), 5);
        left_speed = (int)((fwd + turn) * drive_speed / 100.0);
        right_speed = (int)((fwd - turn) * drive_speed / 100.0);
      } else {
        // Normal arcade: Axis3 (forward/back) for forward/back, Axis1 (left/right) for left/right
        int fwd = (int)deadband(Controller1.Axis3.position(), 5);
        int turn = (int)deadband(Controller1.Axis1.position(), 5);
        left_speed = (int)((fwd + turn) * drive_speed / 100.0);
        right_speed = (int)((fwd - turn) * drive_speed / 100.0);
      }
    }

  // Apply to motors via chassis API (convert percent to volts)
  float leftVolt = to_volt((float)left_speed);
  float rightVolt = to_volt((float)right_speed);
  chassis.drive_with_voltage(leftVolt, rightVolt);

    // === Pneumatics toggle (L2) ===
    bool pneu_press = Controller1.ButtonL2.pressing();
    if (pneu_press && !prev_pneumatics_press){
      pneumatics_extended = !pneumatics_extended;
      Solenoid.set(pneumatics_extended);
      Brain.Screen.clearScreen();
      if (pneumatics_extended) Brain.Screen.print("Pneumatics: Extended"); else Brain.Screen.print("Pneumatics: Retracted");
    }
    prev_pneumatics_press = pneu_press;

    // === Second Pneumatics toggle (A) ===
    bool pneu2_press = Controller1.ButtonA.pressing();
    if (pneu2_press && !prev_pneumatics2_press){
      pneumatics2_extended = !pneumatics2_extended;
      Solenoid2.set(pneumatics2_extended);
      Brain.Screen.clearScreen();
      if (pneumatics2_extended) Brain.Screen.print("Pneumatics 2: Extended"); else Brain.Screen.print("Pneumatics 2: Retracted");
    }
    prev_pneumatics2_press = pneu2_press;

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
