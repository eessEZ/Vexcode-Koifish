#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;

// Controller (primary)
controller Controller1 = controller(primary);

// Drive motors (matching koifish.v5python)
motor LeftFront = motor(PORT1, ratio6_1, true);
motor LeftMiddle = motor(PORT2, ratio6_1, true);
motor LeftBack = motor(PORT11, ratio6_1, true);

motor RightFront = motor(PORT9, ratio6_1, false);
motor RightMiddle = motor(PORT10, ratio6_1, false);
motor RightBack = motor(PORT20, ratio6_1, false);

// Intake motors
// Intake motors - bottom on PORT8, top on PORT18 for Intake1/Intake2 respectively
motor Intake1 = motor(PORT8, ratio6_1, false);   // bottom motor
motor Intake2 = motor(PORT19, ratio6_1, false);  // top motor

// Inertial sensor
inertial InertialSensor = inertial(PORT16);

// Pneumatics - solenoid on brain three-wire port H
digital_out Solenoid = digital_out(Brain.ThreeWirePort.H);
// Second pneumatics - solenoid on brain three-wire port A
digital_out Solenoid2 = digital_out(Brain.ThreeWirePort.A);

void vexcodeInit( void ) {
  // Set stopping modes to COAST for drive and intake, matching Python setup
  LeftFront.setStopping(coast);
  LeftMiddle.setStopping(coast);
  LeftBack.setStopping(coast);
  RightFront.setStopping(coast);
  RightMiddle.setStopping(coast);
  RightBack.setStopping(coast);
  Intake1.setStopping(coast);
  Intake2.setStopping(coast);
}