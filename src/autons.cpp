#include "vex.h"

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, .67, 0, 2, 0); 
  chassis.set_heading_constants(1, .4, 0, 1, 0);
  chassis.set_turn_constants(10, .17, 0, .75, 0);
  chassis.set_swing_constants(12, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  // Tighter settle conditions to prevent overshooting
  chassis.set_drive_exit_conditions(1, 100, 3000);
  chassis.set_turn_exit_conditions(2, 100, 3000);
  chassis.set_swing_exit_conditions(1, 100, 3000);
}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like drive_to_point(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants(){
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = .5;
  chassis.drive_min_voltage = 0;
}

/**
 * The expected behavior is to return to the start position.
 */

void drive_test(){
  chassis.drive_distance(6);
  chassis.drive_distance(12);
  chassis.drive_distance(18);
  chassis.drive_distance(-36);
}

/**
 * The expected behavior is to return to the start angle, after making a complete turn.
 */

void turn_test(){
  chassis.turn_to_angle(5);
  chassis.turn_to_angle(30);
  chassis.turn_to_angle(90);
  chassis.turn_to_angle(225);
  chassis.turn_to_angle(0);
}

/**
 * Should swing in a fun S shape.
 */

void swing_test(){
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(0);
}

/**
 * A little of this, a little of that; it should end roughly where it started.
 */

void full_test(){
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

/**
 * Doesn't drive the robot, but just prints coordinates to the Brain screen 
 * so you can check if they are accurate to life. Push the robot around and
 * see if the coordinates increase like you'd expect.
 */

void odom_test(){
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5,20, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(5,40, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(5,60, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    task::sleep(20);
  }
}

/**
 * Should end in the same place it began, but the second movement
 * will be curved while the first is straight.
 */

void tank_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.turn_to_point(24, 24);
  chassis.drive_to_point(24,24);
  chassis.drive_to_point(0,0);
  chassis.turn_to_angle(0);
}

/**
 * Drives in a square while making a full turn in the process. Should
 * end where it started.
 */

void holonomic_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_pose(0, 18, 90);
  chassis.holonomic_drive_to_pose(18, 0, 180);
  chassis.holonomic_drive_to_pose(0, 18, 270);
  chassis.holonomic_drive_to_pose(0, 0, 0);
}

void left_side_auton(){
  chassis.drive_distance(13);
  Intake1.spin(fwd, 100, pct);
  Intake2.stop();
  Intake2.setStopping(hold);
  chassis.turn_to_angle(-30);
  chassis.drive_max_voltage =4;
  chassis.drive_distance(15);
  chassis.drive_distance(-3);
  Intake1.stop();
  Intake2.stop();
  chassis.turn_max_voltage=6;
  chassis.drive_max_voltage = 6;
  chassis.turn_to_angle(45+180);//180 because it's a negative angle
  chassis.drive_distance(-15);
  Intake1.spin(fwd, 100, pct);
  Intake2.spin(fwd, 100, pct);
  wait(0.2,sec);
  Intake1.stop();
  Intake2.stop();
  chassis.drive_distance(50);
  chassis.turn_to_angle(180);
  chassis.drive_distance(-21);  
  chassis.turn_to_angle(180);
  Intake1.spin(fwd, 100, pct);
  Intake2.spin(fwd, 100, pct);
  wait(2, sec);
  Intake1.stop();
  Intake2.stop();
  Intake2.setStopping(hold);
  
  // Activate pneumatics
  Solenoid.set(true);
  Intake1.spin(fwd, 100, pct);
  chassis.drive_max_voltage = 5.5;
  //drive to the matchload at 50% speed
  chassis.turn_to_angle(180);
  chassis.drive_distance(28);
  chassis.turn_to_angle(180);
  chassis.drive_max_voltage = 8;
  wait(0.4,sec);
  Intake1.stop();
  //drive back
  
   chassis.turn_to_angle(180);
  chassis.drive_distance(-29);
  // Deactivate pneumatics
  Solenoid.set(false);
  Intake1.spin(fwd, 100, pct);
  Intake2.spin(fwd, 100, pct);
  wait(2,sec);
  chassis.drive_distance(10);
  
  chassis.drive_max_voltage = 100;
  chassis.drive_distance(-10);
  
  chassis.drive_distance(5);
}

void right_side_9ball_auton(){
  chassis.drive_distance(13);
  Intake1.spin(fwd, 100, pct);
  Intake2.stop();
  Intake2.setStopping(hold);
  chassis.turn_to_angle(30);
  chassis.drive_distance(14);
  chassis.drive_max_voltage = 8;
  chassis.turn_to_angle(40);
  chassis.drive_distance(20);
  chassis.drive_distance(-20);
  chassis.turn_to_angle(135);
  Intake1.stop();
  Intake2.stop();
  chassis.drive_max_voltage = 6;
  chassis.drive_distance(35);
  chassis.turn_to_angle(180);
  chassis.drive_distance(-20);
  // intake forward for 2 seconds
  Intake1.spin(fwd, 100, pct);
  Intake2.spin(fwd, 100, pct);
  wait(1.5, sec);
  Intake1.stop();
  Intake2.stop();
  Intake2.setStopping(hold);
  chassis.turn_to_angle(180);
  // Activate pneumatics
  Solenoid.set(true);
  Intake1.spin(fwd, 100, pct);
  chassis.drive_max_voltage = 6;
  //drive to the matchload at 50% speed
  chassis.drive_distance(29.5);
  chassis.turn_to_angle(180);
  chassis.drive_max_voltage = 10;
  wait(0.4,sec);
  Intake1.stop();
  //drive back
  chassis.drive_distance(-29.5);
  // Deactivate pneumatics
  Solenoid.set(false);
  Intake1.spin(fwd, 100, pct);
  Intake2.spin(fwd, 100, pct);
  wait(1.5,sec);
  chassis.drive_distance(10);
  
  chassis.drive_distance(-10);
  
  chassis.drive_distance(5);

}



void right_side_auton(){
  chassis.drive_distance(13);
  Intake1.spin(fwd, 100, pct);
  Intake2.stop();
  Intake2.setStopping(hold);
  chassis.turn_to_angle(30);
  chassis.drive_max_voltage = 4;
  chassis.drive_distance(14);
  wait(0.5, sec);
  chassis.turn_to_angle(135);
  Intake1.stop();
  Intake2.stop();
  chassis.turn_max_voltage=6;
  chassis.drive_max_voltage = 6;
  chassis.drive_distance(35);
  chassis.turn_to_angle(180);
  chassis.drive_distance(-20);
  // intake forward for 2 seconds
  Intake1.spin(fwd, 100, pct);
  Intake2.spin(fwd, 100, pct);
  wait(1.5, sec);
  Intake1.stop();
  Intake2.stop();
  Intake2.setStopping(hold);
  chassis.turn_to_angle(180);
  // Activate pneumatics
  Solenoid.set(true);
  Intake1.spin(fwd, 100, pct);
  chassis.drive_max_voltage = 6;
  //drive to the matchload at 50% speed
  chassis.drive_distance(29.5);
  chassis.turn_to_angle(180);
  chassis.drive_max_voltage = 8;
  wait(0.4,sec);
  Intake1.stop();
  //drive back
  chassis.drive_distance(-29.5);
  // Deactivate pneumatics
  Solenoid.set(false);
  Intake1.spin(fwd, 100, pct);
  Intake2.spin(fwd, 100, pct);
  wait(1.5,sec);
  chassis.drive_distance(10);
  
  chassis.drive_distance(-10);
  
  chassis.drive_distance(5);

}





void skills_auton(){
  default_constants();
  
  //chassis.drive_max_voltage = 12;
  //chassis.drive_distance(30);
  

  /*
  chassis.drive_distance(48);
  chassis.turn_to_angle(90);
  chassis.drive_distance(48);
  chassis.turn_to_angle(180);
  chassis.drive_distance(48);
  chassis.turn_to_angle(270);
  chassis.drive_distance(48);
  chassis.turn_to_angle(0);

  */
 
  Solenoid2.set(false);
  chassis.drive_distance(32);

  //Matchloader Pt.1
  chassis.turn_to_angle(90);
  Solenoid.set(true);
  Intake1.spin(fwd, 100, pct);

  //chassis.drive_max_voltage = 4;
  //Matchloader Pt.2
  chassis.turn_to_angle(90);
  chassis.drive_distance(10.7);
  chassis.drive_distance(-5);
  chassis.drive_distance(5.6, 12, 90, 0, 1, 100, 1000);
  chassis.turn_to_angle(90);
  //chassis.drive_max_voltage= 6;
  wait(1.63,sec);

  Intake1.stop();

  //Scoring Long Goal
  chassis.drive_distance(-30.2);
  Intake1.spin(reverse, 50, pct);
  wait(0.25, sec);
  Intake1.spin(fwd, 100, pct);
  Intake2.spin(fwd, 100, pct);
  wait(4.5,sec);
  //chassis.drive_distance(7);
  //chassis.drive_distance(-7);


  //Travel between Long Goal and Wall
  Solenoid.set(false);
  Intake1.stop();
  Intake2.stop();
  chassis.drive_distance(10);
  chassis.turn_to_angle(0);
  chassis.drive_distance(14.35);
  chassis.turn_to_angle(-90);
  chassis.drive_distance(85);
  chassis.turn_to_angle(180);
  chassis.drive_distance(12);
  chassis.turn_to_angle(-90);

  //Matchload #2
  Intake1.spin(fwd, 100, pct);
  Solenoid.set(true);
  chassis.drive_distance(14.5);
  chassis.drive_distance(-5);
  chassis.drive_distance(6);
  wait(1.5,sec);

  //Scoring 
  chassis.drive_distance(-30.2);
  Intake1.spin(reverse, 50, pct);
  wait(0.25, sec);
  Intake1.spin(fwd, 100, pct);
  Intake2.spin(fwd, 100, pct);
  wait(2,sec);

  //Travel to next matchloader
  chassis.drive_distance(12);
  chassis.turn_to_angle(180);
  //chassis drive_distance(96);

  //Other Side
  /*
  chassis.drive_max_voltage= 6 ;
  chassis.drive_distance(95);
  chassis.turn_to_angle(90);
  Solenoid.set(true);

  Intake1.spin(fwd,100,pct);

  chassis.drive_distance(18, 0, 4, 0);
  chassis.drive_distance(-4);
  chassis.drive_distance(4);
  wait(1.5,sec);
  Intake1.stop();

  chassis.drive_distance(-28);
  Intake1.spin(reverse, 30, pct);
  wait(0.15, sec);
  Solenoid.set(false);
  Intake1.spin(fwd, 100, pct);
  Intake2.spin(fwd, 100, pct);
  wait(5,sec);
  Intake1.stop();
  Intake2.stop();
  
  chassis.drive_distance(10);
  chassis.turn_to_angle(0);
  chassis.drive_distance(50);
  chassis.turn_to_angle(-90);
  chassis.drive_max_voltage= 12;
  chassis.drive_distance(-42);
  */

  


  /*
  chassis.drive_distance(48);
  chassis.turn_to_angle(-90);
  chassis.drive_max_voltage= 12;
  chassis.drive_distance(-42);
*/

 


  /*
  Solenoid.set(false);
   Intake1.spin(fwd, 100, pct); //either forward of reverse idk
  wait(0.6,sec);
  Intake1.stop();


  chassis.drive_distance(-6);
  chassis.drive_distance(6);
  chassis.drive_distance(-6);
  chassis.turn_to_angle(90);
  chassis.drive_distance(12.6);
  chassis.turn_to_angle(-90);
  chassis.drive_distance(78);
  chassis.turn_to_angle(-90);
  chassis.drive_distance(13);
  chassis.turn_to_angle(90);


  Solenoid.set(true);


   Intake1.spin(fwd, 100, pct);
  chassis.set_drive_constants(4, 1, 0.5, 6, 0);
  chassis.drive_distance(14);
  chassis.set_drive_constants(8, 1, 0.5, 6, 0);
  wait(1.5,sec);
  Intake1.stop();


  Solenoid.set(false);


  chassis.drive_distance(-21);
   Intake1.spin(fwd, 100, pct);
  wait(1.5,sec);
  Intake1.stop();
  chassis.drive_distance(-6);
  chassis.turn_to_angle(-90);
  chassis.drive_distance(95);
  chassis.turn_to_angle(90);


  Solenoid.set(true);
 
   Intake1.spin(fwd, 100, pct);
  chassis.set_drive_constants(4, 1, 0.5, 6, 0);
  chassis.drive_distance(10);
  chassis.set_drive_constants(8, 1, 0.5, 6, 0);


  wait(0.4,sec);
  Intake1.stop();


  Solenoid.set(false);


  chassis.drive_distance(-20.1);
   Intake1.spin(fwd, 100, pct);
  wait(1.5,sec);
  Intake1.stop();


  chassis.drive_distance(5);
  chassis.turn_to_angle(90);
  chassis.drive_distance(16);
  chassis.turn_to_angle(-90);
  chassis.drive_distance(78);
  chassis.turn_to_angle(-90);
  chassis.drive_distance(13);
  chassis.turn_to_angle(90);


  Solenoid.set(true);
   Intake1.spin(fwd, 100, pct);


  chassis.set_drive_constants(4, 1, 0.5, 6, 0);
  chassis.drive_distance(9);
  chassis.set_drive_constants(8, 1, 0.5, 6, 0);


  wait(0.4,sec);
  Intake1.stop();
  Solenoid.set(false);


  chassis.drive_distance(-21);
   Intake1.spin(fwd, 100, pct);
  wait(1.5,sec);
  Intake1.stop();
  */
  }

