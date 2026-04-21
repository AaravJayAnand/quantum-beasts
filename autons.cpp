#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 95;
const int SWING_SPEED = 110;

void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 2_deg, 250_ms, 5_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 2_deg, 250_ms, 5_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 0.5_in, 250_ms, 2_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

// Left side 15 sec auto
void left_awp() {
    matchloader.set(true); // Deploy matchloader
    chassis.pid_odom_set({{-27_in, 6_in, 180_deg}, fwd, 110}); // Position in front of matchloader
    chassis.pid_wait_quick();
    intake.move(-127); // Start intake
    chassis.pid_odom_set({{-27_in, -6_in, 180_deg}, fwd, 127}); // Ram into matchloader
    chassis.pid_wait_quick(); // This wait is enough to matchload blocks, no further delay is required
    chassis.pid_odom_set({{-27.5_in, 36_in, 180_deg}, rev, 127}); // Reverse into long goal
    matchloader.set(false);
     // Start scoring while moving to long goal to minimize wait time
    pros::delay(675);
    outtake.move(127);
    chassis.pid_wait_quick();
    pros::delay(500); // Wait 500ms to allow for time to score
    /* Keep the outtake running to dispose of any extra blocks that may have been collected
       This prevents interference from blocks blocking (:D) the center goal piston */
    chassis.pid_odom_set({{-28_in, 8_in}, fwd, 127}); // Prepare to move to center goal
    chassis.pid_wait_quick();
    intake.move(-127);
    chassis.pid_turn_set(45_deg, 110);
    chassis.pid_wait();
    chassis.pid_odom_set({{5_in, 40_in, 45_deg}, fwd, 127}); // Position to intake center 3 blocks
    pros::delay(500);
    outtake.move(0); // Stop outtake
    matchloader.set(true); // Extend matchloader to enclose blocks for collection
    chassis.pid_wait_quick();
    chassis.pid_wait();
    chassis.pid_odom_set({{0_in, 36_in}, rev, 127}); // Move back so following turn doesn't hit center goal
    matchloader.set(false); // Contract matchloader
    chassis.pid_wait_quick();
    chassis.pid_turn_set(225_deg, 127); // Turn to 225 deg to prepare to score in center goal
    chassis.pid_wait_quick();
    intake.move(30); // Slightly move intake in opposite direction to allow for a cleaner score
    outtake.move(-30); // Slightly move outtake in opposite direction to allow for a cleaner score
    chassis.pid_odom_set({{13.5_in, 50_in, 225_deg}, rev, 127}); // Move to center goal
    // Start scoring earlier again to counter for feeding time
    pros::delay(250);
    centerGoal.set(false); // Lower center goal piston
    intake.move(-127); // Move intake up to feed outtake
    outtake.move(-127); // Spin reverse (negative) due to counterroller and middle goal
    chassis.pid_wait_quick();
    pros::delay(325); // Wait for blocks to be scored
    intake.move(0); // Stop intake
    outtake.move(0); // Stop outtake
    centerGoal.set(true); // Extend center goal piston
    // Descore long goal
    chassis.pid_odom_set({{-12_in, 35_in, 0_deg}, fwd, 127});
    chassis.pid_wait();
    descorer.set(true);
    chassis.pid_odom_set({{-15_in, 55_in, 0_deg}, fwd, 127});
    chassis.pid_wait_quick();
}

void right_awp() {
    intake.move(-127); // Start intake
    chassis.pid_odom_set({{6_in, 41_in, 15_deg}, fwd, 110}); // Intake center 3 blocks
    pros::delay(600);
    matchloader.set(true); // Bring down matchloader after 600 ms to enclose blocks and matchload later
    chassis.pid_wait_quick();
    chassis.pid_odom_set({{27_in, 6_in, 180_deg}, fwd, 127}); // Move to matchloader
    chassis.pid_wait_quick();
    chassis.pid_odom_set({{27_in, -4_in, 180_deg}, fwd, 127}); // Ram into matchloader
    chassis.pid_wait_quick(); // This wait is enough to matchload blocks
    chassis.pid_odom_set({{28_in, 36_in, 180_deg}, rev, 127}); // Reverse into long goal
    chassis.pid_wait_quick();
    outtake.move(127); // Start outtake
    pros::delay(3000); // Wait 1250 ms for blocks to scor e into long goal
    outtake.move(0); // Stop outtake
    chassis.pid_odom_set({{24_in, 18_in, 180_deg}, fwd, 127}); // Prepare to descore
    chassis.pid_wait_quick();
    chassis.pid_odom_set({{24_in, 30_in, 180_deg}, fwd, 127}); // Descore
    chassis.pid_wait_quick();
}