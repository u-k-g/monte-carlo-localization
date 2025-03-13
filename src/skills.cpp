#include "robot/skills.h"
#include "api.h"
#include "globals.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/lvgl.h"
#include "main.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "robot/monte.hpp"
#include <cmath>
#include <iostream>
#include <random>

void skills() {
  // Start the MCL background task
  // startMCL(chassis);

  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  intake.set_zero_position(intake.get_position());
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, 12, 800);
  chassis.turnToPoint(19, 12, 650, {.forwards = false});
  chassis.moveToPoint(16.5, 12, 850, {.forwards = false, .earlyExitRange = 2},
                      false);
  chassis.moveToPoint(19, 12, 550, {.forwards = false, .maxSpeed = 63}, false);

  // Latch the mogo - delays just in case
  clamp.extend();
  pros::delay(250);

  // Turn to the above ring - Start intake and hook, and move there
  chassis.turnToPoint(24, 36.75, 850, {}, false);
  intake.move_velocity(600);
  chassis.moveToPoint(24, 36.75, 1000, {}, false);

  chassis.turnToPoint(48, 36.75, 850, {}, false);
  chassis.moveToPoint(48, 36.75, 1000, {.maxSpeed = 40}, false);
  pros::delay(750);

  chassis.turnToPoint(45, 10, 850, {.maxSpeed = 40}, false);
  pros::delay(1000);
  chassis.moveToPoint(45, 10, 1000, {.maxSpeed = 40}, false);

  pros::delay(2000);
  chassis.moveToPoint(45, 6, 2000, {.maxSpeed = 40}, false);
  pros::delay(1000);
  chassis.turnToPoint(54.5, 6, 850, {.forwards = false, .maxSpeed = 40}, false);
  pros::delay(1000);
  chassis.moveToPoint(54.5, 1.5, 1000, {.forwards = false, .maxSpeed = 40},
                      true);
  intake.move_voltage(-12000);
  pros::delay(200); // Move at max voltage for 1 second
  intake.move_voltage(0);
  pros::delay(500);
  clamp.retract();
  intake.move_velocity(600);
  chassis.moveToPoint(-29, 13, 4000,
                      {.forwards = false, .maxSpeed = 50, .earlyExitRange = 2},
                      false);
  clamp.extend();
  intake.move_voltage(-12000);
  pros::delay(200); // Move at max voltage for 1 second
  intake.move_voltage(0);
  pros::delay(500);
  intake.move_velocity(600);
  chassis.turnToPoint(-29.8, 37, 1000, {}, false);
  pros::delay(1000);
  chassis.moveToPoint(-29.8, 37, 1000, {.maxSpeed = 50}, false);
  pros::delay(1000);
  chassis.turnToPoint(-53.8, 37, 1000, {}, false);
  pros::delay(1000);
  chassis.moveToPoint(-53.8, 37, 1000, {.maxSpeed = 50}, false);
  pros::delay(1000);
  chassis.moveToPoint(-53.8, 5, 1000, {.maxSpeed = 50}, false);
  pros::delay(1000);
  chassis.turnToPoint(-50.5, 6, 1000, {}, true);
  chassis.moveToPoint(-50.5, 6, 1000, {.maxSpeed = 50}, true);
  pros::delay(1000);
  chassis.turnToPoint(-39.5, 7, 1000, {}, true);
  chassis.moveToPoint(-39.5, 7, 1000, {.maxSpeed = 50}, false);
  chassis.turnToPoint(-67, 0, 1000, {.forwards = false}, false);
  chassis.moveToPoint(-67, 0, 1000, {.forwards = false, .maxSpeed = 50}, false);
  clamp.retract();
  intake.move_voltage(-12000);
  pros::delay(200); // Move at max voltage for 1 second
  intake.move_voltage(0);
  pros::delay(500);
  chassis.turnToPoint(-3, 62, 3000, {.forwards = true, .maxSpeed = 90}, false);
  pros::delay(1000);
  chassis.moveToPoint(-3, 62, 2000, {.forwards = true, .maxSpeed = 70}, false);
  // tune this delay
  intake.move_voltage(12000);
  pros::delay(600); // Move at max voltage for 1 second
  intake.move_voltage(0);
  pros::delay(500);
  chassis.moveToPoint(45, 104, 3000, {.forwards = true, .maxSpeed = 70}, false);
  pros::delay(1000);
  chassis.turnToPoint(-25, 108, 5000, {.forwards = false}, false);
  chassis.moveToPoint(-25, 108, 5000, {.forwards = false, .maxSpeed = 70},
                      false);
  clamp.extend();
  intake.move_velocity(600);
  chassis.moveToPoint(-64.5, 131.1, 2000, {.forwards = false}, false);
  chassis.moveToPoint(-55, 120, 500, {.forwards = true}, false);
  chassis.moveToPoint(-64.5, 131.1, 700, {.forwards = false}, false);
  clamp.retract();
  intake.move_voltage(-12000);
  pros::delay(200); // Move at max voltage for 1 second
  intake.move_voltage(0);
  chassis.moveToPoint(0, 120, 2000, {.forwards = true}, false);
  chassis.moveToPoint(-64.5, 131.1, 700, {.forwards = true}, false);

  stopMCL();
}

void skills1() {
  // Set pre-constants
  intake.set_zero_position(intake.get_position());
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(0, 0, 0);

  // Autonomous routine for the Skills challenge - 60 seconds MAX
  /* ############################################## */

  // De-hook into the alliance stake
  intake.move_absolute(600, 600);
  pros::delay(750);

  // Move to the center, then turn to and touch the right side Mogo.
  chassis.moveToPoint(0, 16, 800);
  chassis.turnToPoint(19, 13, 650, {.forwards = false});
  chassis.moveToPoint(16.5, 13, 850, {.forwards = false, .earlyExitRange = 2},
                      false);
  chassis.moveToPoint(19, 16, 550, {.forwards = false, .maxSpeed = 63}, false);

  // Latch the mogo - delays just in case
  clamp.extend();
  pros::delay(250);

  // Turn to the above ring - Start intake and hook, and move there
  chassis.turnToPoint(24, 38.75, 850, {}, false);
  intake.move_velocity(600);
  chassis.moveToPoint(24, 38.75, 1000);

  // Immediately move to the farthest ring. A couple of other things happen
  // while this is going on
  chassis.moveToPose(48, 90, 0, 2050,
                     {.horizontalDrift = 8, .lead = 0.4, .earlyExitRange = 2},
                     true);

  chassis.waitUntil(50);

  // Set the hook to a high-torque rpm to get the best loading possible, waits
  // for the MoveToPose to end.
  intake.move_velocity(400);
  chassis.waitUntilDone();
  // Turn to a point ~1.5 tiles away from the right neutral stake

  /*
  lady_brown.move_absolute(100, 200);
  chassis.turnToPoint(40, 62.5, 800, {.forwards = false});

  // Actually move there
  chassis.moveToPoint(40, 62.5, 1500, {.forwards = false}, true);
  // Immediately hit against the lady brown to ensure appropriate fit.
  //  Wait until 20 inches passed to get to the point behind the lady brown -
  //  stop the hook here.
  chassis.waitUntil(30);
  intake.brake();

  chassis.waitUntilDone();

  // Turn to the Neutral wall stake.
  chassis.turnToPoint(78, 62.5, 600);

  // Move there
  chassis.moveToPoint(72, 62.5, 900, {.forwards = true, .maxSpeed = 50}, true);

  // At 18 inches of movement, stop the intake in advance of the neutral wall
  // stake
  chassis.waitUntil(18);
  preroller.brake();

  chassis.waitUntilDone();

  // Allow the robot to settle, then move the lady brown to score on the neutral
  // stake.

  lady_brown.move_absolute(500, 200);

  // Move back to the 5th tile edge, and restart the intake 15 inches into the
  // movement.
  chassis.moveToPoint(47, 62.5, 750, {.forwards = false, .maxSpeed = 75});
  chassis.waitUntil(5);

  preroller.move_velocity(600);
  intake.move_velocity(200);

  // Point the robot towards the home side wall, and put the lady brown down.
  chassis.turnToPoint(47, 0, 950);
  lady_brown.move_absolute(0, 200);

  // Go close to the wall, but exit early so that the robot glides to the last
  // ring.
  chassis.moveToPoint(47, 0, 2500,
                      {.forwards = true, .maxSpeed = 45, .earlyExitRange = 2});

  // Start the hook at a higher speed rpm
  intake.move_velocity(200);

  // Start at 10 inches into the movement - schedule a check every 5 inches
  for
    // if the hook is stuck.
    chassis.waitUntil(10);

  chassis.waitUntilDone();
  // Wait for two seconds at the edge to ensure rings get put onto the mogo.
  pros::delay(1000);

  // Turn, then Move to the point where the last ring was nudged to collect it
  // for top ring
  chassis.turnToPoint(61, 17, 1000);
  chassis.moveToPoint(61, 17, 1000,
                      {.forwards = true, .maxSpeed = 50, .earlyExitRange = 4});

  // Turn away to position the mogo in the corner, waiting to let the hook put
  // all items onto the mogo.
  chassis.turnToHeading(
      -30, 1000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false);
  pros::delay(1000);
  // Let the last ring go when the hook gets stuck
  intake.move_velocity(-200);
  // Let the mogo go
  clamp.retract();

  // Move back a little to ensure the mogo goes into the corner.
  chassis.moveToPoint(
      64.5, 5, 1000, {.forwards = false, .maxSpeed = 50, .earlyExitRange = 1.5},
      false);

  // Stop the intake and hook.
  preroller.brake();
  intake.brake();

  // Move to the top edge of the first tile, preparing for alignment. Then
  move
      // back into the wall to use the distance sensor to reorient.
      chassis.moveToPose(48, 20, -90, 1000,
                         {.forwards = true, .horizontalDrift = 9}, true);
  chassis.moveToPoint(67, 20, 1350, {.forwards = false}, false);

  // Get the distance to the wall using the distance sensor, convert to inches
  // and add the distance to tracking center (5in). Then, set the new position
  // float new_x = (float)(72 - 8);
  float new_x = (float)(72 - 8);
  float new_y = ((float)(dSouth.get_distance()) / (float)10 / (float)2.54 +
                 (float)6.5 - (float)8.75);
  std::cout << "Distance: " << new_x << std::endl;
  std::cout << "Distance: " << (float)(72) / (float)10 / (float)2.54
            << std::endl;
  std::cout << "Distance: " << dSouth.get_distance() << std::endl;
  chassis.setPose(new_x, new_y, chassis.getPose().theta);
  chassis.moveToPoint(chassis.getPose().x - 5, chassis.getPose().y, 600,
                      {.earlyExitRange = 1});

  chassis.moveToPose(24, 15, -90, 1000, {.horizontalDrift = 8});

  chassis.turnToHeading(90, 1000, {});
  chassis.moveToPoint(-16.5, 15, 1000, {.forwards = false, .maxSpeed = 80},
                      false);
  chassis.moveToPoint(-20.5, 15, 700, {.forwards = false, .maxSpeed = 55},
                      false);

  // Latch the mogo - delays just in case
  clamp.extend();
  pros::delay(250);

  // Turn to the above ring - Start intake and hook, and move there
  chassis.turnToPoint(-24, 38.75, 850, {}, false);
  preroller.move_velocity(600);
  intake.move_velocity(200);
  chassis.moveToPoint(-24, 38.75, 1000);

  // Immediately move to the farthest ring. A couple of other things happen
  // while this is going on
  chassis.moveToPose(-48, 90, 0, 2050, {.horizontalDrift = 8, .lead = 0.4},
                     true);
  // Checks between 10-45 inches of movement for if the hook got stuck on the
  // mogo flower

  // Set the hook to a high-torque rpm to get the best loading possible, waits
  // for the MoveToPose to end.
  chassis.waitUntil(50);

  intake.move_velocity(110);

  chassis.waitUntilDone();
  // Turn to a point ~1.5 tiles away from the right neutral stake
  lady_brown.move_absolute(100, 200);
  chassis.turnToPoint(-40, 62.5, 800, {.forwards = false});

  // Actually move there
  chassis.moveToPoint(-40, 62.5, 1500, {.forwards = false}, true);

  // Immediately hit against the lady brown to ensure appropriate fit.
  chassis.waitUntil(30);
  intake.brake();

  // Wait until 20 inches passed to get to the point behind the lady brown -
  // stop the hook here.

  chassis.waitUntilDone();

  // Turn to the Neutral wall stake.
  chassis.turnToPoint(-78, 61.5, 660);

  // Move there
  chassis.moveToPoint(-72, 61.5, 920, {.forwards = true, .maxSpeed = 50}, true);

  // At 18 inches of movement, stop the intake in advance of the neutral wall
  // stake
  chassis.waitUntil(18);
  preroller.brake();

  chassis.waitUntilDone();

  // Allow the robot to settle, then move the lady brown to score on the
  neutral
      // stake.
      lady_brown.move_absolute(500, 200);

  // Move back to the 5th tile edge, and restart the intake 15 inches into the
  // movement.
  chassis.moveToPoint(-47, 63.5, 750, {.forwards = false, .maxSpeed = 75});
  chassis.waitUntil(5);
  preroller.move_velocity(600);
  intake.move_velocity(200);

  // Point the robot towards the home side wall, and put the lady brown down.
  chassis.turnToPoint(-47, 0, 950);
  lady_brown.move_absolute(0, 200);

  // Go close to the wall, but exit early so that the robot glides to the last
  // ring.
  chassis.moveToPoint(-47, -5, 2750,
                      {.forwards = true, .maxSpeed = 45, .earlyExitRange = 2});

  // Start the hook at a higher speed rpm
  intake.move_velocity(200);

  // Start at 10 inches into the movement - schedule a check every 5 inches
  for
    // if the hook is stuck.
    chassis.waitUntil(10);

  chassis.waitUntilDone();
  // Wait for two seconds at the edge to ensure rings get put onto the mogo.
  pros::delay(1000);

  // Turn, then Move to the point where the last ring was nudged to collect it
  // for top ring
  chassis.turnToPoint(-62, 17, 1000);
  chassis.moveToPoint(-62, 17, 2000,
                      {.forwards = true, .maxSpeed = 50, .earlyExitRange = 4});

  // Turn away to position the mogo in the corner, waiting to let the hook put
  // all items onto the mogo.
  chassis.turnToHeading(30, 1000, {}, false);
  pros::delay(1300);
  // Let the last ring go when the hook gets stuck
  intake.move_velocity(-200);
  pros::delay(200);
  // Let the mogo go
  clamp.retract();

  // Move back a little to ensure the mogo goes into the corner.
  chassis.moveToPoint(
      -64, 6, 1000, {.forwards = false, .maxSpeed = 50, .earlyExitRange = 1.5},
      false);

  // Stop the intake and hook.
  preroller.brake();
  intake.brake();

  // Move to the top edge of the first tile, preparing for alignment. Then
  move
      // back into the wall to use the distance sensor to reorient.
      chassis.moveToPose(-48, 20, -90, 2050, {.forwards = true, .maxSpeed = 80},
                         true);

  */
}