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

bool antiHookJam() {

  // Get current motor parameters
  double current = hooks.get_current_draw();               // Current draw in mA
  double velocity = std::abs(hooks.get_actual_velocity()); // Actual velocity
  double target_velocity =
      std::abs(hooks.get_target_velocity()); // Target velocity
  double torque = hooks.get_torque();        // Motor torque in Nm

  // Define threshold values for jam detection
  const double CURRENT_THRESHOLD = 2500;       // mA (adjust based on testing)
  const double VELOCITY_RATIO_THRESHOLD = 0.3; // Actual/target velocity ratio
  const double TORQUE_THRESHOLD = 1.8;         // Nm (adjust based on testing)
  const double MIN_TARGET_VELOCITY =
      50; // Minimum velocity to consider for detection

  // Detect jam conditions
  bool highCurrent = (current > CURRENT_THRESHOLD);
  bool lowVelocity = (velocity / target_velocity < VELOCITY_RATIO_THRESHOLD);
  bool highTorque = (torque > TORQUE_THRESHOLD);

  // Log values for debugging
  std::cout << "Intake Motor Status: Current=" << current
            << "mA, Velocity=" << velocity << ", Target=" << target_velocity
            << ", Ratio=" << (velocity / target_velocity)
            << ", Torque=" << torque << "Nm" << std::endl;
  return ((highCurrent && lowVelocity) || highTorque);
}

void noHookJam() {
  for (int i = 10; i < 45; i += 5) {
    chassis.waitUntil(i);
    // Checks every 5 inches for if the hook gets stuck.
    while (hooks.get_target_velocity() - hooks.get_actual_velocity() > 300) {
      // FIX: The hook is lifted back and then hit into the target to ensure
      // fit.
      hooks.move_velocity(-600);
      pros::delay(150);
      hooks.move_velocity(400);
      pros::delay(350);
    }
    // Lift up the Lady brown to load onto at 40 inches from the last position.
  }
}

void skills() {
  // Start the MCL background task
  // startMCL(chassis);

  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  hooks.set_zero_position(hooks.get_position());
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
  hooks.move_velocity(600);
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
  hooks.move_voltage(-12000);
  pros::delay(200); // Move at max voltage for 1 second
  hooks.move_voltage(0);
  pros::delay(500);
  clamp.retract();
  hooks.move_velocity(600);
  chassis.moveToPoint(-29, 13, 4000,
                      {.forwards = false, .maxSpeed = 50, .earlyExitRange = 2},
                      false);
  clamp.extend();
  hooks.move_voltage(-12000);
  pros::delay(200); // Move at max voltage for 1 second
  hooks.move_voltage(0);
  pros::delay(500);
  hooks.move_velocity(600);
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
  hooks.move_voltage(-12000);
  pros::delay(200); // Move at max voltage for 1 second
  hooks.move_voltage(0);
  pros::delay(500);
  chassis.turnToPoint(-3, 62, 3000, {.forwards = true, .maxSpeed = 90}, false);
  pros::delay(1000);
  chassis.moveToPoint(-3, 62, 2000, {.forwards = true, .maxSpeed = 70}, false);
  // tune this delay
  hooks.move_voltage(12000);
  pros::delay(600); // Move at max voltage for 1 second
  hooks.move_voltage(0);
  pros::delay(500);
  chassis.moveToPoint(45, 104, 3000, {.forwards = true, .maxSpeed = 70}, false);
  pros::delay(1000);
  chassis.turnToPoint(-25, 108, 5000, {.forwards = false}, false);
  chassis.moveToPoint(-25, 108, 5000, {.forwards = false, .maxSpeed = 70},
                      false);
  clamp.extend();
  hooks.move_velocity(600);
  chassis.moveToPoint(-64.5, 131.1, 2000, {.forwards = false}, false);
  chassis.moveToPoint(-55, 120, 500, {.forwards = true}, false);
  chassis.moveToPoint(-64.5, 131.1, 700, {.forwards = false}, false);
  clamp.retract();
  hooks.move_voltage(-12000);
  pros::delay(200); // Move at max voltage for 1 second
  hooks.move_voltage(0);
  chassis.moveToPoint(0, 120, 2000, {.forwards = true}, false);
  chassis.moveToPoint(-64.5, 131.1, 700, {.forwards = true}, false);

  stopMCL();
}

void skills1() {

  pros::Task antiJamHooks([&]() {
    while (true) {
      int targetVel = hooks.get_target_velocity();

      while (hooks.get_target_velocity() - hooks.get_actual_velocity() > 500 && (hooks.get_torque() > 0.6f || hooks.get_power() > 9)) {

        hooks.move_velocity(-600);
        pros::delay(150);
        hooks.move_velocity(400);
        pros::delay(350);
        hooks.move_velocity(targetVel);
      }

      pros::delay(50);
    }
  });

  // Set pre-constants
  hooks.set_zero_position_all(0);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  /*
  chassis.setPose(0, ((dSouth.get_distance() / 25.4f) - 5.19),
                  0); // front of robot is 6.5 in away from end of first tile
  chassis.moveToPose(0, 0, 0, 1500, {.maxSpeed=40, .earlyExitRange=.1}, false);
  */
  chassis.setPose(0, -.6, 0);

  // Autonomous routine for the Skills challenge - 60 seconds MAX
  /* ############################################## */

  // De-hook into the alliance stake
  hooks.move_absolute(600, 600);
  pros::delay(750);

  // Move to the center, then turn to and touch the right side Mogo.
  chassis.moveToPoint(0, 15.5, 800);
  chassis.turnToPoint(19, 15.5, 650, {.forwards = false});
  chassis.moveToPoint(16.5, 15.5, 850, {.forwards = false, .earlyExitRange = 2},
                      false);
  chassis.moveToPoint(20, 15.5, 550, {.forwards = false, .maxSpeed = 63}, true);

  chassis.waitUntil(2);

  // Latch the mogo - delays just in case
  clamp.extend();
  pros::delay(250);

  // Turn to the above ring - Start intake and hook, and move there
  chassis.turnToPoint(24, 38.75, 850, {}, false);
  hooks.move_velocity(600);
  preroller.move_velocity(200);
  chassis.moveToPoint(24, 38.75, 1000);

  // Immediately move to the farthest ring. A couple of other things happen
  // while this is going on
  chassis.moveToPose(45, 95, 0, 2050,
                     {.horizontalDrift = 8, .lead = 0.4, .earlyExitRange = .5},
                     true);
  if (antiHookJam()) { // intake is jammed
    hooks.move_relative(-50, 600);
  }

  chassis.waitUntil(50);

  // Set the hook to a high-torque rpm to get the best loading possible, waits
  // for the MoveToPose to end.
  preroller.move_velocity(200);

  hooks.brake();
  chassis.waitUntilDone();
  // Turn to a point ~1.5 tiles away from the right neutral stake

  lady_brown.move_absolute(72, 200); // optimal scoring when robot is 7.5" away
  // from wall / when north sensor reads 17"
  hooks.move_velocity(400);

  chassis.waitUntilDone();
  if (antiHookJam()) { // intake is jammed
    hooks.move_relative(-50, 600);
  }

  chassis.moveToPoint(45, 95, 700, {}, true);

  pros::delay(1800);
  hooks.brake();
  hooks.move_relative(-10, 400);
  chassis.turnToPoint(40, 65, 800, {.forwards = false});

  // Actually move there
  chassis.moveToPoint(35, 65, 1500, {.forwards = false}, true);
  // Immediately hit against the lady brown to ensure appropriate fit.
  //  Wait until 20 inches passed to get to the point behind the lady brown -
  //  stop the hook here.
  chassis.waitUntil(30);
  preroller.move_velocity(200);

  chassis.waitUntilDone();

  // Turn to the Neutral wall stake.
  chassis.turnToPoint(70, 64, 600);

  // Move there
  chassis.moveToPoint(56, 64, 900, {.forwards = true, .maxSpeed = 50}, false);
  chassis.turnToPoint(64, 64, 500);

  chassis.moveToPose(59, 64, 90, 900, {.forwards = true, .maxSpeed = 50}, true);

  chassis.waitUntilDone();

  // Check distance with north sensor and adjust position if needed
  float wallDistance =
      (float)(dNorth.get_distance()) / 25.4f; // Convert to inches

  // Only deploy lady_brown if distance is within ideal range
  if (wallDistance >= 14.9f && wallDistance <= 16.2f) {
    // We're in the ideal range, score normally
    lady_brown.move_absolute(500, 200);
    hooks.move_relative(-70, 400);

  } else {
    // We're out of position - calculate where we actually are
    float idealDistance = 15.2f; // Middle of our desired range
    float positionError =
        wallDistance -
        idealDistance; // Positive means too far, negative means too close

    // Set new position based on sensor reading
    float currentX = chassis.getPose().x;

    // Update X position (since we're facing 90 degrees, X coordinate changes
    // with distance)
    float correctedX = 59.9f - positionError;

    chassis.setPose(correctedX, chassis.getPose().y, chassis.getPose().theta);

    // Now move to the correct position
    if (positionError > 0) {
      // Too far from wall, move forward
      chassis.moveToPoint(59.9, chassis.getPose().y, 600,
                          {.forwards = true, .maxSpeed = 30}, true);
    } else {
      // Too close to wall, back up
      chassis.moveToPoint(59.9, chassis.getPose().y, 600,
                          {.forwards = false, .maxSpeed = 30}, true);
    }
    chassis.waitUntilDone();

    // Now score after correction
    lady_brown.move_absolute(500, 200);
    hooks.move_relative(-70, 400);
  }
  // Always deploy, even if correction failed
  lady_brown.move_absolute(500, 200);
  hooks.move_relative(-70, 400);

  // Give the lady_brown time to deploy
  pros::delay(200);

  chassis.moveToPoint(55.5, chassis.getPose().y, 500);

  // Move back to the 5th tile edge, and restart the intake 15 inches into the
  // movement.
  chassis.moveToPoint(61, 64, 1000);
  hooks.move_velocity(600);
  chassis.moveToPoint(49, 64, 750, {.forwards = false, .maxSpeed = 75});
  lady_brown.move_absolute(0, 200);

  chassis.waitUntil(5);

  chassis.turnToPoint(50, -2, 950);

  // Go close to the wall, but exit early so that the robot glides to the last
  // ring.
  chassis.moveToPoint(50, -3, 2500,
                      {.forwards = true, .maxSpeed = 45, .earlyExitRange = 1});

  // Start the hook at a higher speed rpm
  preroller.move_velocity(200);
  hooks.move_velocity(600);
  // Start at 10 inches into the movement - schedule a check every 5 inches
  // if the hook is stuck.
  chassis.waitUntil(10);

  chassis.waitUntilDone();
  if (antiHookJam()) { // intake is jammed
    hooks.move_relative(-5000, 600);
  }
  hooks.move_velocity(600);

  // Wait for two seconds at the edge to ensure rings get put onto the mogo.
  pros::delay(1000);

  // Turn, then Move to the point where the last ring was nudged to collect it
  // for top ring
  chassis.turnToPoint(61, 19, 1000);
  chassis.moveToPoint(61, 19, 1000,
                      {.forwards = true, .maxSpeed = 50, .earlyExitRange = 4});

  // Turn away to position the mogo in the corner, waiting to let the hook put
  // all items onto the mogo.

  chassis.turnToHeading(
      -30, 1000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false);

  hooks.move_relative(-500, 600);
  hooks.move_velocity(600);

  pros::delay(1000);
  // Let the last ring go when the hook gets stuck
  // Let the mogo go
  hooks.move_velocity(-600);

  clamp.retract();

  preroller.brake();

  // Move back a little to ensure the mogo goes into the corner.
  chassis.moveToPoint(62, 5, 1000,
                      {.forwards = false, .maxSpeed = 30, .earlyExitRange = 2},
                      false);
  chassis.waitUntil(1);
  hooks.brake();
  chassis.waitUntilDone();

  // Move to the top edge of the first tile, preparing for alignment. Then move
  // back into the wall to use the distance sensor to reorient. 0 theta and fix
  // the x
  chassis.moveToPose(46, 28, 90, 1000,
                     {.horizontalDrift = 9}, true);

  chassis.turnToHeading(90, 1000, {.earlyExitRange=0.1});

  chassis.moveToPose(65, 28, 90, 1350, {.minSpeed=80, .earlyExitRange=1}, false);

  chassis.waitUntilDone();

  // Get the distance to the wall using the distance sensor, convert to inches
  // and add the distance to tracking center (5in). Then, set the new position
  // Step 1: Reset X position and theta using south sensor
  float new_x = 62.3 - (dNorthW.get_distance() / 25.4) + 4.25;
  std::cout << "South distance: " << dNorthW.get_distance() / 25.4 << " inches"
            << std::endl;
  std::cout << "Setting X pose to: " << new_x << std::endl;

  // Reset X and theta, keep current Y
  chassis.setPose(new_x, chassis.getPose().y, 0);

  // Move forward to clear the mobile goal
  chassis.moveToPoint(chassis.getPose().x - 24, chassis.getPose().y, 800, {},
                      false);
  chassis.waitUntilDone();

  // Step 2: Now reset Y position using west sensor
  float new_y = 14.625f - ((dWest.get_distance() / 25.4f) - 15.8f);
  std::cout << "West distance: " << dWest.get_distance() / 25.4 << " inches"
            << std::endl;
  std::cout << "Setting Y pose to: " << new_y << std::endl;

  // Reset Y, keep current X and theta
  chassis.setPose(chassis.getPose().x, new_y, chassis.getPose().theta);

  /*
  // Continue with movement
  chassis.moveToPoint(chassis.getPose().x - 5, chassis.getPose().y, 600,
                      {.earlyExitRange = 1});

  chassis.moveToPose(24, 15, 90, 1000, {.horizontalDrift = 8});

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
  hooks.move_velocity(200);
  chassis.moveToPoint(-24, 38.75, 1000);

  // Immediately move to the farthest ring. A couple of other things happen
  // while this is going on
  chassis.moveToPose(-48, 90, 0, 2050, {.horizontalDrift = 8, .lead = 0.4},
                     true);
  // Checks between 10-45 inches of movement for if the hook got stuck on the
  // mogo flower
  for (int i = 10; i < 45; i += 5) {
    chassis.waitUntil(i);
    std::cout << "velocity" << hooks.get_actual_velocity() << std::endl;
    // Checks every 5 inches for if the hook gets stuck.
    while (hooks.get_actual_velocity() < 50) {
      // FIX: The hook is lifted back and then hit into the target to ensure
      // fit.
      hooks.move_velocity(-600);
      pros::delay(150);
      hooks.move_velocity(400);
      pros::delay(350);
    }
    // Lift up the Lady brown to load onto at 40 inches from the last position.
  }

  // Set the hook to a high-torque rpm to get the best loading possible, waits
  // for the MoveToPose to end.
  chassis.waitUntil(50);

  hooks.move_velocity(600);

  chassis.waitUntilDone();
  // Turn to a point ~1.5 tiles away from the right neutral stake
  lady_brown.move_absolute(72, 200);
  chassis.turnToPoint(-40, 62.5, 800, {.forwards = false});

  // Actually move there
  chassis.moveToPoint(-40, 62.5, 1500, {.forwards = false}, true);

  // Immediately hit against the lady brown to ensure appropriate fit.
  chassis.waitUntil(30);
  hooks.brake();

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

  // Allow the robot to settle, then move the lady brown to score on the neutral
  // stake.
  lady_brown.move_absolute(500, 200);

  // Move back to the 5th tile edge, and restart the intake 15 inches into the
  // movement.
  chassis.moveToPoint(-47, 63.5, 750, {.forwards = false, .maxSpeed = 75});
  chassis.waitUntil(5);
  preroller.move_velocity(200);
  hooks.move_velocity(600);

  // Point the robot towards the home side wall, and put the lady brown down.
  chassis.turnToPoint(-47, 0, 950);
  lady_brown.move_absolute(0, 200);

  // Go close to the wall, but exit early so that the robot glides to the last
  // ring.
  chassis.moveToPoint(-47, -5, 2750,
                      {.forwards = true, .maxSpeed = 45, .earlyExitRange = 2});

  // Start the hook at a higher speed rpm
  hooks.move_velocity(600);

  // Start at 10 inches into the movement - schedule a check every 5 inches for
  // if the hook is stuck.
  chassis.waitUntil(10);
  for (int i = 12; i < 47; i += 5) {
    chassis.waitUntil(i);
    std::cout << "velocity" << hooks.get_actual_velocity() << std::endl;
    while (hooks.get_actual_velocity() < 50) {
      // FIX: The hook is lifted back and then hit into the target to ensure
      // fit.
      hooks.move_velocity(-600);
      pros::delay(150);
      hooks.move_velocity(400);
      pros::delay(350);
    }
  }

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
  hooks.move_velocity(-200);
  pros::delay(200);
  // Let the mogo go
  clamp.retract();

  // Move back a little to ensure the mogo goes into the corner.
  chassis.moveToPoint(
      -64, 6, 1000, {.forwards = false, .maxSpeed = 50, .earlyExitRange = 1.5},
      false);

  // Stop the intake and hook.
  preroller.brake();
  hooks.brake();

  // Move to the top edge of the first tile, preparing for alignment. Then move
  // back into the wall to use the distance sensor to reorient.
  chassis.moveToPose(-48, 20, -90, 2050, {.forwards = true, .maxSpeed = 80},
                     true);
*/
}