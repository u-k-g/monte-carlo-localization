#include "globals.h"
#include "lemlib/chassis/chassis.hpp"
#include "main.h" // IWYU pragma: export
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot/auton.h"

#include <cmath>
#include <iostream>
#include <string>

using namespace lemlib;

void redNeg() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(0, 0, 0);
  // Move to first stake, then a bit farther at a slower speed for alignment
  chassis.moveToPoint(0, -27, 1800, {.forwards = false, .maxSpeed = 60}, true);
  chassis.moveToPoint(0, -35, 2050, {.forwards = false, .maxSpeed = 30}, true);
  chassis.waitUntilDone();
  pros::delay(500);
  // Grab the closest MOGO mech
  clamp.extend();
  pros::delay(1500);
  // Load the ring onto the stake
  preroller.move_velocity(200);
  hooks.move_velocity(600);
  pros::delay(3000);
}

void redPos() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(0, 0, 0);

  // Move to first stake
  chassis.moveToPose(0, -28, 0, 2000, {.forwards = false});
  chassis.waitUntilDone();
  // Grab the closest MOGO mech
  clamp.toggle();
  pros::delay(250);

  // Rotate toward blue ring
  chassis.turnToPoint(-25, -27, 1000);
  chassis.waitUntilDone();

  // Load the ring onto the stake
  preroller.move_velocity(200);
  hooks.move_velocity(600);

  // Moves to 2 ring stack

  chassis.moveToPoint(-35, 7, 1500);
  chassis.waitUntilDone();
  preroller.brake();
  hooks.brake();

  chassis.moveToPose(-25, -4, 170, 2000, {.horizontalDrift = 2});
  chassis.moveToPoint(-32, 10, 1000, {.forwards = false});
  chassis.waitUntilDone();
  clamp.toggle();

  chassis.turnToPoint(14, -36, 800);
  lady_brown.move_absolute(72, 200);

  chassis.moveToPoint(14, -36, 5000);
}

void bluePos() {

  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(0, 0, 0);

  // Move to first stake
  chassis.moveToPose(0, -28, 0, 2000, {.forwards = false});
  chassis.waitUntilDone();
  // Grab the closest MOGO mech
  clamp.toggle();
  pros::delay(250);

  // Rotate toward blue ring
  chassis.turnToPoint(25, -27, 1000);
  chassis.waitUntilDone();

  // Load the ring onto the stake
  preroller.move_velocity(200);
  hooks.move_velocity(600);

  chassis.moveToPoint(35, 7, 1500);
  chassis.waitUntilDone();
  preroller.move_velocity(0);
  hooks.move_velocity(0);

  chassis.moveToPose(25, -4, 170, 2000, {.horizontalDrift = 2});
  chassis.moveToPoint(32, 10, 1000, {.forwards = false});
  chassis.waitUntilDone();
  clamp.toggle();

  chassis.turnToPoint(-14, -36, 800);
  lady_brown.move_absolute(72, 200);
  chassis.moveToPoint(-14, -36, 5000);
}

void blueNeg() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(0, 0, 0);
  // Move to first stake, then a bit farther at a slower speed for alignment
  chassis.moveToPoint(0, -27, 1800, {.forwards = false, .maxSpeed = 60}, true);
  chassis.moveToPoint(0, -35, 2050, {.forwards = false, .maxSpeed = 30}, true);
  chassis.waitUntilDone();
  pros::delay(500);
  // Grab the closest MOGO mech
  clamp.extend();
  pros::delay(2000);
  // Load the ring onto the stake
  preroller.move_velocity(200);
  hooks.move_velocity(600);
  pros::delay(3000);
}

void test360() {
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, -12, 1000, {.forwards = false});
  chassis.turnToHeading(360, 5000,
                        {.direction = lemlib::AngularDirection::CW_CLOCKWISE,
                         .maxSpeed = 70,
                         .earlyExitRange = .00000000001});

  {
    {                                                  // Get sensor readings
      double distNorth = dNorth.get_distance() / 25.4; // Convert to inches
      double distNorthW = dNorthW.get_distance() / 25.4;

      // Expected sensor reading at (0,0,0) - when aligned with wall
      double expectedDistance = 9.1;

      // Calculate angle error (in degrees) -  sensors are facing Y axis now
      // If dNorthW > dNorth, robot is rotated clockwise, theta error is
      // negative
      double theta_error_rad =
          atan2((distNorthW - distNorth),
                13.75); // Still use horizontal separation for angle
      double theta_error_deg = theta_error_rad * (180 / M_PI);

      // Calculate Y position error (in inches) - sensors facing Y axis
      double averageDistance = (distNorth + distNorthW) / 2.0;
      double y_error = averageDistance - expectedDistance;

      // Get current pose
      lemlib::Pose current_pose = chassis.getPose();

      // Calculate corrected pose - now correcting Y and Theta
      double corrected_theta = current_pose.theta - theta_error_deg;
      double corrected_y = current_pose.y - y_error;
      double corrected_x = current_pose.x; // Keep X unchanged

      // Set corrected pose
      chassis.setPose(corrected_x, corrected_y, corrected_theta);

      std::cout << "dNorth: " << distNorth << " inches, dNorthW: " << distNorthW
                << " inches" << std::endl;
      std::cout << "Theta Error: " << theta_error_deg
                << " degrees, Y Error: " << y_error << " inches" << std::endl;
      std::cout << "Corrected Pose: x=" << corrected_x << ", y=" << corrected_y
                << ", theta=" << corrected_theta << std::endl;
    }
  }
}

void hooks_score(int degrees, int direction) {
  hooks.move_relative(degrees, 600 * direction);
  hooks.brake();
}

void hooks_on(int speed) {
  if (speed == 0) {
    hooks.brake();
    return;
  }

  hooks.move_velocity(600);
}

void hooks_off() { hooks_on(0); }

void Auton1() {
  // Autonomous winpoint blue positive side / red positive side

  // score on alliance stake

  chassis.setPose(-60, -12, 0);
  chassis.moveToPose(-60, 0, 0, 5000);
  chassis.turnToHeading(90, 1000);
  chassis.moveToPoint(-65, 0, 1000, {.forwards = false});
  hooks_score(2000, 1);

  // pick up ring and score

  chassis.setPose(-62, 0, 90, false);
  hooks_on(600);
  chassis.moveToPose(-24, -48, 135, 2700, {}, false);
  hooks_off();

  clamp.toggle();
  chassis.turnToHeading(180, 2000);
  chassis.moveToPoint(-24, -22, 5000, {.forwards = false, .maxSpeed = 25},
                      false);
  clamp.toggle();

  pros::delay(500);
  hooks_score(1000, 1);

  chassis.turnToHeading(0, 1000);
  chassis.moveToPoint(-20, -2, 5000, {.forwards = true, .maxSpeed = 40}, false);
}

void Auton2() {

  pros::delay(5000);
  // Autonomous winpoint blue negative side / red negative side

  // score on alliance stake

  chassis.setPose(-60, 24, 180);
  chassis.moveToPose(-60, 0, 180, 5000);
  chassis.turnToHeading(90, 1000);
  chassis.moveToPoint(-65, 0, 1000, {.forwards = false});
  hooks_score(2000, 1);

  // pick up ring and score

  chassis.setPose(-62, 0, 90, false);
  hooks_on(600);
  chassis.moveToPose(-24, 48, 45, 2700, {}, false);
  hooks_off();

  clamp.toggle();
  chassis.turnToHeading(0, 2000);
  chassis.moveToPoint(-24, 22, 5000, {.forwards = false, .maxSpeed = 25},
                      false);
  clamp.toggle();

  pros::delay(500);
  hooks_score(1000, 1);

  chassis.turnToHeading(180, 1000);
  chassis.moveToPoint(-20, 2, 5000, {.forwards = true}, false);
}

void Auton3() {
  chassis.setPose(0, 0, 0, false);
  chassis.moveToPose(0, -36, 0, 2700, {.forwards = false, .maxSpeed = 70},
                     false);
  clamp.extend();
  hooks.move_velocity(600);
}

void Auton5() {
  // Skills challenge autonomous

  // Chassis position: coordinate from the back of the drivetrain
  // Chassis heading: front hooks is direction

  // Step 1. We start under the red alliance stake.
  // With the preloaded ring, we will score on the stake using our wall stake
  // mechanism.

  chassis.setPose(-165, 0, 90, false);
  hooks_score(1000, 1);
  // -- TODO: Score on the red alliance stake

  // Step 2. We will go to pick up the top left mobile goal
  // with our clamp facing into the mobile goal.

  chassis.turnToHeading(180, 5000, {}, false);

  chassis.moveToPoint(-120, 60, 5000, {.forwards = true}, false);
  pros::delay(200);

  clamp.toggle();

  // Step 3. We will go and score the 6 rings around the mobile goal onto our
  // robot. This will take a lot of precise coding and movement to nail
  // autonomously

  hooks_score(1000, 1);

  // -- Score bottom right ring (1)

  chassis.turnToHeading(90, 5000, {}, false);

  chassis.moveToPoint(-60, 60, 5000, {.forwards = true}, false);
  pros::delay(200);

  // -- Score second top ring (2)

  chassis.turnToHeading(0, 5000, {}, false);

  chassis.moveToPoint(-60, 120, 5000, {.forwards = true}, false);
  pros::delay(200);

  // -- Score center top ring (3)

  chassis.turnToHeading(90, 5000, {}, false);

  chassis.moveToPoint(0, 150, 5000, {.forwards = true}, false);
  pros::delay(200);

  // -- Score corner center ring (4)

  chassis.turnToHeading(270, 5000, {}, false);

  chassis.moveToPoint(-120, 120, 5000, {.forwards = true}, false);
  pros::delay(200);

  // -- Score corner back left ring (5)

  chassis.moveToPoint(-150, 120, 5000, {.forwards = true}, false);
  pros::delay(200);

  // -- Score corner top ring (6)

  chassis.moveToPoint(-120, 150, 5000, {.forwards = true}, false);
  pros::delay(200);

  // Step 4. We will go and put the fully scored out mobile goal into the top
  // right corner to double its points.

  chassis.moveToPose(-168, -168, 135, 5000, {.forwards = false}, false);
  pros::delay(200);

  clamp.toggle();

  // Step 5. We will go to the center, and pick up the center ring on our robot.
  // This will later be used to score on the bottom right mobile goal.

  chassis.moveToPose(0, 0, 0, 5000, {.forwards = true}, false);
  pros::delay(500);
  hooks_score(1000, 1);

  // Step 6. We will pick up the bottom right's mobile goal to score more rings
  // onto.

  chassis.moveToPose(-120, 60, 45, 5000, {.forwards = false}, false);

  clamp.toggle();

  // Step 7. We will pick up all of the rings in the bottom right corner.
  // This will required high precision and a well-tuned autonomous to accomplish
  // quickly.

  // -- Pick up the top right ring (2)

  hooks_score(1000, 1);

  chassis.moveToPose(-60, -60, 135, 5000, {.forwards = true}, false);
  pros::delay(200);

  // -- Pick up the bottom right ring (3)

  chassis.moveToPose(-60, -120, 180, 5000, {.forwards = true}, false);
  pros::delay(200);

  // -- Pick up the middle ring (4)
  chassis.moveToPose(-120, -120, 270, 5000, {.forwards = true}, false);
  pros::delay(200);

  // -- Pick up a ring (5)
  chassis.moveToPose(-150, -120, 270, 5000, {.forwards = true}, false);
  pros::delay(200);

  // -- Pick up last ring (6)
  chassis.moveToPose(-120, -150, 135, 5000, {.forwards = true}, false);
  pros::delay(200);

  // Step 8. We will put the mobile goal into the positive corner at the bottom
  // right. This will double all of the points on our current mobile goal.

  chassis.moveToPose(-166, -166, 45, 5000, {.forwards = false}, false);

  // Step 9. Move to the center line, and pick up a ring, then turn around and
  // score it on the high stakes.

  chassis.moveToPose(0, -150, 90, 5000, {.forwards = true}, false);
  // TODO: SCORE RING ON HIGH STAKE

  // Step 10. We will go to the center bar and hang
  chassis.moveToPose(-25, -40, 45, 5000, {.forwards = true}, false);
  // TODO: TOGGLE HANG MECHANISM
}

void match1() {

  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.setPose(-62.4, 40.5, 270);
  chassis.moveToPoint(-45, 40.5, 1000);
  chassis.turnToPoint(-31.8, 29.4, 1000, {.forwards = false}, false);
  chassis.moveToPoint(-31.8, 29.4, 1000, {.forwards = false}, false);

  clamp.extend();

  chassis.turnToPoint(-23.3, 48.8, 1000, {}, false);
  hooks.move_velocity(600);
  chassis.moveToPoint(-20.3, 50.8, 1000, {.maxSpeed = 50}, false);

  chassis.turnToPoint(-67.1, 67, 1000);
  chassis.moveToPoint(-67.1, 67, 1000);

  pros::delay(3000);

  chassis.moveToPose(-49, -23, 180, 3000);
}

void match2() {
  // red neg

  chassis.setPose(0, 0, 0, false);
  chassis.moveToPose(0, -36, 0, 2700, {.forwards = false, .maxSpeed = 70},
                     false);
  clamp.extend();
  hooks.move_velocity(600);

  pros::delay(6000);

  chassis.turnToHeading(90, 1000);
  chassis.moveToPoint(0, 40, 1000);
}