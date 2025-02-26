#include "main.h"
#include "api.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/lvgl.h"
#include "main.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <iostream>
#include "robot/skills.h"

using namespace lemlib;

void x() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
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
}