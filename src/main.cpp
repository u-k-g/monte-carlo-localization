#include "main.h"
#include "api.h"
#include "globals.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/timer.hpp"
#include "liblvgl/lvgl.h"
#include "main.h"
#include "pros/device.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "robodash/core.h"
#include "robot/auton.h"
#include "robot/monte.hpp"
#include "robot/skills.h"
#include <iostream>

using namespace lemlib;

rd::Selector selector({{"redNeg", &redNeg},
                       {"redPos", &redPos},
                       {"blueNeg", &blueNeg},
                       {"bluePos", &bluePos},
                       {"skills", &skills1}});

rd::Console console;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize(); // initialize brain screen
  selector.focus();
  chassis.calibrate(); // calibrate sensors
  chassis.setPose(0, 0, 0);
  lady_brown.set_zero_position_all(0);
  selector.on_select([](std::optional<rd::Selector::routine_t> routine) {
		if (routine == std::nullopt) {
			std::cout << "No routine selected" << std::endl;
		} else {
			std::cout << "Selected Routine: " << routine.value().name << std::endl;
		}
	});
  // /*
  pros::Task screenTask([&]() {
    while (true) {

      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
      pros::lcd::print(3, "LB Deg: %f", lady_brown.get_position());

      // Print distance sensor values in inches
      pros::lcd::print(4, "N: %.2f in", dNorth.get_distance() / 25.4);
      pros::lcd::print(5, "E: %.2f in", dEast.get_distance() / 25.4);
      pros::lcd::print(6, "S: %.2f in", dNorthW.get_distance() / 25.4);
      pros::lcd::print(7, "W: %.2f in", dWest.get_distance() / 25.4);

      // log position telemetry
      lemlib::telemetrySink()->info("Chassis pose: {}",
                                    chassis.getPose()); // log file to sd card
      // delay to save resources
      pros::delay(50);
    }
  });
  // */
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // unclamp
  // clamp.retract();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() { selector.focus(); }

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() {
  bluePos();
  selector.run_auton();
}

// Create MCL instance using the existing distance sensors

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // autonomous();
  stopMCL();
  // startMCL(chassis);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  bool flagged = false;
  lady_brown.move_absolute(0, 200);
  hooks.move_velocity(0);
  enum LadyBrownState { IDLE, PRIMED, SCORED };
  lemlib::Timer matchTimer(66000);
  lemlib::Timer cornerProtection(76000);

  // Static variable to track current state
  static LadyBrownState ladyBrownState = IDLE;

  // loop forever
  while (true) {

    if (matchTimer.isDone() && flagged == false) { // warning
      // controller.rumble(". - . -");
    } else if (cornerProtection.isDone()) {
      flagged = true;
    }
    // get left y and right y positions
    float leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    float rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // move the robot
    chassis.tank(leftY, rightY);

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {

      preroller.move_velocity(200);
      hooks.move_velocity(600);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      hooks.move_velocity(-600);
      preroller.move_velocity(-200);
    } else {
      preroller.brake();
      hooks.brake();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
      clamp.toggle();
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
      switch (ladyBrownState) {
      case IDLE:
        lady_brown.move_absolute(72, 200); // Move to primed position
        ladyBrownState = PRIMED;
        break;

      case PRIMED:
        lady_brown.move_absolute(425, 200); // Maintain primed position
        hooks.move_relative(-200, -600);
        pros::delay(100);
        ladyBrownState = SCORED; // optimal scoring when robot is x" away from
                                 // wall / when north sensor reads 17" or 14"
        break;

      case SCORED:
        lady_brown.move_absolute(0, 80); // Move to scoring position
        ladyBrownState = IDLE;
        break;
      }
    }
    pros::delay(20);
  }
}