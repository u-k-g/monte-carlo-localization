#include "api.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/lvgl.h"
#include "main.h"
#include "pros/distance.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <iostream>

using namespace lemlib;
pros::MotorGroup dt_left({-5, 2, -9}, pros::v5::MotorGears::blue,
                         pros::v5::MotorUnits::degrees);
pros::MotorGroup dt_right({1, -6, 19}, pros::v5::MotorGears::blue,
                          pros::v5::MotorUnits::degrees);

pros::Motor lady_brown(14, pros::v5::MotorGears::green,
                       pros::v5::MotorUnits::degrees);
pros::Motor hooks(11, pros::v5::MotorGears::blue,
                  pros::v5::MotorUnits::degrees);
pros::Motor preroller(20, pros::v5::MotorGears::green,
                      pros::v5::MotorUnits::degrees); // intake motor on port 9
pros::Controller controller(pros::E_CONTROLLER_MASTER);
;

pros::Imu imu(15);

pros::adi::Pneumatics clamp('A', false);

lemlib::Drivetrain drivetrain(
    &dt_left,  // left motor group
    &dt_right, // right motor group
    12.875,
    lemlib::Omniwheel::OLD_325, 
    480,                       
    8 
);

lemlib::ControllerSettings
    linearController(6,   // proportional gain (kP)
                     0,   // integral gain (kI) try 0.1
                     8,   // derivative gain (kD)
                     3,   // anti windup
                     .25, // small error range, in inches .5
                     150, // small error range timeout, in milliseconds
                     3,   // large error range, in inches
                     500, // large error range timeout, in milliseconds
                     60   // maximum acceleration (slew)
    );

// angular motion controller
lemlib::ControllerSettings
    angularController(2.1, // proportional gain (kP)
                      0,   // integral gain (kI) try 0.1
                      16,  // derivative gain (kD)
                      3,   // anti windup
                      .1,  // small error range, in degrees .5
                      600, // small error range timeout, in milliseconds
                      3,   // large error range, in degrees
                      800, // large error range timeout, in milliseconds
                      40    // maximum acceleration (slew)
    );

// Create a new rotation sensor on port 11 (adjust the port number as needed)
pros::Rotation horizontalRotation(-4);

// Create a new horizontal tracking wheel using the rotation sensor .5 inches
// behind and 1 inch to the left of tracking center
lemlib::TrackingWheel horizontal1(&horizontalRotation, lemlib::Omniwheel::NEW_2,
                                  1.25);

// Update the OdomSensors object to include the new horizontal tracking wheel
lemlib::OdomSensors sensors(&horizontal1, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to
                                     // nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr,      // horizontal tracking wheel 2, set to
                                     // nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve
    throttleCurve(3,    // joystick deadband out of 127
                  10,   // minimum output where drivetrain will move out of 127
                  0.992 // expo curve gain
    );

// input curve for steer input during driver control
lemlib::ExpoDriveCurve
    steerCurve(3,    // joystick deadband out of 127
               10,   // minimum output where drivetrain will move out of 127
               1.019 // expo curve gain
    );

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController,
                        sensors, &throttleCurve, &steerCurve);

pros::Distance dNorth(3);
pros::Distance dEast(17);
pros::Distance dNorthW(10); //13.75
pros::Distance dWest(16);
