#ifndef GLOBALS_H
#define GLOBALS_H

#include "api.h"
#include "lemlib/api.hpp"
#include "pros/motors.hpp"
#include "pros/adi.hpp"

// extern tells the compiler that these variables are defined elsewhere (in globals.cpp)
extern pros::MotorGroup dt_left;
extern pros::MotorGroup dt_right;
extern pros::Motor lady_brown;
extern pros::MotorGroup intake;
extern pros::Motor preroller;
extern pros::Controller controller;
extern pros::Imu imu;
extern int intakeSpeed;
extern pros::adi::Pneumatics clamp;
extern lemlib::Drivetrain drivetrain;
extern lemlib::ControllerSettings linearController;
extern lemlib::ControllerSettings angularController;
extern pros::Rotation horizontalRotation;
extern lemlib::TrackingWheel horizontal1;
extern lemlib::OdomSensors sensors;
extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;
extern lemlib::Chassis chassis;

extern pros::Distance dNorth;
extern pros::Distance dEast;
extern pros::Distance dSouth;
extern pros::Distance dWest;



#endif // GLOBALS_H