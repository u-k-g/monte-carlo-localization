#include "api.h"
#include "pros/rtos.hpp"
#include "globals.h"
#include "robot/mcl.h"
#include <array>

// Example usage of Monte Carlo Localization

// Field dimensions in mm (adjust to match your competition field)
const double FIELD_WIDTH = 3658.0;  // ~12 feet
const double FIELD_HEIGHT = 3658.0; // ~12 feet

// Number of particles to use (more particles = more accurate but slower)
const int NUM_PARTICLES = 500;

// Sensor positions relative to robot center in mm
// These should be measured from your robot's center to each sensor
// Format: [x, y] where positive x is forward, positive y is left
const std::array<double, 2> NORTH_SENSOR_POS = {0.0, 0.0};  // Sensor facing north
const std::array<double, 2> EAST_SENSOR_POS = {0.0, 0.0};   // Sensor facing east
const std::array<double, 2> SOUTH_SENSOR_POS = {0.0, 0.0};  // Sensor facing south
const std::array<double, 2> WEST_SENSOR_POS = {0.0, 0.0};   // Sensor facing west

// Global MCL instance
MonteCarloLocalization* mcl = nullptr;

// Previous position for calculating deltas
double prevX = 0.0;
double prevY = 0.0;
double prevTheta = 0.0;
bool firstUpdate = true;

// Initialize MCL system
void initializeMCL() {
    // Create MCL instance
    mcl = new MonteCarloLocalization(
        FIELD_WIDTH,
        FIELD_HEIGHT,
        NUM_PARTICLES,
        NORTH_SENSOR_POS,
        EAST_SENSOR_POS,
        SOUTH_SENSOR_POS,
        WEST_SENSOR_POS
    );
    
    // Initialize particles around starting position
    // Assuming robot starts at position (500, 500) with orientation 0 radians
    // with some uncertainty in each dimension
    mcl->initializeParticles(500.0, 500.0, 0.0, 100.0, 100.0, 0.2);
    
    // Reset previous position
    prevX = 500.0;
    prevY = 500.0;
    prevTheta = 0.0;
    firstUpdate = true;
}

// Update MCL with current odometry and sensor readings
void updateMCL() {
    if (mcl == nullptr) return;
    
    // Get current position from odometry (e.g., from chassis)
    auto pose = chassis.getPose();
    double currentX = pose.x;
    double currentY = pose.y;
    double currentTheta = pose.theta;
    
    // Calculate deltas since last update
    double deltaX = 0.0;
    double deltaY = 0.0;
    double deltaTheta = 0.0;
    
    if (!firstUpdate) {
        deltaX = currentX - prevX;
        deltaY = currentY - prevY;
        deltaTheta = currentTheta - prevTheta;
    }
    
    // Update previous position
    prevX = currentX;
    prevY = currentY;
    prevTheta = currentTheta;
    firstUpdate = false;
    
    // Get distance sensor readings (in mm)
    double northReading = dNorth.get();
    double eastReading = dEast.get();
    double southReading = dSouth.get();
    double westReading = dWest.get();
    
    // Update MCL
    mcl->update(deltaX, deltaY, deltaTheta, northReading, eastReading, southReading, westReading);
    
    // Get estimated pose
    auto estimatedPose = mcl->getEstimatedPose();
    
    // Get uncertainty
    auto uncertainty = mcl->getUncertainty();
    
    // Print estimated position (for debugging)
    std::cout << "MCL Position: (" << estimatedPose[0] << ", " << estimatedPose[1] 
              << ", " << estimatedPose[2] << ")" << std::endl;
    std::cout << "Uncertainty: (" << uncertainty[0] << ", " << uncertainty[1] 
              << ", " << uncertainty[2] << ")" << std::endl;
}

// MCL update task
void mclTask(void* param) {
    // Initialize MCL
    initializeMCL();
    
    // Main loop
    while (true) {
        // Update MCL
        updateMCL();
        
        // Sleep to avoid hogging CPU
        pros::delay(20); // 50Hz update rate
    }
}

// Start MCL in a separate task
void startMCL() {
    pros::Task mcl_task(mclTask, nullptr, "MCL Task");
} 