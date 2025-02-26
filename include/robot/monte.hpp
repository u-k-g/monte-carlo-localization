#pragma once

#include "lemlib/api.hpp"
#include <vector>

// Particle structure definition
struct Particle {
    lemlib::Pose pose;  // Use the lemlib::Pose for x, y, theta
    float weight; // Weight of the particle (probability)

    // Constructor - required since lemlib::Pose has no default constructor
    Particle(const lemlib::Pose& p = lemlib::Pose(0, 0, 0), float w = 0.0f) 
        : pose(p), weight(w) {}
};

// Constants
const int PARTICLE_QUANTITY = 500; //tune
const int WALL_DISTANCE_FROM_FIELD_EXTENT = 2; // the distance sensor senses this many inches away from the field extents
const int FIELD_DIMENSIONS = 144; //12'x12' field

// Function declarations
/**
 * Initialize particles around an initial pose estimate
 * @param initialPose The initial pose estimate from chassis.getPose()
 */
void initializeParticles(const lemlib::Pose& initialPose);

/**
 * Update particles based on robot motion (prediction step)
 * @param deltaMotion The change in pose since the last update
 */
void motionUpdate(const lemlib::Pose& deltaMotion);

/**
 * Calculate expected sensor readings for a particle
 * @param particlePose The pose of the particle
 * @param direction Character indicating sensor direction ('N', 'S', 'E', 'W')
 * @return Expected distance reading
 */
float predictSensorReading(const lemlib::Pose& particlePose, const char direction);

/**
 * Update particle weights based on sensor measurements
 * @param north_dist Distance reading from north sensor
 * @param south_dist Distance reading from south sensor
 * @param east_dist Distance reading from east sensor
 * @param west_dist Distance reading from west sensor
 */
void measurementUpdate(float north_dist, float south_dist, float east_dist, float west_dist);

/**
 * Resample particles based on their weights
 */
void resampleParticles();

/**
 * Get the estimated pose from particle distribution
 * @return Estimated pose of the robot
 */
lemlib::Pose getEstimatedPose();

/**
 * Update the Monte Carlo Localization system with new sensor data
 * @param chassis The chassis object to get/set pose from
 * @param north_dist Distance reading from north sensor
 * @param south_dist Distance reading from south sensor
 * @param east_dist Distance reading from east sensor
 * @param west_dist Distance reading from west sensor
 */
void updateMCL(lemlib::Chassis& chassis, float north_dist, float south_dist, float east_dist, float west_dist);

/**
 * Background task for running Monte Carlo Localization
 * @param param Unused parameter required by PROS task API
 */
void mclTask(void* param);

/**
 * Start the MCL background task
 * @param chassis The chassis to use for localization
 */
void startMCL(lemlib::Chassis& chassis);

/**
 * Stop the MCL background task
 */
void stopMCL();