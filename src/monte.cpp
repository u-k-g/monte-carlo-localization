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
#include "robot/monte.hpp"
#include <random>
#include <cmath>

// Global variables (these stay in the cpp file)
namespace {
    // Global vector of particles; we'll resize it during initialization.
    std::vector<Particle> particles;
    std::random_device rd;
    std::mt19937 gen(rd()); // Mersenne Twister random number generator

    // Track the last pose for delta calculations; initialize explicitly.
    lemlib::Pose lastPose(0, 0, 0);
}

// Initialize particles around an initial pose estimate
void initializeParticles(const lemlib::Pose& initialPose) {
    particles.resize(PARTICLE_QUANTITY);
    lastPose = initialPose;
    
    std::normal_distribution<float> x_dist(initialPose.x, 3.0);
    std::normal_distribution<float> y_dist(initialPose.y, 3.0);
    std::normal_distribution<float> theta_dist(initialPose.theta, 10.0);

    for (auto& particle : particles) {
        particle = Particle(
            lemlib::Pose(x_dist(gen), y_dist(gen), theta_dist(gen)),
            1.0f / PARTICLE_QUANTITY
        );
    }
}

// Update particles based on robot motion (prediction step)
void motionUpdate(const lemlib::Pose& deltaMotion) {
    // Add noise to the motion update.
    std::normal_distribution<float> motion_noise(0.0, 0.5);   // 0.5-inch standard deviation
    std::normal_distribution<float> rotation_noise(0.0, 2.0);   // 2-degree standard deviation

    for (auto &particle : particles) {
        particle.pose.x += deltaMotion.x + motion_noise(gen);
        particle.pose.y += deltaMotion.y + motion_noise(gen);
        particle.pose.theta += deltaMotion.theta + rotation_noise(gen);
    }
}

// Calculate expected sensor readings for a particle
float predictSensorReading(const lemlib::Pose& particlePose, const char direction) {
    float expected_distance = 0.0f;
    
    switch(direction) {
        case 'N':
            expected_distance = FIELD_DIMENSIONS - particlePose.y;
            break;
        case 'S':
            expected_distance = particlePose.y;
            break;
        case 'E':
            expected_distance = FIELD_DIMENSIONS - particlePose.x;
            break;
        case 'W':
            expected_distance = particlePose.x;
            break;
    }
    return expected_distance;
}

// Update particle weights based on sensor measurements
void measurementUpdate(float north_dist, float south_dist, float east_dist, float west_dist) {
    float total_weight = 0.0f;
    
    for (auto &particle : particles) {
        float north_diff = std::abs(predictSensorReading(particle.pose, 'N') - north_dist);
        float south_diff = std::abs(predictSensorReading(particle.pose, 'S') - south_dist);
        float east_diff  = std::abs(predictSensorReading(particle.pose, 'E') - east_dist);
        float west_diff  = std::abs(predictSensorReading(particle.pose, 'W') - west_dist);
        
        // Use a Gaussian probability model with sigma as the sensor noise parameter.
        float sigma = 2.0f; 
        particle.weight = std::exp(-(north_diff*north_diff + south_diff*south_diff +
                                     east_diff*east_diff + west_diff*west_diff)  
                                     / (2.0f * sigma * sigma));
        
        total_weight += particle.weight;
    }
    
    // Normalize weights so that they sum to 1.
    if (total_weight > 0) {
        for (auto &particle : particles) {
            particle.weight /= total_weight;
        }
    }
}

// Resample particles based on their weights using systematic resampling.
void resampleParticles() {
    std::vector<Particle> new_particles(PARTICLE_QUANTITY);
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    
    float step = 1.0f / PARTICLE_QUANTITY;
    float r = dist(gen) * step;
    float c = particles[0].weight;
    int i = 0;
    
    for (int m = 0; m < PARTICLE_QUANTITY; m++) {
        float U = r + m * step;
        while (U > c && i < PARTICLE_QUANTITY - 1) {
            i++;
            c += particles[i].weight;
        }
        new_particles[m] = particles[i];
        new_particles[m].weight = 1.0f / PARTICLE_QUANTITY;
    }
    
    particles = new_particles;
}

// Get the estimated pose from particle distribution
lemlib::Pose getEstimatedPose() {
    lemlib::Pose estimated(0, 0, 0); // Initialize with zeros
    float total_weight = 0.0f;
    
    for (const auto& particle : particles) {
        estimated.x += particle.weight * particle.pose.x;
        estimated.y += particle.weight * particle.pose.y;
        estimated.theta += particle.weight * particle.pose.theta;
        total_weight += particle.weight;
    }
    
    if (total_weight > 0) {
        estimated.x /= total_weight;
        estimated.y /= total_weight;
        estimated.theta /= total_weight;
    }
    
    return estimated;
}

// Calculate motion delta
lemlib::Pose calculateMotionDelta(const lemlib::Pose& currentPose) {
    lemlib::Pose delta(
        currentPose.x - lastPose.x,
        currentPose.y - lastPose.y,
        currentPose.theta - lastPose.theta
    );
    lastPose = currentPose;
    return delta;
}

// Update the Monte Carlo Localization with the current chassis and sensor data.
void updateMCL(lemlib::Chassis& chassis, 
               float north_dist, float south_dist, float east_dist, float west_dist) {
    lemlib::Pose currentPose = chassis.getPose();
    lemlib::Pose delta = calculateMotionDelta(currentPose);
    
    motionUpdate(delta);
    measurementUpdate(north_dist, south_dist, east_dist, west_dist);
    resampleParticles();
    
    lemlib::Pose estimatedPose = getEstimatedPose();
    chassis.setPose(estimatedPose.x, estimatedPose.y, estimatedPose.theta);
}