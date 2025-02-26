#include "api.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/lvgl.h"
#include "main.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <iostream>

using namespace lemlib;

#include "robot/mcl.h"
#include "globals.h"
#include <cmath>
#include <algorithm>
#include <random>
#include <numeric>

// Constructor
MonteCarloLocalization::MonteCarloLocalization(
    double fieldWidth, 
    double fieldHeight, 
    int numParticles,
    std::array<double, 2> northSensorPos,
    std::array<double, 2> eastSensorPos,
    std::array<double, 2> southSensorPos,
    std::array<double, 2> westSensorPos
) : fieldWidth(fieldWidth), 
    fieldHeight(fieldHeight), 
    numParticles(numParticles),
    northSensorPos(northSensorPos),
    eastSensorPos(eastSensorPos),
    southSensorPos(southSensorPos),
    westSensorPos(westSensorPos) {
    
    // Initialize random number generator with random seed
    std::random_device rd;
    rng = std::mt19937(rd());
    
    // Set default noise parameters
    alphaRot1 = 0.1;    // Rotation noise
    alphaTrans = 0.1;   // Translation noise
    alphaRot2 = 0.1;    // Rotation noise due to translation
    
    // Set default sensor parameters
    sensorSigma = 20.0;  // 20mm standard deviation
    maxSensorDist = 2000.0; // 2 meters max reliable distance
    
    // Initialize particles (will be properly initialized with initializeParticles)
    particles.resize(numParticles);
}

// Initialize particles around a given position with some spread
void MonteCarloLocalization::initializeParticles(
    double x, double y, double theta, 
    double spreadX, double spreadY, double spreadTheta) {
    
    // Create distributions for each dimension
    std::normal_distribution<double> distX(x, spreadX);
    std::normal_distribution<double> distY(y, spreadY);
    std::normal_distribution<double> distTheta(theta, spreadTheta);
    
    // Initialize each particle
    for (int i = 0; i < numParticles; i++) {
        // Sample position and orientation
        particles[i].x = distX(rng);
        particles[i].y = distY(rng);
        particles[i].theta = distTheta(rng);
        
        // Normalize theta to [-π, π]
        particles[i].theta = std::fmod(particles[i].theta + M_PI, 2 * M_PI) - M_PI;
        
        // Initial weight is uniform
        particles[i].weight = 1.0 / numParticles;
    }
}

// Update localization with odometry and sensor readings
void MonteCarloLocalization::update(
    double deltaX, double deltaY, double deltaTheta,
    double dNorth, double dEast, double dSouth, double dWest) {
    
    // Motion update based on odometry
    motionUpdate(deltaX, deltaY, deltaTheta);
    
    // Sensor update based on distance readings
    std::array<double, 4> sensorReadings = {dNorth, dEast, dSouth, dWest};
    sensorUpdate(sensorReadings);
    
    // Resample particles based on weights
    resampleParticles();
}

// Motion model update
void MonteCarloLocalization::motionUpdate(double deltaX, double deltaY, double deltaTheta) {
    // Calculate motion in robot frame
    double deltaDistance = std::sqrt(deltaX * deltaX + deltaY * deltaY);
    
    // For each particle
    for (auto& p : particles) {
        // Add noise to motion
        std::normal_distribution<double> rotNoise1(0, alphaRot1 * std::abs(deltaTheta) + 
                                                 alphaRot2 * deltaDistance);
        std::normal_distribution<double> transNoise(0, alphaTrans * deltaDistance);
        std::normal_distribution<double> rotNoise2(0, alphaRot1 * std::abs(deltaTheta) + 
                                                 alphaRot2 * deltaDistance);
        
        // Sample noisy motion
        double noisyDeltaTheta1 = rotNoise1(rng);
        double noisyDeltaDistance = deltaDistance + transNoise(rng);
        double noisyDeltaTheta2 = rotNoise2(rng);
        
        // Apply first rotation
        p.theta += deltaTheta + noisyDeltaTheta1;
        
        // Apply translation in robot frame, then convert to global frame
        double cosTheta = std::cos(p.theta);
        double sinTheta = std::sin(p.theta);
        p.x += (deltaX + noisyDeltaDistance * cosTheta);
        p.y += (deltaY + noisyDeltaDistance * sinTheta);
        
        // Apply second rotation
        p.theta += noisyDeltaTheta2;
        
        // Normalize theta to [-π, π]
        p.theta = std::fmod(p.theta + M_PI, 2 * M_PI) - M_PI;
        
        // Enforce field boundaries (optional)
        p.x = std::max(0.0, std::min(p.x, fieldWidth));
        p.y = std::max(0.0, std::min(p.y, fieldHeight));
    }
}

// Predict sensor readings for a particle
std::array<double, 4> MonteCarloLocalization::predictSensorReadings(const Particle& p) {
    std::array<double, 4> predictions;
    
    // Calculate global positions of sensors
    double cosTheta = std::cos(p.theta);
    double sinTheta = std::sin(p.theta);
    
    // North sensor
    double northX = p.x + northSensorPos[0] * cosTheta - northSensorPos[1] * sinTheta;
    double northY = p.y + northSensorPos[0] * sinTheta + northSensorPos[1] * cosTheta;
    predictions[0] = fieldHeight - northY; // Distance to north wall
    
    // East sensor
    double eastX = p.x + eastSensorPos[0] * cosTheta - eastSensorPos[1] * sinTheta;
    double eastY = p.y + eastSensorPos[0] * sinTheta + eastSensorPos[1] * cosTheta;
    predictions[1] = fieldWidth - eastX; // Distance to east wall
    
    // South sensor
    double southX = p.x + southSensorPos[0] * cosTheta - southSensorPos[1] * sinTheta;
    double southY = p.y + southSensorPos[0] * sinTheta + southSensorPos[1] * cosTheta;
    predictions[2] = southY; // Distance to south wall
    
    // West sensor
    double westX = p.x + westSensorPos[0] * cosTheta - westSensorPos[1] * sinTheta;
    double westY = p.y + westSensorPos[0] * sinTheta + westSensorPos[1] * cosTheta;
    predictions[3] = westX; // Distance to west wall
    
    // Clip predictions to max sensor distance
    for (auto& pred : predictions) {
        pred = std::min(pred, maxSensorDist);
    }
    
    return predictions;
}

// Normal probability density function
double MonteCarloLocalization::normalPDF(double x, double mean, double sigma) {
    double diff = x - mean;
    return std::exp(-0.5 * diff * diff / (sigma * sigma)) / (sigma * std::sqrt(2 * M_PI));
}

// Sensor model update
void MonteCarloLocalization::sensorUpdate(const std::array<double, 4>& sensorReadings) {
    // For each particle
    for (auto& p : particles) {
        // Predict sensor readings for this particle
        std::array<double, 4> predictions = predictSensorReadings(p);
        
        // Calculate likelihood of each sensor reading
        double likelihood = 1.0;
        for (int i = 0; i < 4; i++) {
            // Skip if sensor reading is beyond max distance
            if (sensorReadings[i] >= maxSensorDist) continue;
            
            // Calculate probability of this reading given the prediction
            double prob = normalPDF(sensorReadings[i], predictions[i], sensorSigma);
            
            // Multiply likelihoods (assuming independence)
            likelihood *= prob;
        }
        
        // Update particle weight
        p.weight *= likelihood;
    }
    
    // Normalize weights
    double sumWeights = 0.0;
    for (const auto& p : particles) {
        sumWeights += p.weight;
    }
    
    // Avoid division by zero
    if (sumWeights > 0) {
        for (auto& p : particles) {
            p.weight /= sumWeights;
        }
    } else {
        // If all weights are zero, reset to uniform
        for (auto& p : particles) {
            p.weight = 1.0 / numParticles;
        }
    }
}

// Resample particles based on weights
void MonteCarloLocalization::resampleParticles() {
    // Create a copy of current particles
    std::vector<Particle> oldParticles = particles;
    
    // Prepare for resampling
    std::vector<double> cumulativeWeights(numParticles);
    cumulativeWeights[0] = oldParticles[0].weight;
    for (int i = 1; i < numParticles; i++) {
        cumulativeWeights[i] = cumulativeWeights[i-1] + oldParticles[i].weight;
    }
    
    // Resample with replacement
    std::uniform_real_distribution<double> dist(0, 1.0 / numParticles);
    double r = dist(rng);
    int idx = 0;
    
    for (int i = 0; i < numParticles; i++) {
        double u = r + i * (1.0 / numParticles);
        while (u > cumulativeWeights[idx] && idx < numParticles - 1) {
            idx++;
        }
        particles[i] = oldParticles[idx];
        particles[i].weight = 1.0 / numParticles; // Reset weights
    }
}

// Get estimated pose (weighted average of particles)
std::array<double, 3> MonteCarloLocalization::getEstimatedPose() {
    double sumX = 0, sumY = 0;
    double sumSinTheta = 0, sumCosTheta = 0;
    
    for (const auto& p : particles) {
        sumX += p.weight * p.x;
        sumY += p.weight * p.y;
        sumSinTheta += p.weight * std::sin(p.theta);
        sumCosTheta += p.weight * std::cos(p.theta);
    }
    
    // Calculate average theta using atan2 to handle circular mean
    double avgTheta = std::atan2(sumSinTheta, sumCosTheta);
    
    return {sumX, sumY, avgTheta};
}

// Get uncertainty (weighted variance of particles)
std::array<double, 3> MonteCarloLocalization::getUncertainty() {
    auto pose = getEstimatedPose();
    double varX = 0, varY = 0, varTheta = 0;
    
    for (const auto& p : particles) {
        double dx = p.x - pose[0];
        double dy = p.y - pose[1];
        
        // Calculate angular difference, handling wrap-around
        double dtheta = std::fmod(p.theta - pose[2] + M_PI, 2 * M_PI) - M_PI;
        
        varX += p.weight * dx * dx;
        varY += p.weight * dy * dy;
        varTheta += p.weight * dtheta * dtheta;
    }
    
    return {varX, varY, varTheta};
}

// Get all particles (for visualization)
const std::vector<Particle>& MonteCarloLocalization::getParticles() const {
    return particles;
}

