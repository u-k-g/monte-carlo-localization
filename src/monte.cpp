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
#include "globals.h"


// Constants
const int PARTICLE_QUANTITY = 500; //tune
const int WALL_DISTANCE_FROM_FIELD_EXTENT = 2; // the distance sensor senses this many inches away from the field extents

namespace {
    // Global vector of particles; we'll resize it during initialization.
    std::vector<Particle> particles;
    std::random_device rd;
    std::mt19937 gen(rd()); // Mersenne Twister random number generator

    // Track the last pose for delta calculations; initialize explicitly.
    lemlib::Pose lastPose(0, 0, 0);
    
    // Add these new variables
    pros::Task* mclTaskHandle = nullptr;
    bool mclRunning = false;
    lemlib::Chassis* chassisPtr = nullptr;
    
    // Add constants for the task
    const int MCL_DELAY = 20; // Run at 50Hz
    const float FIELD_DIMENSIONS = 142.0f; // Field dimensions in inches (corrected to 140")

    // Sensor offsets relative to the tracking center (in inches)
    const float NORTH_SENSOR_X_OFFSET = 0.0f; // Example offset, adjust as needed
    const float NORTH_SENSOR_Y_OFFSET = 0.0f; // Example offset, adjust as needed

    const float SOUTH_SENSOR_X_OFFSET = 0.0f; // Example offset, adjust as needed
    const float SOUTH_SENSOR_Y_OFFSET = 0.0f; // Example offset, adjust as needed

    const float EAST_SENSOR_X_OFFSET = 0.0f;  // Example offset, adjust as needed
    const float EAST_SENSOR_Y_OFFSET = 0.0f;  // Example offset, adjust as needed

    const float WEST_SENSOR_X_OFFSET = 0.0f; // Example offset, adjust as needed
    const float WEST_SENSOR_Y_OFFSET = 0.0f; // Example offset, adjust as needed
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
    float half_dimension = FIELD_DIMENSIONS / 2.0f; // Half of the field dimension
    float sensor_x_offset = 0.0f;
    float sensor_y_offset = 0.0f;

    switch (direction) {
        case 'N':
            sensor_x_offset = NORTH_SENSOR_X_OFFSET;
            sensor_y_offset = NORTH_SENSOR_Y_OFFSET;
            break;
        case 'S':
            sensor_x_offset = SOUTH_SENSOR_X_OFFSET;
            sensor_y_offset = SOUTH_SENSOR_Y_OFFSET;
            break;
        case 'E':
            sensor_x_offset = EAST_SENSOR_X_OFFSET;
            sensor_y_offset = EAST_SENSOR_Y_OFFSET;
            break;
        case 'W':
            sensor_x_offset = WEST_SENSOR_X_OFFSET;
            sensor_y_offset = WEST_SENSOR_Y_OFFSET;
            break;
    }

    // Rotate sensor offsets based on robot's theta
    float cos_theta = cos(particlePose.theta * M_PI / 180.0f);
    float sin_theta = sin(particlePose.theta * M_PI / 180.0f);

    //finds sensor offsets after rotating (rotation matrix)
    float rotated_sensor_x_offset = sensor_x_offset * cos_theta - sensor_y_offset * sin_theta;
    float rotated_sensor_y_offset = sensor_x_offset * sin_theta + sensor_y_offset * cos_theta;

    switch (direction) {
        case 'N':
            expected_distance = half_dimension - (particlePose.y + rotated_sensor_y_offset);
            break;
        case 'S':
            expected_distance = (particlePose.y + rotated_sensor_y_offset) + half_dimension;
            break;
        case 'E':
            expected_distance = half_dimension - (particlePose.x + rotated_sensor_x_offset);
            break;
        case 'W':
            expected_distance = (particlePose.x + rotated_sensor_x_offset) + half_dimension;
            break;
    }
    return expected_distance;
}

// Update particle weights based on sensor measurements
void measurementUpdate(float north_dist, float south_dist, float east_dist, float west_dist) {
    float total_weight = 0.0f;
    
    for (auto &particle : particles) {
        float weight_sum = 0.0f;
        int valid_readings = 0;
        
        // Only include valid readings (not -1) in the weight calculation
        if (north_dist >= 0) {
            float north_diff = std::abs(predictSensorReading(particle.pose, 'N') - north_dist);
            weight_sum += north_diff * north_diff;
            valid_readings++;
        }
        if (south_dist >= 0) {
            float south_diff = std::abs(predictSensorReading(particle.pose, 'S') - south_dist);
            weight_sum += south_diff * south_diff;
            valid_readings++;
        }
        if (east_dist >= 0) {
            float east_diff = std::abs(predictSensorReading(particle.pose, 'E') - east_dist);
            weight_sum += east_diff * east_diff;
            valid_readings++;
        }
        if (west_dist >= 0) {
            float west_diff = std::abs(predictSensorReading(particle.pose, 'W') - west_dist);
            weight_sum += west_diff * west_diff;
            valid_readings++;
        }
        
        // Only update weights if we have at least one valid reading
        if (valid_readings > 0) {
            float sigma = 2.0f;
            particle.weight = std::exp(-weight_sum / (2.0f * sigma * sigma * valid_readings));
            total_weight += particle.weight;
        } else {
            // If no valid readings, maintain current weight
            particle.weight = 1.0f / PARTICLE_QUANTITY;
            total_weight += particle.weight;
        }
    }
    
    // Normalize weights so that they sum to 1
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

// Add these new functions
void mclTask(void* param) {
    if (!chassisPtr) return;
    
    initializeParticles(chassisPtr->getPose());
    
    while (mclRunning) {
        // Get sensor readings in mm and convert to inches
        float north = dNorth.get() / 25.4;
        float south = dSouth.get() / 25.4;
        float east = dEast.get() / 25.4;
        float west = dWest.get() / 25.4;
        
        // Get both confidence (0-63) and object size (0-400) values
        int north_conf = dNorth.get_confidence();
        int south_conf = dSouth.get_confidence();
        int east_conf = dEast.get_confidence();
        int west_conf = dWest.get_confidence();
        
        int north_size = dNorth.get_object_size();
        int south_size = dSouth.get_object_size();
        int east_size = dEast.get_object_size();
        int west_size = dWest.get_object_size();
        
        // Filter readings based on both metrics
        const int MIN_CONFIDENCE = 45;  // High confidence threshold (max is 63)
        const int MIN_OBJECT_SIZE = 50; // Minimum object size (walls should be larger than a grey card)
        const int MAX_OBJECT_SIZE = 300; // Maximum object size (filter out potentially erroneous readings)
        
        // Mark readings as invalid (-1) if they don't meet our criteria
        if (north_conf < MIN_CONFIDENCE || north_size < MIN_OBJECT_SIZE || north_size > MAX_OBJECT_SIZE || north >= 9999) north = -1;
        if (south_conf < MIN_CONFIDENCE || south_size < MIN_OBJECT_SIZE || south_size > MAX_OBJECT_SIZE || south >= 9999) south = -1;
        if (east_conf < MIN_CONFIDENCE || east_size < MIN_OBJECT_SIZE || east_size > MAX_OBJECT_SIZE || east >= 9999) east = -1;
        if (west_conf < MIN_CONFIDENCE || west_size < MIN_OBJECT_SIZE || west_size > MAX_OBJECT_SIZE || west >= 9999) west = -1;
        
        updateMCL(*chassisPtr, north, south, east, west);
        pros::delay(MCL_DELAY);
    }
}

void startMCL(lemlib::Chassis& chassis) {
    if (mclTaskHandle != nullptr) {
        stopMCL(); // Stop existing task if running
    }
    
    chassisPtr = &chassis;
    mclRunning = true;
    mclTaskHandle = new pros::Task(mclTask, nullptr, "MCL Task");
}

void stopMCL() {
    if (mclTaskHandle != nullptr) {
        mclRunning = false;
        pros::delay(MCL_DELAY * 2); // Give task time to stop
        delete mclTaskHandle;
        mclTaskHandle = nullptr;
        chassisPtr = nullptr;
    }
}

// Add this test function after the other functions
void testParticleInitialization() {
    // Initialize particles at (0,0,0)
    lemlib::Pose initialPose(0, 0, 0);
    initializeParticles(initialPose);

    // Print the pose of each particle after initialization
    std::cout << "Initial Particle States:" << std::endl;
    for (size_t i = 0; i < particles.size(); ++i) {
        std::cout << "Particle " << i << ": x=" << particles[i].pose.x
                  << ", y=" << particles[i].pose.y
                  << ", theta=" << particles[i].pose.theta << std::endl;
    }
}
