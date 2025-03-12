#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/lvgl.h"
#include "pros/rtos.hpp"
#include <iostream>
#include "robot/skills.h"
#include "robot/monte.hpp"
#include <random>
#include <cmath>
#include "globals.h"


// Constants
const int PARTICLE_QUANTITY = 5000; //tune

namespace {
    // Global vector of particles; we'll resize it during initialization.
    std::vector<Particle> particles;
    std::random_device rd;
    std::mt19937 gen(rd()); // Mersenne Twister random number generator

    // Track the last odometry pose for odometry delta calculations
    lemlib::Pose lastOdomPose(0, 0, 0);
    
    // Add these new variables
    pros::Task* mclTaskHandle = nullptr;
    bool mclRunning = false;
    lemlib::Chassis* chassisPtr = nullptr;
    
    // Add constants for the task
    const int MCL_DELAY = 20; // Run at 50Hz
    const float FIELD_DIMENSIONS = 144.0f; // Corrected to 144 inches

    // Sensor offsets relative to the tracking center (in inches)
    const float NORTH_SENSOR_X_OFFSET = 0.0f; // Example offset, adjust as needed
    const float NORTH_SENSOR_Y_OFFSET = 0.0f; // Example offset, adjust as needed

    const float SOUTH_SENSOR_X_OFFSET = 0.0f; // Example offset, adjust as needed
    const float SOUTH_SENSOR_Y_OFFSET = 0.0f; // Example offset, adjust as needed

    const float EAST_SENSOR_X_OFFSET = 0.0f;  // Example offset, adjust as needed
    const float EAST_SENSOR_Y_OFFSET = 0.0f;  // Example offset, adjust as needed

    const float WEST_SENSOR_X_OFFSET = 0.0f; // Example offset, adjust as needed
    const float WEST_SENSOR_Y_OFFSET = 0.0f; // Example offset, adjust as needed

    // Add boolean flags to control sensor usage
    bool useNorthSensor = false;
    bool useSouthSensor = true;
    bool useEastSensor = false;
    bool useWestSensor = false;

    // Add sigma values for distance-dependent noise
    const float sigma_close_range = 0.3f; // Sigma for distances below 200mm (approx 7.87 inches)
    const float sigma_far_range = 1.0f;   // Sigma for distances above 200mm
    const float DISTANCE_THRESHOLD_INCHES = 7.87f; // 200mm in inches

    // Add variables for update frequency control
    int lastUpdateTime = 0; // Timestamp of the last MCL update
    const int UPDATE_INTERVAL = 1000; // Update interval in milliseconds (1 second)
    lemlib::Pose lastUpdatedPose(0, 0, 0); // Last pose at which MCL was updated
    const float MIN_MOTION_THRESHOLD = 0.25f; // Minimum motion in inches to trigger update

    // Add constant for motion noise threshold
    const float MOTION_NOISE_THRESHOLD = 0.25f; // Threshold to consider motion as static

    const float UNIFORM_WEIGHT_FACTOR = 0.0001f; // Small uniform weight factor, tune as needed

    // Add variables to store previous sensor readings
    float prev_north_dist = -1.0f; // Initialize with an out-of-range value
    float prev_south_dist = -1.0f;
    float prev_east_dist = -1.0f;
    float prev_west_dist = -1.0f;
}

// Initialize particles around an initial pose estimate
void initializeParticles(const lemlib::Pose& initialPose) {
    particles.resize(PARTICLE_QUANTITY);
    lastOdomPose = initialPose;
    
    std::normal_distribution<float> x_dist(initialPose.x, 1.0);
    std::normal_distribution<float> y_dist(initialPose.y, 1.0);
    std::normal_distribution<float> theta_dist(initialPose.theta, 0.0);

    for (auto& particle : particles) {
        particle = Particle(
            lemlib::Pose(x_dist(gen), y_dist(gen), theta_dist(gen)),
            1.0f / PARTICLE_QUANTITY
        );
    }
}

// Update particles based on robot motion (prediction step)
void motionUpdate(const lemlib::Pose& localOdomDelta) {
    float motion_magnitude = std::sqrt(localOdomDelta.x * localOdomDelta.x + localOdomDelta.y * localOdomDelta.y);
    bool add_noise = motion_magnitude > MOTION_NOISE_THRESHOLD; // Use MOTION_NOISE_THRESHOLD for comparison

    std::normal_distribution<float> motion_noise(0.0, 0.1);
    std::normal_distribution<float> rotation_noise(0.0, 0.0);

    for (auto &particle : particles) {
        float theta_rad = particle.pose.theta * M_PI / 180.0f; // Convert to radians
        float dx_global = localOdomDelta.x * cos(theta_rad) - localOdomDelta.y * sin(theta_rad); // Transform local x to global x
        float dy_global = localOdomDelta.x * sin(theta_rad) + localOdomDelta.y * cos(theta_rad); // Transform local y to global y

        particle.pose.x += dx_global + (add_noise ? motion_noise(gen) : 0.0f); // Apply global x delta with noise
        particle.pose.y += dy_global + (add_noise ? motion_noise(gen) : 0.0f); // Apply global y delta with noise
        particle.pose.theta += localOdomDelta.theta + (add_noise ? rotation_noise(gen) : 0.0f); // Apply theta delta with noise (no transformation needed for theta)
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
    const float DISTANCE_CHANGE_THRESHOLD = 0.15f; // 0.15 inches or approx 3mm

    // Check if sensor readings have changed significantly
    if (std::abs(north_dist - prev_north_dist) <= DISTANCE_CHANGE_THRESHOLD &&
        std::abs(south_dist - prev_south_dist) <= DISTANCE_CHANGE_THRESHOLD &&
        std::abs(east_dist - prev_east_dist) <= DISTANCE_CHANGE_THRESHOLD &&
        std::abs(west_dist - prev_west_dist) <= DISTANCE_CHANGE_THRESHOLD) {
        // Sensor readings haven't changed significantly, skip update
        printf("Sensor readings unchanged, skipping measurement update.\n"); // Optional debug print
        prev_north_dist = north_dist; // Update previous readings even if skipping update
        prev_south_dist = south_dist;
        prev_east_dist = east_dist;
        prev_west_dist = west_dist;
        return; // Exit the function early, skipping the weight update
    }

    float total_weight = 0.0f;

    for (auto &particle : particles) {
        float particle_weight = 1.0f; // Initialize particle weight to 1 (for product)
        int valid_readings = 0; // Keep track of valid readings

        // --- Function to get distance-dependent sigma ---
        auto getSigma = [&](float predicted_distance) {
            return (predicted_distance < DISTANCE_THRESHOLD_INCHES) ? sigma_close_range : sigma_far_range;
        };

        // Only include valid readings (not -1) in the weight calculation
        if (north_dist >= 0) {
            float predicted_north_dist = predictSensorReading(particle.pose, 'N');
            float north_diff = std::abs(predicted_north_dist - north_dist);
            float sigma = getSigma(predicted_north_dist);
            float north_likelihood = std::exp(-(north_diff * north_diff) / (2.0f * sigma * sigma)); // Calculate likelihood
            particle_weight *= north_likelihood; // Multiply likelihood to particle weight
            valid_readings++;
        }
        if (south_dist >= 0) {
            float predicted_south_dist = predictSensorReading(particle.pose, 'S');
            float south_diff = std::abs(predicted_south_dist - south_dist);
            float sigma = getSigma(predicted_south_dist);
            float south_likelihood = std::exp(-(south_diff * south_diff) / (2.0f * sigma * sigma)); // Calculate likelihood
            particle_weight *= south_likelihood; // Multiply likelihood to particle weight
            valid_readings++;
        }
        if (east_dist >= 0) {
            float predicted_east_dist = predictSensorReading(particle.pose, 'E');
            float east_diff = std::abs(predicted_east_dist - east_dist);
            float sigma = getSigma(predicted_east_dist);
            float east_likelihood = std::exp(-(east_diff * east_diff) / (2.0f * sigma * sigma)); // Calculate likelihood
            particle_weight *= east_likelihood; // Multiply likelihood to particle weight
            valid_readings++;
        }
        if (west_dist >= 0) {
            float predicted_west_dist = predictSensorReading(particle.pose, 'W');
            float west_diff = std::abs(predicted_west_dist - west_dist);
            float sigma = getSigma(predicted_west_dist);
            float west_likelihood = std::exp(-(west_diff * west_diff) / (2.0f * sigma * sigma)); // Calculate likelihood
            particle_weight *= west_likelihood; // Multiply likelihood to particle weight
            valid_readings++;
        }

        // Apply uniform weight if few valid readings
        if (valid_readings > 0) {
            particle.weight = particle_weight; // Set particle weight to the product of likelihoods
        } else {
            particle.weight = UNIFORM_WEIGHT_FACTOR; // Assign uniform weight if no valid readings
        }
        total_weight += particle.weight;
    }

    // Normalize weights so that they sum to 1
    if (total_weight > 0) {
        for (auto &particle : particles) {
            particle.weight /= total_weight;
        }
    }

    // Update previous sensor readings for the next iteration
    prev_north_dist = north_dist;
    prev_south_dist = south_dist;
    prev_east_dist = east_dist;
    prev_west_dist = west_dist;
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

// Calculate motion delta - now calculates odometry delta
lemlib::Pose calculateMotionDelta(const lemlib::Pose& currentOdomPose) {
    lemlib::Pose delta(
        currentOdomPose.x - lastOdomPose.x,
        currentOdomPose.y - lastOdomPose.y,
        currentOdomPose.theta - lastOdomPose.theta
    );
    lastOdomPose = currentOdomPose;
    return delta;
}

// Update the Monte Carlo Localization with the current chassis and sensor data.
void updateMCL(lemlib::Chassis& chassis, float north_dist, float south_dist, float east_dist, float west_dist) {
    lemlib::Pose localOdomDelta = chassis.getPose(); // Local movement since last setPose
    motionUpdate(localOdomDelta); // Using localOdomDelta here
    measurementUpdate(north_dist, south_dist, east_dist, west_dist);
    resampleParticles();

    lemlib::Pose estimatedPose = getEstimatedPose();
    printf("Odometry: (%.2f, %.2f, %.2f), MCL: (%.2f, %.2f, %.2f)\n", // Debug print for comparison
           chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta,
           estimatedPose.x, estimatedPose.y, estimatedPose.theta);
    chassis.setPose(estimatedPose.x, estimatedPose.y, estimatedPose.theta);
}

// Add these new functions
void mclTask(void* param) { //gets the sensor readings and throws out unreliable readings
    if (!chassisPtr) return;
    
    initializeParticles(chassisPtr->getPose());
    lastUpdatedPose = chassisPtr->getPose();
    lastOdomPose = chassisPtr->getPose();
    
    while (mclRunning) {
        float north = dNorth.get() / 25.4;
        float south = dSouth.get() / 25.4;
        float east = dEast.get() / 25.4;
        float west = dWest.get() / 25.4;
        
        int north_conf = dNorth.get_confidence();
        int south_conf = dSouth.get_confidence();
        int east_conf = dEast.get_confidence();
        int west_conf = dWest.get_confidence();
        
        int north_size = dNorth.get_object_size();
        int south_size = dSouth.get_object_size();
        int east_size = dEast.get_object_size();
        int west_size = dWest.get_object_size();
        
        const int MIN_CONFIDENCE = 45;
        const int MIN_OBJECT_SIZE = 50;
        const int MAX_OBJECT_SIZE = 401;
        
        if (north_conf < MIN_CONFIDENCE || north_size < MIN_OBJECT_SIZE || north_size > MAX_OBJECT_SIZE || north >= 9999 || north > 210) north = -1;
        if (south_conf < MIN_CONFIDENCE || south_size < MIN_OBJECT_SIZE || south_size > MAX_OBJECT_SIZE || south >= 9999 || south > 210) south = -1;
        if (east_conf < MIN_CONFIDENCE || east_size < MIN_OBJECT_SIZE || east_size > MAX_OBJECT_SIZE || east >= 9999 || east > 210) east = -1;
        if (west_conf < MIN_CONFIDENCE || west_size < MIN_OBJECT_SIZE || west_size > MAX_OBJECT_SIZE || west >= 9999 || west > 210) west = -1;
        
        if (!useNorthSensor) north = -1;
        if (!useSouthSensor) south = -1;
        if (!useEastSensor) east = -1;
        if (!useWestSensor) west = -1;
        
        int currentTime = pros::millis();
        lemlib::Pose currentPose = chassisPtr->getPose();

        printf("Time: %d, Current Pose X: %.2f, Y: %.2f, Update Interval: %d, Last Update: %d\n",
               currentTime, currentPose.x, currentPose.y, UPDATE_INTERVAL, lastUpdateTime);

        if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
            printf("MCL Update Triggered!\n");
            lemlib::Pose poseBeforeUpdate = chassisPtr->getPose();
            updateMCL(*chassisPtr, north, south, east, west);
            lemlib::Pose poseAfterUpdate = chassisPtr->getPose();

            float deltaX_mcl = poseAfterUpdate.x - poseBeforeUpdate.x;
            float deltaY_mcl = poseAfterUpdate.y - poseBeforeUpdate.y;

            lastUpdateTime = currentTime;
            lastUpdatedPose = chassisPtr->getPose();
        } else {
            float deltaX_mcl = 0.0f;
            float deltaY_mcl = 0.0f;
        }

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