#include "robot/monte.hpp"
#include "globals.h"
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/lvgl.h"
#include "main.h"
#include "pros/rtos.hpp"
#include "robot/skills.h"
#include <cmath>
#include <iostream>
#include <random>

// Constants
const int PARTICLE_QUANTITY = 5000; // tune

namespace {
// Global vector of particles; we'll resize it during initialization.
std::vector<Particle> particles;
std::random_device rd;
std::mt19937 gen(rd()); // Mersenne Twister random number generator
std::normal_distribution<float> noise_x(0.0f,
                                        0.1f); // Mean 0, std dev 0.1 inches
std::normal_distribution<float> noise_y(0.0f,
                                        0.1f); // Mean 0, std dev 0.1 inches
std::normal_distribution<float> noise_theta(0.0f, 5.0f); // Std dev 5.0 degrees

// Track the last odometry pose for odometry delta calculations
lemlib::Pose lastOdomPose(0, 0, 0);

// Add these new variables
pros::Task *mclTaskHandle = nullptr;
bool mclRunning = false;
lemlib::Chassis *chassisPtr = nullptr;

// Add constants for the task
const int MCL_DELAY = 20;              // Run at 50Hz
const float FIELD_DIMENSIONS = 144.0f; // Corrected to 144 inches

// Sensor offsets relative to the tracking center (in inches)
const float NORTH_SENSOR_X_OFFSET = 0.0f; // Example offset, adjust as needed
const float NORTH_SENSOR_Y_OFFSET = 0.0f; // Example offset, adjust as needed

const float SOUTH_SENSOR_X_OFFSET = 0.0f; // Example offset, adjust as needed
const float SOUTH_SENSOR_Y_OFFSET = 0.0f; // Example offset, adjust as needed

const float EAST_SENSOR_X_OFFSET = 0.0f; // Example offset, adjust as needed
const float EAST_SENSOR_Y_OFFSET = 0.0f; // Example offset, adjust as needed

const float WEST_SENSOR_X_OFFSET = 0.0f; // Example offset, adjust as needed
const float WEST_SENSOR_Y_OFFSET = 0.0f; // Example offset, adjust as needed

// Add boolean flags to control sensor usage
bool useNorthSensor = false;
bool useSouthSensor = false;
bool useEastSensor = false;
bool useWestSensor = true;

// Add sigma values for distance-dependent noise
const float sigma_close_range =
    0.3f; // Sigma for distances below 200mm (approx 7.87 inches)
const float sigma_far_range = 1.0f; // Sigma for distances above 200mm
const float DISTANCE_THRESHOLD_INCHES = 7.87f; // 200mm in inches

// Add variables for update frequency control
int lastUpdateTime = 0;           // Timestamp of the last MCL update
const int UPDATE_INTERVAL = 1000; // Update interval in milliseconds (1 second)
lemlib::Pose lastUpdatedPose(0, 0, 0); // Last pose at which MCL was updated
const float MIN_MOTION_THRESHOLD =
    0.25f; // Minimum motion in inches to trigger update

// Add constant for motion noise threshold
const float MOTION_NOISE_THRESHOLD =
    0.25f; // Threshold to consider motion as static

const float UNIFORM_WEIGHT_FACTOR =
    0.0001f;                   // Small uniform weight factor, tune as needed
const float MIN_WEIGHT = 0.1f; // Minimum weight floor for particles
const int RESAMPLING_INTERVAL = 3; // Only resample every 3rd update

// Add variables to store previous sensor readings
float prev_north_dist = -1.0f; // Initialize with an out-of-range value
float prev_south_dist = -1.0f;
float prev_east_dist = -1.0f;
float prev_west_dist = -1.0f;
const float DISTANCE_CHANGE_THRESHOLD = 0.25f; // Adjust based on sensor noise

// Add this at the global scope
lemlib::Pose filteredPose(0, 0, 0); // {{ Added filteredPose }}
const float FILTER_ALPHA =
    0.3f; // Smoothing factor (0.3 = 30% new, 70% old) {{ Added FILTER_ALPHA }}
const float ODOMETRY_TRUST_FACTOR =
    0.5f; // 0.5 = equal trust in odometry and sensors {{ Added
          // ODOMETRY_TRUST_FACTOR }}
} // namespace

// Initialize particles around an initial pose estimate
void initializeParticles(const lemlib::Pose &initialPose) {
  particles.resize(PARTICLE_QUANTITY);
  lastOdomPose = initialPose;

  std::normal_distribution<float> x_dist(initialPose.x, 1.0);
  std::normal_distribution<float> y_dist(initialPose.y, 1.0);
  std::normal_distribution<float> theta_dist(initialPose.theta, 0.0);

  for (auto &particle : particles) {
    particle = Particle(lemlib::Pose(x_dist(gen), y_dist(gen), theta_dist(gen)),
                        1.0f / PARTICLE_QUANTITY);
  }
}

// Update particles based on robot motion (prediction step)
void motionUpdate(const lemlib::Pose &localOdomDelta) {
  float motion_magnitude = std::sqrt(localOdomDelta.x * localOdomDelta.x +
                                     localOdomDelta.y * localOdomDelta.y);

  // Only add noise if motion is significant
  const float MIN_MOTION_FOR_NOISE =
      0.5f; // Increased threshold for adding noise {{ Added
            // MIN_MOTION_FOR_NOISE and increased threshold }}
  bool add_noise =
      motion_magnitude > MIN_MOTION_FOR_NOISE; // Use MIN_MOTION_FOR_NOISE

  // Reduced noise values
  std::normal_distribution<float> motion_noise(
      0.0, 0.03); // Reduced from 0.1 {{ Reduced motion_noise std dev }}
  std::normal_distribution<float> rotation_noise(
      0.0, 0.05); // Small rotation noise {{ Added rotation_noise std dev }}

  for (auto &particle : particles) {
    float theta_rad = particle.pose.theta * M_PI / 180.0f;
    float dx_global =
        localOdomDelta.x * cos(theta_rad) - localOdomDelta.y * sin(theta_rad);
    float dy_global =
        localOdomDelta.x * sin(theta_rad) + localOdomDelta.y * cos(theta_rad);

    // Scale noise based on motion magnitude
    float noise_scale =
        add_noise
            ? std::min(1.0f, motion_magnitude / 2.0f)
            : 0.0f; // Scale noise {{ Scaled noise based on motion magnitude }}

    particle.pose.x +=
        dx_global +
        noise_scale * motion_noise(gen); // Apply scaled motion noise
    particle.pose.y +=
        dy_global +
        noise_scale * motion_noise(gen); // Apply scaled motion noise
    particle.pose.theta +=
        localOdomDelta.theta +
        noise_scale * rotation_noise(gen); // Apply scaled rotation noise

    // Normalize theta to be within 0-360 degrees
    particle.pose.theta = fmod(
        particle.pose.theta, 360.0f); // Normalize theta {{ Normalized theta }}
    if (particle.pose.theta <
        0) { // Ensure theta is positive {{ Ensure theta is positive }}
      particle.pose.theta +=
          360.0f; // Add 360 if negative {{ Add 360 if negative }}
    }
  }
}

// Calculate expected sensor readings for a particle
float predictSensorReading(const lemlib::Pose &particlePose,
                           const char direction) {
  float expected_distance = 0.0f; // Declare expected_distance here
  float half_dimension = FIELD_DIMENSIONS / 2.0f; // 72 inches
  float sensor_x = particlePose.x;
  float sensor_y = particlePose.y;

  // For simplicity, assume offset = 0 since you confirmed it’s not the issue
  switch (direction) {
  case 'N':
    expected_distance = half_dimension - sensor_y; // Distance to y = 72
    break;
  case 'S':
    expected_distance = sensor_y + half_dimension; // Distance to y = -72
    break;
  case 'E':
    expected_distance = half_dimension - sensor_x; // Distance to x = 72
    break;
  case 'W':
    expected_distance = sensor_x + half_dimension; // Distance to x = -72
    break;
  default:
    return -1.0f; // Invalid direction
  }

  // Clamp to positive values (sensors don’t measure negative distances)
  if (expected_distance < 0)
    expected_distance = 0;

  return expected_distance;
}

// Update particle weights based on sensor measurements
void measurementUpdate(float north_dist, float south_dist, float east_dist,
                       float west_dist) {
  // Store previous readings (existing code)

  // Increase this threshold to reduce sensitivity to small changes
  const float DISTANCE_CHANGE_THRESHOLD =
      0.25f; // Increased from 0.15 to 0.25 inches

  // Skip update if readings haven't changed significantly
  bool significant_change =
      false; // {{ Initialize significant_change to false }}
  if (north_dist >= 0 && prev_north_dist >= 0 &&
      std::abs(north_dist - prev_north_dist) > DISTANCE_CHANGE_THRESHOLD)
    significant_change = true; // {{ Check north sensor change }}
  if (south_dist >= 0 && prev_south_dist >= 0 &&
      std::abs(south_dist - prev_south_dist) > DISTANCE_CHANGE_THRESHOLD)
    significant_change = true; // {{ Check south sensor change }}
  if (east_dist >= 0 && prev_east_dist >= 0 &&
      std::abs(east_dist - prev_east_dist) > DISTANCE_CHANGE_THRESHOLD)
    significant_change = true; // {{ Check east sensor change }}
  if (west_dist >= 0 && prev_west_dist >= 0 &&
      std::abs(west_dist - prev_west_dist) > DISTANCE_CHANGE_THRESHOLD)
    significant_change = true; // {{ Check west sensor change }}

  if (!significant_change) { // {{ If no significant change, skip update }}
    return;                  // Skip measurement update if no significant change
  }

  float total_weight = 0.0f;

  for (auto &particle : particles) {
    float particle_weight = 1.0f;
    int valid_readings = 0;

    // Modify sigma values to be less sensitive
    auto getSigma = [&](float predicted_distance) {
      // Increase sigma values to be more tolerant of measurement noise
      return (predicted_distance < 20.0f)
                 ? 1.0f
                 : 2.0f; // Example values in inches            // Increased
                         // from 0.3/1.0 to 0.5/1.5
    };

    // Process sensor readings (existing code with updated sigmas)
    if (north_dist >= 0) {
      float predicted_north_dist = predictSensorReading(particle.pose, 'N');
      float north_diff = std::abs(predicted_north_dist - north_dist);
      float sigma = getSigma(predicted_north_dist);
      float north_likelihood =
          std::exp(-(north_diff * north_diff) /
                   (2.0f * sigma * sigma)); // Calculate likelihood
      particle_weight *=
          north_likelihood; // Multiply likelihood to particle weight
      valid_readings++;
    }
    if (south_dist >= 0) {
      float predicted_south_dist = predictSensorReading(particle.pose, 'S');
      float south_diff = std::abs(predicted_south_dist - south_dist);
      float sigma = getSigma(predicted_south_dist);
      float south_likelihood =
          std::exp(-(south_diff * south_diff) /
                   (2.0f * sigma * sigma)); // Calculate likelihood
      particle_weight *=
          south_likelihood; // Multiply likelihood to particle weight
      valid_readings++;
    }
    if (east_dist >= 0) {
      float predicted_east_dist = predictSensorReading(particle.pose, 'E');
      float east_diff = std::abs(predicted_east_dist - east_dist);
      float sigma = getSigma(predicted_east_dist);
      float east_likelihood =
          std::exp(-(east_diff * east_diff) /
                   (2.0f * sigma * sigma)); // Calculate likelihood
      particle_weight *=
          east_likelihood; // Multiply likelihood to particle weight
      valid_readings++;
    }
    if (west_dist >= 0) {
      float predicted_west_dist = predictSensorReading(particle.pose, 'W');
      float west_diff = std::abs(predicted_west_dist - west_dist);
      float sigma = getSigma(predicted_west_dist);
      float west_likelihood =
          std::exp(-(west_diff * west_diff) /
                   (2.0f * sigma * sigma)); // Calculate likelihood
      particle_weight *=
          west_likelihood; // Multiply likelihood to particle weight
      valid_readings++;
    }

    // Apply a minimum weight floor to prevent particles from getting too low
    // weight
    if (valid_readings > 0) {
      // Apply minimum weight floor
      particle.weight = std::max(
          particle_weight,
          MIN_WEIGHT *
              UNIFORM_WEIGHT_FACTOR); // {{ Applied minimum weight floor }}
    } else {
      particle.weight = UNIFORM_WEIGHT_FACTOR;
    }

    total_weight += particle.weight;
  }

  // Normalize weights (existing code)
  if (total_weight > 0) {
    for (auto &particle : particles) {
      particle.weight /= total_weight;
    }
  }

  // Update previous sensor readings for the next iteration
  prev_north_dist =
      north_dist; // Update prev_north_dist {{ Update prev_north_dist }}
  prev_south_dist =
      south_dist; // Update prev_south_dist {{ Update prev_south_dist }}
  prev_east_dist =
      east_dist; // Update prev_east_dist {{ Update prev_east_dist }}
  prev_west_dist =
      west_dist; // Update prev_west_dist {{ Update prev_west_dist }}
}

// Perform systematic resampling based on particle weights
std::vector<Particle> weightedResample(const std::vector<Particle> &particles) {
  std::vector<Particle> new_particles(PARTICLE_QUANTITY);
  std::uniform_real_distribution<float> dist(0.0f, 1.0f);

  // Compute cumulative weights
  std::vector<float> cumulative_weights(PARTICLE_QUANTITY);
  cumulative_weights[0] = particles[0].weight;
  for (size_t i = 1; i < PARTICLE_QUANTITY; ++i) {
    cumulative_weights[i] = cumulative_weights[i - 1] + particles[i].weight;
  }

  // Systematic resampling
  float step = 1.0f / PARTICLE_QUANTITY;
  float r = dist(gen) * step; // Initial random offset
  size_t index = 0;

  for (size_t m = 0; m < PARTICLE_QUANTITY; ++m) {
    float U = r + m * step;
    while (index < PARTICLE_QUANTITY - 1 && U > cumulative_weights[index]) {
      ++index;
    }
    new_particles[m] = particles[index];
    new_particles[m].weight =
        1.0f / PARTICLE_QUANTITY; // Reset weights uniformly
  }

  return new_particles;
}

// Resample particles based on their weights using systematic resampling.
void resampleParticles() {
  // Perform systematic resampling
  std::vector<Particle> new_particles = weightedResample(particles);

  // Apply noise to resampled particles to prevent sample impoverishment
  std::normal_distribution<float> noise_x(0.0f, 0.1f); // Small noise in inches
  std::normal_distribution<float> noise_y(0.0f, 0.1f);
  std::normal_distribution<float> noise_theta(0.0f,
                                              0.05f); // Small noise in degrees

  for (auto &particle : new_particles) {
    particle.pose.x += noise_x(gen);
    particle.pose.y += noise_y(gen);
    particle.pose.theta += noise_theta(gen);

    // Normalize theta to stay within 0-360 degrees
    particle.pose.theta = fmod(particle.pose.theta, 360.0f);
    if (particle.pose.theta < 0) {
      particle.pose.theta += 360.0f;
    }
  }
  particles = new_particles;
}

// Modified getEstimatedPose function with filtering
lemlib::Pose getEstimatedPose() {
  lemlib::Pose rawEstimated(0, 0, 0);
  float total_weight = 0.0f;

  for (const auto &particle : particles) {
    rawEstimated.x += particle.weight * particle.pose.x;
    rawEstimated.y += particle.weight * particle.pose.y;
    rawEstimated.theta += particle.weight * particle.pose.theta;
    total_weight += particle.weight;
  }

  if (total_weight > 0) {
    rawEstimated.x /= total_weight;
    rawEstimated.y /= total_weight;
    rawEstimated.theta /= total_weight;
  }

  // Apply low-pass filter
  filteredPose.x =
      FILTER_ALPHA * rawEstimated.x +
      (1 - FILTER_ALPHA) *
          filteredPose.x; // Apply filter to x {{ Applied filter to x }}
  filteredPose.y =
      FILTER_ALPHA * rawEstimated.y +
      (1 - FILTER_ALPHA) *
          filteredPose.y; // Apply filter to y {{ Applied filter to y }}

  // Filter theta with angle wrapping
  float theta_diff = rawEstimated.theta - filteredPose.theta;
  if (theta_diff > 180)
    theta_diff -= 360;
  if (theta_diff < -180)
    theta_diff += 360;
  filteredPose.theta +=
      FILTER_ALPHA *
      theta_diff; // Apply filter to theta {{ Applied filter to theta }}

  // Normalize theta
  while (filteredPose.theta > 360)
    filteredPose.theta -= 360;
  while (filteredPose.theta < 0)
    filteredPose.theta += 360;

  // Apply odometry trust factor
  lemlib::Pose blendedPose(0, 0, 0); // {{ Added blendedPose variable }}
  blendedPose.x = ODOMETRY_TRUST_FACTOR * lastOdomPose.x +
                  (1 - ODOMETRY_TRUST_FACTOR) *
                      filteredPose.x; // {{ Blended x with odometry }}
  blendedPose.y = ODOMETRY_TRUST_FACTOR * lastOdomPose.y +
                  (1 - ODOMETRY_TRUST_FACTOR) *
                      filteredPose.y; // {{ Blended y with odometry }}

  // Handle theta blending with angle wrapping
  theta_diff = filteredPose.theta -
               lastOdomPose.theta; // {{ Recalculate theta_diff for blending }}
  if (theta_diff > 180)
    theta_diff -= 360;
  if (theta_diff < -180)
    theta_diff += 360;
  blendedPose.theta =
      lastOdomPose.theta + (1 - ODOMETRY_TRUST_FACTOR) *
                               theta_diff; // {{ Blended theta with odometry }}

  return blendedPose; // {{ Returning blendedPose }}
}

// Calculate motion delta - now calculates odometry delta
lemlib::Pose calculateMotionDelta(const lemlib::Pose &currentOdomPose) {
  // Calculate the change in position since the last update
  lemlib::Pose delta(currentOdomPose.x - lastOdomPose.x,
                     currentOdomPose.y - lastOdomPose.y,
                     currentOdomPose.theta - lastOdomPose.theta);

  // Update the last odometry position for next time
  lastOdomPose = currentOdomPose;

  // Add debug output to verify deltas are small when robot is static
  printf("Motion Delta: (%.4f, %.4f, %.4f)\n", delta.x, delta.y, delta.theta);

  return delta;
}

// Update the Monte Carlo Localization with the current chassis and sensor data.
void updateMCL(lemlib::Chassis &chassis, float north_dist, float south_dist,
               float east_dist, float west_dist) {
  lemlib::Pose currentOdomPose = chassis.getPose();
  lemlib::Pose motionDelta = calculateMotionDelta(currentOdomPose);

  // Only update particles if there's significant motion
  float motion_magnitude =
      std::sqrt(motionDelta.x * motionDelta.x + motionDelta.y * motionDelta.y);
  if (motion_magnitude > MOTION_NOISE_THRESHOLD) {
    motionUpdate(motionDelta);
  }

  measurementUpdate(north_dist, south_dist, east_dist, west_dist);
  resampleParticles();

  lemlib::Pose estimatedPose = getEstimatedPose();
  chassis.setPose(estimatedPose.x, estimatedPose.y, estimatedPose.theta);
}

// Add these new functions
void mclTask(void *param) {
  if (!chassisPtr)
    return;

  // Initialize with current pose
  initializeParticles(chassisPtr->getPose());
  lastOdomPose =
      chassisPtr->getPose(); // Record initial pose for delta calculation
  lastUpdateTime = pros::millis();

  int resampleCounter = 0; // Only resample every 3rd update

  while (mclRunning) {
    // Get distance readings and filter unreliable values
    float north = useNorthSensor ? dNorth.get() / 25.4 : -1;
    float south = useSouthSensor ? dSouth.get() / 25.4 : -1;
    float east = useEastSensor ? dEast.get() / 25.4 : -1;
    float west = useWestSensor ? dWest.get() / 25.4 : -1;

    // Apply confidence and size filters
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

    if (north_conf < MIN_CONFIDENCE || north_size < MIN_OBJECT_SIZE ||
        north_size > MAX_OBJECT_SIZE || north >= 9999 || north > 210)
      north = -1;
    if (south_conf < MIN_CONFIDENCE || south_size < MIN_OBJECT_SIZE ||
        south_size > MAX_OBJECT_SIZE || south >= 9999 || south > 210)
      south = -1;
    if (east_conf < MIN_CONFIDENCE || east_size < MIN_OBJECT_SIZE ||
        east_size > MAX_OBJECT_SIZE || east >= 9999 || east > 210)
      east = -1;
    if (west_conf < MIN_CONFIDENCE || west_size < MIN_OBJECT_SIZE ||
        west_size > MAX_OBJECT_SIZE || west >= 9999 || west > 210)
      west = -1;

    if (!useNorthSensor)
      north = -1;
    if (!useSouthSensor)
      south = -1;
    if (!useEastSensor)
      east = -1;
    if (!useWestSensor)
      west = -1;

    int currentTime = pros::millis();

    // Only update MCL periodically or when significant motion has occurred
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
      // Get current odometry pose and calculate delta
      lemlib::Pose currentOdomPose = chassisPtr->getPose();
      lemlib::Pose motionDelta = calculateMotionDelta(currentOdomPose);

      // Motion update
      if (std::sqrt(motionDelta.x * motionDelta.x +
                    motionDelta.y * motionDelta.y) > MOTION_NOISE_THRESHOLD) {
        motionUpdate(motionDelta);
      }

      // Measurement update
      measurementUpdate(north, south, east, west);

      // Only resample periodically to prevent particle depletion
      resampleCounter++;
      if (resampleCounter >= RESAMPLING_INTERVAL) {
        resampleParticles();
        resampleCounter = 0;
      }

      // Get estimated pose
      lemlib::Pose estimatedPose = getEstimatedPose();

      // Add debugging
      printf("Estimated: (%.2f, %.2f, %.2f) | Motion: (%.4f, %.4f, %.4f)\n",
             estimatedPose.x, estimatedPose.y, estimatedPose.theta,
             motionDelta.x, motionDelta.y, motionDelta.theta);

      // Update chassis position
      chassisPtr->setPose(estimatedPose.x, estimatedPose.y,
                          estimatedPose.theta);

      lastUpdateTime = currentTime;
    }

    pros::delay(MCL_DELAY);
  }
}

void startMCL(lemlib::Chassis &chassis) {
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