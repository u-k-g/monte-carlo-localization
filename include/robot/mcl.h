#ifndef MCL_H
#define MCL_H

#include <vector>
#include <array>
#include <random>
#include "api.h"
#include "pros/distance.hpp"

/**
 * @brief Particle for Monte Carlo Localization
 * 
 * Represents a possible robot position and orientation
 */
struct Particle {
    double x;        // x position in mm
    double y;        // y position in mm
    double theta;    // orientation in radians
    double weight;   // particle weight (likelihood)
};

/**
 * @brief Monte Carlo Localization class
 * 
 * Uses distance sensors to estimate robot position on the field
 */
class MonteCarloLocalization {
private:
    // Field dimensions in mm
    double fieldWidth;
    double fieldHeight;
    
    // Particle set
    std::vector<Particle> particles;
    
    // Number of particles to use
    int numParticles;
    
    // Sensor positions relative to robot center (mm)
    std::array<double, 2> northSensorPos; // [x, y]
    std::array<double, 2> eastSensorPos;
    std::array<double, 2> southSensorPos;
    std::array<double, 2> westSensorPos;
    
    // Motion model noise parameters
    double alphaRot1;    // Rotation noise factor for rotation
    double alphaTrans;   // Translation noise factor
    double alphaRot2;    // Rotation noise factor for translation
    
    // Sensor model parameters
    double sensorSigma;  // Standard deviation of sensor noise (mm)
    double maxSensorDist; // Maximum reliable sensor distance (mm)
    
    // Random number generator
    std::mt19937 rng;
    
    // Map representation (simplified as rectangular field boundaries)
    // In a more complex implementation, this would be a 2D occupancy grid
    
    // Private methods
    void motionUpdate(double deltaX, double deltaY, double deltaTheta);
    void sensorUpdate(const std::array<double, 4>& sensorReadings);
    void resampleParticles();
    double normalPDF(double x, double mean, double sigma);
    std::array<double, 4> predictSensorReadings(const Particle& p);
    
public:
    /**
     * @brief Construct a new Monte Carlo Localization object
     * 
     * @param fieldWidth Width of the field in mm
     * @param fieldHeight Height of the field in mm
     * @param numParticles Number of particles to use
     * @param northSensorPos Position of north sensor relative to robot center [x, y] in mm
     * @param eastSensorPos Position of east sensor relative to robot center [x, y] in mm
     * @param southSensorPos Position of south sensor relative to robot center [x, y] in mm
     * @param westSensorPos Position of west sensor relative to robot center [x, y] in mm
     */
    MonteCarloLocalization(
        double fieldWidth, 
        double fieldHeight, 
        int numParticles,
        std::array<double, 2> northSensorPos,
        std::array<double, 2> eastSensorPos,
        std::array<double, 2> southSensorPos,
        std::array<double, 2> westSensorPos
    );
    
    /**
     * @brief Initialize particles with uniform distribution
     * 
     * @param x Initial x position (mm)
     * @param y Initial y position (mm)
     * @param theta Initial orientation (radians)
     * @param spreadX Spread in x direction (mm)
     * @param spreadY Spread in y direction (mm)
     * @param spreadTheta Spread in orientation (radians)
     */
    void initializeParticles(double x, double y, double theta, 
                            double spreadX, double spreadY, double spreadTheta);
    
    /**
     * @brief Update localization based on odometry and sensor readings
     * 
     * @param deltaX Change in x position since last update (mm)
     * @param deltaY Change in y position since last update (mm)
     * @param deltaTheta Change in orientation since last update (radians)
     * @param dNorth North sensor reading (mm)
     * @param dEast East sensor reading (mm)
     * @param dSouth South sensor reading (mm)
     * @param dWest West sensor reading (mm)
     */
    void update(double deltaX, double deltaY, double deltaTheta,
               double dNorth, double dEast, double dSouth, double dWest);
    
    /**
     * @brief Get the estimated position and orientation
     * 
     * @return std::array<double, 3> [x, y, theta]
     */
    std::array<double, 3> getEstimatedPose();
    
    /**
     * @brief Get the uncertainty of the estimate
     * 
     * @return std::array<double, 3> [x_variance, y_variance, theta_variance]
     */
    std::array<double, 3> getUncertainty();
    
    /**
     * @brief Get all particles for visualization
     * 
     * @return const std::vector<Particle>& Reference to particles
     */
    const std::vector<Particle>& getParticles() const;
};

#endif // MCL_H
