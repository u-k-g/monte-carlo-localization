#pragma once

#include "lemlib/api.hpp"
#include "pros/rtos.hpp"

/**
 * Executes the skills autonomous routine
 * This function handles the entire skills challenge including:
 * - Mobile goal manipulation
 * - Ring scoring
 * - Path following
 * - MCL-based localization
 */
void skills();