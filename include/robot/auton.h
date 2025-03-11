#pragma once


// Utility functions
void intake_score(int degrees, int direction);
void intake_on(int speed);
void intake_off();

// Autonomous routines
void Auton1();  // Autonomous winpoint blue positive side / red positive side
void Auton2();  // Autonomous winpoint blue negative side / red negative side
void Auton3();  // Basic autonomous routine
void Auton5();  // Skills challenge autonomous

// Match specific routines
void match1();  // Match specific autonomous routine 1
void match2();  // Match specific autonomous routine 2