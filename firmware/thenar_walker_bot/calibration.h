#pragma once
// MG996R pulse calibration — same meaning as thenar-arms firmware/thenar/calibration.h.
// Keep FOLLOWER_CALIBRATED=false until each servo's zero and direction are measured unloaded;
// while false the robot drives but the arm PWM stays off.
constexpr bool FOLLOWER_CALIBRATED = false;
constexpr float SERVO_ZERO_US[6] = {1500, 1500, 1500, 1500, 1500, 1500};
constexpr float US_PER_DEGREE[6] = {5.555556, 5.555556, 5.555556, 5.555556, 5.555556, 5.555556};
constexpr int SERVO_SIGN[6] = {1, 1, 1, 1, 1, 1};
constexpr float PULSE_MIN_US[6] = {1000, 1000, 1000, 1000, 1000, 1000};
constexpr float PULSE_MAX_US[6] = {2000, 2000, 2000, 2000, 2000, 2000};
constexpr float PCA_CLOCK_HZ = 25000000;
