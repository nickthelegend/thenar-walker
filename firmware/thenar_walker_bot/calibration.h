#pragma once
// MG996R pulse calibration — same meaning as thenar-arms firmware/thenar/calibration.h.
// Measured with docs/calibrator (references A/B per joint, safe ends), October 2026. Also stored on the robot (NVS "tw-armcal").
constexpr bool FOLLOWER_CALIBRATED = true;
constexpr float SERVO_ZERO_US[6] = {2360.0, 755.7, 1669.2, 796.2, 1920.0, 1460.0};
constexpr float US_PER_DEGREE[6] = {10.0, 8.22222, 11.77778, 14.88889, 10.55556, 10.0};
constexpr int SERVO_SIGN[6] = {-1, -1, 1, 1, -1, -1};
constexpr float PULSE_MIN_US[6] = {1250, 500, 500, 500, 500, 500};
constexpr float PULSE_MAX_US[6] = {2500, 1920, 1860, 1660, 2450, 1640};
constexpr float PCA_CLOCK_HZ = 25000000;
