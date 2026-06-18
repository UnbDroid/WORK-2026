#include <Arduino.h>

// Gripper Limits = Fully Open: 180 Degrees, Fully Closed: 100 Degrees. 

#define STEPS_PER_REV_ARM 872    // 200 Steps per Rotation (Full Step Mode) * 4.36:1 Gearbox.
#define STEPS_PER_REV_BASE 1400  // 200 Steps per Rotation (Full Step Mode) * 7:1 Gearbox.
#define MICROSTEPPING_MULT 16    // All Microstepping Pins Set to High.

#define DIR_M1_PIN 26
#define STEP_M1_PIN 25
#define ENABLE_M1_PIN 21

#define MS1_M1_PIN 16
#define MS2_M1_PIN 17
#define MS3_M1_PIN 18

#define DIR_M2_PIN 32
#define STEP_M2_PIN 33
#define ENABLE_M2_PIN 14

#define MS1_M2_PIN 22
#define MS2_M2_PIN 23
#define MS3_M2_PIN 19

#define GRIPPER_PIN 13