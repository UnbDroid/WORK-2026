#include <Arduino.h>

#define STEPS_PER_REV_ARM 872    // 200 Steps per Rotation (Full Step Mode) * 4.36:1 Gearbox.
#define STEPS_PER_REV_BASE 1400  // 200 Steps per Rotation (Full Step Mode) * 7:1 Gearbox.
#define MICROSTEPPING_MULT 16

#define Dir_M1_Pin 26
#define Step_M1_Pin 25
#define MS1_M1_Pin 16
#define MS2_M1_Pin 17
#define MS3_M1_Pin 18

#define Dir_M2_Pin 32
#define Step_M2_Pin 33

#define Garra_Pin 13