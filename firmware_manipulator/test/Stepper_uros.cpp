#include <Arduino.h>
#include "Config.h"
#include "Arm.h"

#include <ESP32Servo.h>
#include <FastAccelStepper.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>

rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

FastAccelStepperEngine engine;
FastAccelStepper *stepper_base = nullptr;
FastAccelStepper *stepper_arm = nullptr;
Servo gripper;

Manipulator robot_arm;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1) {delay(100);}}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

volatile double cmd_base = 0.0;
volatile double cmd_arm = 0.0;
volatile double cmd_gripper = PI;

void arm_cmd_callback(const void * msgin) {
    const std_msgs__msg__Float32MultiArray  * msg_cmd = (const std_msgs__msg__Float32MultiArray  *) msgin;

    cmd_base = msg_cmd->data.data[0];
    cmd_arm = msg_cmd->data.data[1];
    cmd_gripper = msg_cmd->data.data[2];

    robot_arm.drive_angle(cmd_base, cmd_arm, cmd_gripper);
}

void setup() { 
  Serial.begin(115200);

  set_microros_serial_transports(Serial);

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_manipulator_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/arm_cmd"
    ));

  // Initialize an executor that will manage the execution of all the ROS2 entities (publishers, subscribers, services, timers).
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &msg,
    &arm_cmd_callback,
    ON_NEW_DATA
  ));

  std_msgs__msg__Float32MultiArray__init(&msg);

  msg.data.data = (double*) malloc(3 * sizeof(double));
  msg.data.size = 3;
  msg.data.capacity = 3;

  pinMode(MS1_M1_PIN, OUTPUT);
  pinMode(MS2_M1_PIN, OUTPUT);
  pinMode(MS3_M1_PIN, OUTPUT);

  pinMode(MS1_M2_PIN, OUTPUT);
  pinMode(MS2_M2_PIN, OUTPUT);
  pinMode(MS3_M2_PIN, OUTPUT);

  digitalWrite(MS1_M1_PIN, HIGH);
  digitalWrite(MS2_M1_PIN, HIGH);
  digitalWrite(MS3_M1_PIN, HIGH);

  digitalWrite(MS1_M2_PIN, HIGH);
  digitalWrite(MS2_M2_PIN, HIGH);
  digitalWrite(MS3_M2_PIN, HIGH);

  engine.init();

  stepper_base = engine.stepperConnectToPin(STEP_M1_PIN);
  stepper_arm = engine.stepperConnectToPin(STEP_M2_PIN);
  
  robot_arm.init(stepper_base, stepper_arm, &gripper);
}

void loop() {
  // Execute pending tasks in the executor. This will handle all ROS2 communications.
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  
  delay(100);
}