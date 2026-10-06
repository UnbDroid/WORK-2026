#include <Arduino.h>
#include "Config.h"
#include "Arm.h"

#include <ESP32Servo.h>
#include <FastAccelStepper.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/point.h>
#include <std_msgs/msg/bool.h> 

rcl_subscription_t point_subscriber;
geometry_msgs__msg__Point point_msg;

rcl_subscription_t gripper_subscriber;
std_msgs__msg__Bool gripper_state_msg; 

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

FastAccelStepperEngine engine;
FastAccelStepper *stepper_base = nullptr;
FastAccelStepper *stepper_arm = nullptr;
Servo gripper;

Manipulator robot_arm;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1) {delay(100);}}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void arm_cmd_callback(const void *msgin) {
    const geometry_msgs__msg__Point *msg_cmd = (const geometry_msgs__msg__Point *) msgin; 

    double x = msg_cmd->x;
    double y = msg_cmd->y;
    double z = msg_cmd->z;

    robot_arm.drive_position(x, y, z);
}

void gripper_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg_state = (const std_msgs__msg__Bool *) msgin; 

    if (msg_state->data) {
        robot_arm.drive_gripper(GRIPPER_OPEN_ANGLE);
    } else {
        robot_arm.drive_gripper(GRIPPER_CLOSE_ANGLE);
    }
}

void setup() { 
  Serial.begin(115200);

  set_microros_serial_transports(Serial);

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "arm_controller_node", "", &support));

  geometry_msgs__msg__Point__init(&point_msg);
  std_msgs__msg__Bool__init(&gripper_state_msg);

  RCCHECK(rclc_subscription_init_default(
    &point_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point),
    "arm_coordinates"
  ));

  RCCHECK(rclc_subscription_init_default(
    &gripper_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "gripper_command"
  ));

  // Initialize an executor that will manage the execution of all the ROS2 entities (publishers, subscribers, services, timers).
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &point_subscriber,
    &point_msg,
    &arm_cmd_callback,
    ON_NEW_DATA
  ));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &gripper_subscriber,
    &gripper_state_msg,
    &gripper_callback,
    ON_NEW_DATA
  ));

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
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}