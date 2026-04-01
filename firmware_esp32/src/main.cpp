#include <Arduino.h>
#include "Config.h"
#include "MotorDC.h"
#include "Robot.h"
#include "driver/ledc.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){}}}

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_publisher_t wheel_pub;
std_msgs__msg__Float32MultiArray wheel_msg;

MotorDC motor1(M1_IN1, M1_IN2, M1_PWM, M1_CANAL_PWM, M1_ENC_A, M1_ENC_B, M1_KP, M1_KI);
MotorDC motor2(M2_IN1, M2_IN2, M2_PWM, M2_CANAL_PWM, M2_ENC_A, M2_ENC_B, M2_KP, M2_KI);
MotorDC motor3(M3_IN1, M3_IN2, M3_PWM, M3_CANAL_PWM, M3_ENC_A, M3_ENC_B, M3_KP, M3_KI);
MotorDC motor4(M4_IN1, M4_IN2, M4_PWM, M4_CANAL_PWM, M4_ENC_A, M4_ENC_B, M4_KP, M4_KI);

void IRAM_ATTR isr_m1() { motor1.lerEncoder(); }
void IRAM_ATTR isr_m2() { motor2.lerEncoder(); }
void IRAM_ATTR isr_m3() { motor3.lerEncoder(); }
void IRAM_ATTR isr_m4() { motor4.lerEncoder(); }

Robot RobotWS(motor1, motor2, motor3, motor4);

volatile double cmd_vx = 0.0;
volatile double cmd_vy = 0.0;
volatile double cmd_wz = 0.0;

volatile unsigned long last_cmd_time = 0;

void cmd_vel_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * twist = (const geometry_msgs__msg__Twist *) msgin;

    cmd_vx = twist->linear.x;
    cmd_vy = twist->linear.y;
    cmd_wz = twist->angular.z;

    last_cmd_time = millis();
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    set_microros_serial_transports(Serial);

    motor1.init();
    motor2.init();
    motor3.init();
    motor4.init();

    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), isr_m1, RISING);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), isr_m2, RISING);
    attachInterrupt(digitalPinToInterrupt(M3_ENC_A), isr_m3, RISING);
    attachInterrupt(digitalPinToInterrupt(M4_ENC_A), isr_m4, RISING);

    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_mecanum_node", "", &support));

    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"
    ));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &subscriber,
        &msg,
        &cmd_vel_callback,
        ON_NEW_DATA
    ));

    RCCHECK(rclc_publisher_init_default(
        &wheel_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/wheel_speeds"
    ));

    std_msgs__msg__Float32MultiArray__init(&wheel_msg);

    wheel_msg.data.data = (float*) malloc(4 * sizeof(float));
    wheel_msg.data.size = 4;
    wheel_msg.data.capacity = 4;
}

void loop() {
    // Running Micro-ROS Executor (Non-Blocking).
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

    // Safety Timeout.
    if (millis() - last_cmd_time > 500) {
        cmd_vx = 0.0;
        cmd_vy = 0.0;
        cmd_wz = 0.0;
    }

    RobotWS.moveRobot(cmd_vx, cmd_vy, cmd_wz);

    wheel_msg.data.data[0] = motor1.getRPMAtual();
    wheel_msg.data.data[1] = motor2.getRPMAtual();
    wheel_msg.data.data[2] = motor3.getRPMAtual();
    wheel_msg.data.data[3] = motor4.getRPMAtual();

    RCCHECK(rcl_publish(&wheel_pub, &wheel_msg, NULL));

    // Delay to Stabilize Loop Timing.
    delay(10);  // ~100 Hz
}