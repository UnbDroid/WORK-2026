#include <Arduino.h>
#include "Config.h"
#include "Arm.h"

#include <ESP32Servo.h>
#include <FastAccelStepper.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/point_stamped.h> 

rcl_subscription_t subscriber;
geometry_msgs__msg__PointStamped point_msg; 

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// declara as variaveis de controle do braco
FastAccelStepperEngine engine;
FastAccelStepper *stepper_base = nullptr;
FastAccelStepper *stepper_arm = nullptr;
Servo gripper;

// cria o objeto manipulador
Manipulator robot_arm;

// define macros para checagem de erros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1) {delay(100);}}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void arm_cmd_callback(const void * msgin) {
    const geometry_msgs__msg__PointStamped  * msg_cmd = (const geometry_msgs__msg__PointStamped  *) msgin; // cast da mensagem recebida 

    // pega as coordenadas da mensagem
    double x = msg_cmd->point.x;
    double y = msg_cmd->point.y;
    double z = msg_cmd->point.z;

    // move o manipulador para as coordenadas recebidas
    robot_arm.drive_position(x, y, z);
}

void setup() { 
  // configuracoes do micro-ROS
  Serial.begin(115200);

  set_microros_serial_transports(Serial);

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "arm_controller_node", "", &support)); // arm_controler_node e o topico em que a esp vai receber as coordenadas 

  geometry_msgs__msg__PointStamped__init(&point_msg); // inicializa a mensagem que vai receber as coordenadas

  // aloca memoria para o frame_id (height_link) da mensagem, que e uma string, o frame identifica o frame de referencia do ponto, nesse caso o frame do cubo
  point_msg.header.frame_id.capacity = 30; 
  point_msg.header.frame_id.data = (char*) malloc(30 * sizeof(char));
  point_msg.header.frame_id.size = 0;

  // inicializa o subscriber que vai receber as coordenadas do braco em relacao ao cubo
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PointStamped),
    "cube_arm_coordinates"
  ));

  // Initialize an executor that will manage the execution of all the ROS2 entities (publishers, subscribers, services, timers).
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &point_msg,
    &arm_cmd_callback,
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
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  
  delay(100);
}