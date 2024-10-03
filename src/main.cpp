#include <Arduino.h>
#include <SPI.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float64.h>

// for motor control
#include "ESP32Encoder.h"
#include "PID_v1.h"
#include "Cdrv8833.h"

// tft stuff
#include "TFT_eSPI.h"
#include "pin_config.h"

// pins
#define BL_PIN 15
#define MOTOR_A 10
#define MOTOR_B 11
#define ENCODER_A 1
#define ENCODER_B 2
#define MOTOR_PPR 1428.2

// wifi variables
IPAddress agent_ip(AGENT_IP);
size_t agent_port = AGENT_PORT;
char ssid[] = WIFI_SSID;
char psk[] = WIFI_PASSWORD;

// ros data structures
rcl_subscription_t angle_subscriber;
std_msgs__msg__Float64 angle_setpoint_msg;
rclc_executor_t sub_executor;

rcl_publisher_t publisher;
std_msgs__msg__Float64 msg;
rclc_executor_t pub_executor;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_timer_t pid_loop_timer;

// motor stuff
Cdrv8833 motor;
ESP32Encoder encoder;

// Pid variables
double setpoint, input, output;
double Kp = 25;
double Ki = 50.0;
double Kd = 0;

PID motor_pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
// -----------------

// tft stuff
TFT_eSPI tft = TFT_eSPI();

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // publish actual velocity
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data = input;
  }

  // show in tft
  tft.setCursor(0, 0);
  tft.print("Setpoint: \t");
  tft.println(setpoint);
  tft.print("Input: \t");
  tft.println(input);
  tft.print("Output: \t");
  tft.println(output);
}

void pid_loop_callback(rcl_timer_t * timer, int64_t last_call_time) {
  // compute input in radians
  int64_t pulses = encoder.getCount();
  input = (pulses / MOTOR_PPR) * 2 * M_PI;
  // compute pid output
  motor_pid.Compute();

  Serial.print("count: \t");
  Serial.print(pulses);
  Serial.print("\tinput: \t");
  Serial.print(input);
  Serial.print("\tsetpoint: ");
  Serial.print(setpoint);
  Serial.print("\toutput: \t");
  Serial.println(output);

  // assign output to motor driver
  motor.move(output);
}

void angle_subscription_callback(const void * msgin) {
  const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
  setpoint = msg->data; 
}

void setup_micro_ros() {
  // set transport
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "motor_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "motor_angle"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &angle_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "angle_setpoint"));

  // create timer,
  const unsigned int timer_timeout = 200;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create PID Loop timer,
  const unsigned int pid_loop_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &pid_loop_timer,
    &support,
    RCL_MS_TO_NS(pid_loop_timeout),
    pid_loop_callback));

  // create executors
  RCCHECK(rclc_executor_init(&pub_executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&pub_executor, &timer));
  RCCHECK(rclc_executor_add_timer(&pub_executor, &pid_loop_timer));

  RCCHECK(rclc_executor_init(&sub_executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&sub_executor, &angle_subscriber, &angle_setpoint_msg, &angle_subscription_callback, ON_NEW_DATA));

}

void setup_hardware() {
  motor.init(MOTOR_A, MOTOR_B, 0, false);
  motor.setDecayMode(drv8833DecaySlow);
  encoder.attachFullQuad(ENCODER_A, ENCODER_B);
  encoder.setCount(0);
  motor_pid.SetMode(AUTOMATIC);
  motor_pid.SetOutputLimits(-100, 100);
}

void setup() {
  // for debugging
  Serial.begin(115200);
  pinMode(BL_PIN, OUTPUT);
  digitalWrite(BL_PIN, 1);

  setup_micro_ros();  
  setup_hardware();
  
  // tft stuff
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextSize(3);
}

void loop() {
  delay(5);
  RCSOFTCHECK(rclc_executor_spin_some(&pub_executor, RCL_MS_TO_NS(5)));
  RCSOFTCHECK(rclc_executor_spin_some(&sub_executor, RCL_MS_TO_NS(5)));
}