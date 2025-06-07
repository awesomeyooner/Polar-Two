#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float64.h>

#include "../include/HardwareManager.hpp"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif
#include <exception>

#define NODE_NAME "esp32_bridge"
#define NAMESPACE "esp32"

hardware_component::HardwareManager hardwareManager;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    hardwareManager.update();
  }
}

bool is_agent_connected(int timeout_ms = 500, uint8_t attempts = 5){
  // Ping the agent
  rmw_ret_t ping_result = rmw_uros_ping_agent(timeout_ms, attempts);

  return ping_result == RMW_RET_OK;
}

void setup() {

  // ===== Boilerplate =====

    // Configure serial transport
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);

    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, NAMESPACE, &support));

    // create timer,
    const unsigned int timer_timeout = 20;
    RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, hardwareManager.getNumberOfHandles(), &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // ===== Custom Code =====

    // Initialize Hardwaremanager
    hardwareManager.initialize(&node, &executor);
}

void loop() {
  // Spin ROS
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  // Handle enabling status
  hardwareManager.toggleEnabled(is_agent_connected());
}
