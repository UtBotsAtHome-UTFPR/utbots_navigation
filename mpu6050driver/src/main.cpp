// We include the Arduino library for general Arduino functionality
#include <Arduino.h>
// The micro_ros_platformio library provides the functions to communicate with ROS2
#include <micro_ros_platformio.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// These are core ROS2 libraries for creating nodes, publishers, and executors
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// This is a standard ROS2 message type, an integer
#include <sensor_msgs/msg/imu.h>

// Ensure that the transport layer being used is Arduino Serial.
// If it's not, compilation is stopped and error is printed.
#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// Define ROS2 objects for a publisher, a message, an executor, support objects, an allocator, a node, and a timer
rcl_publisher_t publisher;
sensor_msgs__msg__Imu imu_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

Adafruit_MPU6050 mpu;

#define LED_PIN 13

// Macros for checking return of ROS2 functions and entering an infinite error loop in case of error
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Infinite error loop function. If something fails, the device will get stuck here
void error_loop() {
  while(1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// This is the function that will be called every time the timer expires
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    // We publish our message here
    RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    imu_msg.linear_acceleration.x = a.acceleration.x;
    imu_msg.linear_acceleration.y = a.acceleration.y;
    imu_msg.linear_acceleration.z = a.acceleration.z;

    imu_msg.angular_velocity.x = a.gyro.x;
    imu_msg.angular_velocity.y = a.gyro.y;
    imu_msg.angular_velocity.z = a.gyro.z;
  }
}

void setup() {
  // Start serial communication with a baud rate of 115200
  Serial.begin(115200);
  // Configure Micro-ROS library to use Arduino serial
  set_microros_serial_transports(Serial);
  // Allow some time for everything to start properly
  delay(2000);

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  // Get the default memory allocator provided by rcl
  allocator = rcl_get_default_allocator();

  // Initialize rclc_support with default allocator
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Initialize a ROS node with the name "micro_ros_platformio_node"
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // Initialize a ROS publisher with the name "micro_ros_platformio_node_publisher" to publish Int32 messages
  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_info_topic"));


  // Initialize a timer with a period of 1 second which calls the function timer_callback() every time it expires
  const unsigned int timer_timeout = 200;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Initialize an executor that will manage the execution of all the ROS entities (publishers, subscribers, services, timers)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // Add our timer to the executor
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
}

void loop() {
  // Wait a little bit
  delay(100);
  // Execute pending tasks in the executor. This will handle all ROS communications.
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
