// laser scan dummy data r=0.5

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>

//LaserScan publisher
const float pi = 3.14;
const uint8_t LASER_MSG_SIZE = 91; //データの個数

//micro-ros
rcl_publisher_t publisher;
sensor_msgs__msg__LaserScan scan_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    Serial.println("error");
    delay(1000);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    for (int i = 0; i < LASER_MSG_SIZE; i++) {
      scan_msg.ranges.data[i] = 0.5 + float(random(10))/100.0;
    }
    RCSOFTCHECK(rcl_publish(&publisher, &scan_msg, NULL));
  }
}

void init_scan_msg() {
  scan_msg.header.frame_id.data = const_cast<char*>("laser_frame");
  scan_msg.header.stamp.sec = millis() / 1000;
  scan_msg.angle_min = -0.5 * pi;
  scan_msg.angle_max = 0.5 * pi;
  scan_msg.angle_increment = 1 * pi / LASER_MSG_SIZE;
  //scan_msg.time_increment = 9.1 / LASER_MSG_SIZE;
  scan_msg.range_min = 0.0;
  scan_msg.range_max = 1.0;
  scan_msg.ranges.data = (float *)calloc(LASER_MSG_SIZE, sizeof(float));
  scan_msg.ranges.size = LASER_MSG_SIZE;
}

void setup() {
  Serial.begin(115200);

  //*** micro-ros settings ↓***
  //WiFiの場合
  set_microros_wifi_transports("WIFI SSID", "PASSWORD", "192.168.0.8", 8888);
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  //*** micro-ros settings ↑***

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    "micro_ros_arduino_node_publisher"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  // total number of handles = #timers
  unsigned int num_handles = 1;
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  init_scan_msg();
  rcl_publish(&publisher, &scan_msg, NULL);
}

void loop() {
  delay(1);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
