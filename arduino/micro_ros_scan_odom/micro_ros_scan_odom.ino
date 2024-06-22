#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include "sub_servo.hpp"
#include "sub_enc.hpp"
#include "sub_usonic.hpp"

//servo
#define SERVO_PIN 15
#define SV_FREQ 50  // サーボ信号周波数
#define MIN_SV_PULSE 0.6  // 最小パルス幅　0°
#define MAX_SV_PULSE 2.4  // 最大パルス幅 180°
volatile int svAng = 0;

//LaserScan publisher
const float pi = 3.14;
const uint8_t LASER_MSG_SIZE = 91; //データの個数
volatile int dataCount = 0;             //データ個数カウント用
volatile bool senseFlag = false;         			//測定開始用フラグ
volatile bool makeMap = false;

//micro-ros
rcl_subscription_t subscriber;
rcl_publisher_t publisher_scan;
rcl_publisher_t publisher_odom;
sensor_msgs__msg__LaserScan laser_scan_msg;
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

volatile float targetSpeedR = 0; //目標速度（カウント数/秒）
volatile float targetSpeedL = 0;

//odometry
volatile float odm_pos_x = 0.0;
volatile float odm_pos_y = 0.0;
volatile float odm_ang_z = 0.0;
volatile float odm_qt_qz = 0.0;
volatile float odm_qt_qw = 0.0;
float enc2dist;

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
    senseFlag = true;	//測定フラグ
    pid_control(targetSpeedR, targetSpeedL, countR, countL);
    uint32_t current_time_in_micros = micros();
    //calc odometry
    float dist = (speedR + speedL) / 2 * enc2dist;
    float theta = (speedR - speedL) / 2 * enc2dist / (pi * 100) / (2 * pi);
    odm_pos_x += 0.001;//(dist * cos(odm_ang_z) * 0.0001);
    odm_pos_y += (dist * sin(odm_ang_z) * 0.0001);
    odm_ang_z += theta;
    odm_qt_qz = sin(odm_ang_z/2);
    odm_qt_qw = cos(odm_ang_z/2);
    odom_msg.twist.twist.angular.z = odm_ang_z;
    odom_msg.pose.pose.position.x = odm_pos_x;
    odom_msg.pose.pose.position.y = odm_pos_y;
    odom_msg.pose.pose.orientation.z = odm_qt_qz;
    odom_msg.pose.pose.orientation.w = odm_qt_qw;
    //publish odometry
    odom_msg.header.stamp.sec = current_time_in_micros/1000000;
    odom_msg.header.frame_id.data = const_cast<char*>("odom");
    RCSOFTCHECK(rcl_publish(&publisher_odom, &odom_msg, NULL));

    //publish laser scan
    if ((dataCount >= LASER_MSG_SIZE) && makeMap) {	//データが溜まったらpublish
      for (int i = 0; i < LASER_MSG_SIZE; i++) {
        laser_scan_msg.ranges.data[i] = 0.5 + float(random(10))/100.0;//distance[i];
      }
      laser_scan_msg.header.stamp.sec = current_time_in_micros/1000000;
      RCSOFTCHECK(rcl_publish(&publisher_scan, &laser_scan_msg, NULL));
      dataCount = 0;
      targetSpeedR = 0;
      targetSpeedL = 0;
      makeMap = false;
    }
  }
}

void init_scan_msg() {
  laser_scan_msg.header.frame_id.data = const_cast<char*>("base_link");
  laser_scan_msg.header.stamp.sec = millis() / 1000;
  laser_scan_msg.angle_min = -0.5 * pi;
  laser_scan_msg.angle_max = 0.5 * pi;
  laser_scan_msg.angle_increment = 1 * pi / LASER_MSG_SIZE;
  laser_scan_msg.range_min = 0.0;
  laser_scan_msg.range_max = 1.0;
  laser_scan_msg.ranges.data = (float *)calloc(LASER_MSG_SIZE, sizeof(float));
  laser_scan_msg.ranges.size = LASER_MSG_SIZE;
  for (int i = 0; i < LASER_MSG_SIZE; i++) {
    distance[i] = 0;
  }
}

//操縦コマンドを受信
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  if(msg->linear.y == 1){
    makeMap = true;
  }
  targetSpeedR = msg->linear.x*400 + msg->angular.z*200;
  targetSpeedL = msg->linear.x*400 - msg->angular.z*200;
  Serial.print("並進速度: ");
  Serial.print(msg->linear.x*400);
  Serial.print(" 回転速度: ");
  Serial.println(msg->angular.z*200);
}

void setup() {
  Serial.begin(115200);
  enc2dist = pi * 60.0 / 48.0 / 20.0;

  //WiFiの場合
  set_microros_wifi_transports("WARPSTAR-AC6D64", "435C37A1AD1BE", "192.168.0.8", 8888);
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_scan,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    "/scan"));
  RCCHECK(rclc_publisher_init_default(
    &publisher_odom,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/odom"));

  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  
  set_usonic();
  set_enc_interrupts();
  set_motors();

  ledcSetup(2, SV_FREQ, 16);
  ledcAttachPin(SERVO_PIN, 2);

  init_scan_msg();
  rcl_publish(&publisher_scan, &laser_scan_msg, NULL);
}

void loop() {
  if (makeMap && senseFlag) {
   	sense_dist(dataCount);
    while (!distUpdated) {}
    dataCount++;
 		senseFlag = false;
    distUpdated = false;
    Serial.print(dataCount);
    svAng += 2;
    if(svAng > 180){
      svAng = 0;
    }
    ledcWrite(2, get_pulse_width(svAng));
  }
  delay(1);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

// パルス幅計算
int get_pulse_width(int angle) {
  float pulseMs = MIN_SV_PULSE + 
    (MAX_SV_PULSE - MIN_SV_PULSE) * angle / 180;
  return (int)(65536 * (pulseMs * SV_FREQ /1000.0));
}