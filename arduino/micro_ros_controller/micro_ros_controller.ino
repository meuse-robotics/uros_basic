//#include <Ticker.h>
//Ticker tickerSet;
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include "sub_drive.hpp"
#include "sub_enc.hpp"
#include "sub_usonic.hpp"


/*void tick(){
  for(int i=0;i<LASER_MSG_SIZE-1;i++){
    Serial.print(distance[i]);
    Serial.print(",");
  }
  Serial.println(distance[LASER_MSG_SIZE-1]);
}*/
//micro-ros
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

volatile float targetSpeedR = 0; //目標速度（カウント数/秒）
volatile float targetSpeedL = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

/*void tick(){ //タイマー割込み 0.1秒ごと
  	pid_control(targetSpeedR, targetSpeedL, countR, countL);	//モーターを制御
  	//sense_dist();	//障害物チェック
}*/
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    senseFlag = true;	//測定フラグ
    pid_control(targetSpeedR, targetSpeedL, countR, countL);
  }
}
void error_loop(){
  while(1){
    Serial.println("error");
    delay(1000);
  }
}

//操縦コマンドを受信
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  targetSpeedR = msg->linear.x*400 + msg->angular.z*200;
  targetSpeedL = msg->linear.x*400 - msg->angular.z*200;
  Serial.print("並進速度: ");
  Serial.print(msg->linear.x*400);
  Serial.print(" 回転速度: ");
  Serial.println(msg->angular.z*200);
  //driveCount = 0;
}

void setup() {
  Serial.begin(115200);
  //WiFiの場合
  set_microros_wifi_transports("WARPSTAR-AC6D64", "435C37A1AD1BE", "192.168.0.8", 8888);
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

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
  // total number of handles = #timer + #subscriber
  unsigned int num_handles = 2;
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  
  set_enc_interrupts();
  set_motors();
  set_usonic();
  //tickerSet.attach_ms(DURATION * 2000, tick); //タイマー割込み設定（0.1秒）
}

void loop() {
  delay(1);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
