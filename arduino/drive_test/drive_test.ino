#include <Ticker.h>
#include "sub_drive.hpp"
#include "sub_enc.hpp"

float targetSpeedR = 200; //目標速度（カウント数/秒）
float targetSpeedL = 200;

Ticker tickerSet;

void tick(){ //タイマー割込み 0.1秒ごと
  //モーターを制御
  pid_control(targetSpeedR, targetSpeedL, countR, countL);
}

void setup() {
  set_motors();                             //モーター設定

  set_enc_interrupts();                     //エンコーダ設定

  tickerSet.attach_ms(DURATION*1000, tick); //タイマー割込み設定
}

void loop() {
  // put your main code here, to run repeatedly:
}
