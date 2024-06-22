#pragma once

//encoder pin settings
const uint8_t encRAPin = 36;  //エンコーダ入力ピン
const uint8_t encRBPin = 39;  //エンコーダ入力ピン
const uint8_t encLAPin = 34;  //エンコーダ入力ピン
const uint8_t encLBPin = 35;  //エンコーダ入力ピン
volatile int32_t countR = 0;  //右エンコーダカウント
volatile int32_t countL = 0;  //左エンコーダカウント

//void set_enc_interrupts();
void isr_RA();
void isr_RB();
void isr_LA();
void isr_LB();

void set_enc_interrupts(){
  //外部（ピン変化）割り込み設定
  attachInterrupt(digitalPinToInterrupt(encRAPin), isr_RA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encRBPin), isr_RB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encLAPin), isr_LA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encLBPin), isr_LB, CHANGE);
}

void isr_RA(){ //右エンコーダA信号割り込み関数
  if(digitalRead(encRAPin) == HIGH){ //A RISE
    if(digitalRead(encRBPin) == LOW){ //そのときB : LOWなら
      countR--; //タイヤ逆転
    }else {
      countR++; //タイヤ正転
    }
  }else{  //pin A FALL
    if(digitalRead(encRBPin) == LOW) { //そのときB : LOWなら
      countR++; //タイヤ正転
    }else {
      countR--; //タイヤ逆転
    }
  }
}

void isr_RB(){ //右エンコーダB信号割り込み関数
  if(digitalRead(encRBPin) == HIGH){ 
    if(digitalRead(encRAPin) == LOW) {
      countR++;
    }else {
      countR--;
    }
  }else{  //pin B fell
    if(digitalRead(encRAPin) == LOW) {
      countR--;
    }else {
      countR++;
    }
  }
}

void isr_LA(){ //左エンコーダA信号割り込み関数
  if(digitalRead(encLAPin) == HIGH){ 
    if(digitalRead(encLBPin) == LOW) {
      countL++;
    }else {
      countL--;
    }
  }else{  //pin A fell
    if(digitalRead(encLBPin) == LOW) {
      countL--;
    }else {
      countL++;
    }
  }
}

void isr_LB(){ //左エンコーダB信号割り込み関数
  if(digitalRead(encLBPin) == HIGH){ 
    if(digitalRead(encLAPin) == LOW) {
      countL--;
    }else {
      countL++;
    }
  }else{  //pin A fell
    if(digitalRead(encLAPin) == LOW) {
      countL++;
    }else {
      countL--;
    }
  }
}