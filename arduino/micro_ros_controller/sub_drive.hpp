#pragma once

const uint8_t PWM_FREQ = 100;   //PWM周波数
const float DURATION = 0.1;     //制御周期（秒）

//motor pin settings
const uint8_t mtRPwmPin = 32;   //R wheel用PWM出力ピンをGPIO32に設定
const uint8_t mtRDir2Pin = 33;  //R wheel用方向出力ピンをGPIO33に設定
const uint8_t mtRDir1Pin = 25;  //R wheel用方向出力ピンをGPIO25に設定
const uint8_t mtLDir1Pin = 26;  //L wheel用方向出力ピンをGPIO26に設定
const uint8_t mtLDir2Pin = 27;  //L wheel用方向出力ピンをGPIO27に設定
const uint8_t mtLPwmPin = 14;   //L wheel用PWM出力ピンをGPIO14に設定

//servo settings
volatile int32_t prevCountR = 0;  //前回カウント
volatile int32_t prevCountL = 0;
volatile float errPrevR = 0;      //前回誤差
volatile float errPrevL = 0;
volatile float errIR = 0;         //誤差の累積
volatile float errIL = 0;
float Kp = 0.05;                  //比例ゲイン
float Ki = 1.0;                   //積分ゲイン
float Kd = 0.001;                 //微分ゲイン
float speedR = 0.0;
float speedL = 0.0;

/*void stopPWM(){
  ledcDetachPin(mtRPwmPin);
  ledcDetachPin(mtLPwmPin);
}

void resumePWM(){
  ledcAttachPin(mtRPwmPin, 8);
  ledcAttachPin(mtLPwmPin, 9);
}*/

void pid_control(float tgtSpeedR, float tgtSpeedL, int cntR, int cntL){
  //R wheel
  speedR = float(cntR - prevCountR) / DURATION; //現在の回転速度（カウント/秒）
  float errP = tgtSpeedR - speedR;                  //目標速度との差
  errIR += (errP * DURATION);                       //速度誤差の累積
  float errD = (errP - errPrevR) / DURATION;        //誤差の変化具合
  float dutyR = Kp * errP + Ki * errIR + Kd * errD; //モーターに与えるデューティを計算
  if(dutyR > 0){ //正方向トルク
    ledcWrite(0, constrain(abs(dutyR), 0, 255));    //255以下に制限
    digitalWrite(mtRDir2Pin, HIGH);
    digitalWrite(mtRDir1Pin, LOW);
  }else{ //逆方向トルク
    ledcWrite(0, constrain(abs(-dutyR), 0, 255));   //255以下に制限
    digitalWrite(mtRDir2Pin, LOW);
    digitalWrite(mtRDir1Pin, HIGH);
  }
  prevCountR = cntR;                //カウント値を更新（次回用）
  errPrevR = errP;                  //誤差を更新（次回用）
  
  //L wheel
  speedL = float(cntL - prevCountL) / DURATION;
  errP = tgtSpeedL - speedL;
  errIL += (errP * DURATION);
  errD = (errP - errPrevL) / DURATION;
  float dutyL = Kp * errP + Ki * errIL + Kd * errD;
  if(dutyL > 0){ //正方向トルク
    ledcWrite(1, constrain(abs(dutyL), 0, 255)); //255以下に制限
    digitalWrite(mtLDir1Pin, HIGH);
    digitalWrite(mtLDir2Pin, LOW);
  }else{ //逆方向トルク
    ledcWrite(1, constrain(abs(-dutyL), 0, 255)); //255以下に制限
    digitalWrite(mtLDir1Pin, LOW);
    digitalWrite(mtLDir2Pin, HIGH);
  }
  prevCountL = cntL;
  errPrevL = errP;
}

void set_motors(){
  //motor settings
  ledcSetup(0, PWM_FREQ, 8);    //PWM幅分解能8bitで指定
  ledcAttachPin(mtRPwmPin, 0);  //mtRPwmPinピンにチャネル0を設定
  ledcSetup(1, PWM_FREQ, 8);    //PWM幅分解能8bitで指定
  ledcAttachPin(mtLPwmPin, 1);  //mtLPwmPinピンにチャネル1を設定
  pinMode(mtRDir2Pin,OUTPUT);
  pinMode(mtRDir1Pin,OUTPUT);
  pinMode(mtLDir1Pin,OUTPUT);
  pinMode(mtLDir2Pin,OUTPUT);
}