const uint8_t PWM_FREQ = 100; //PWM周波数

//motor pin settings
const uint8_t mtRPwmPin = 32; //R wheel用PWM出力ピンをGPIO32に設定
const uint8_t mtRDir2Pin = 33; //R wheel用方向出力ピンをGPIO33に設定
const uint8_t mtRDir1Pin = 25; //R wheel用方向出力ピンをGPIO25に設定
const uint8_t mtLDir1Pin = 26; //L wheel用方向出力ピンをGPIO26に設定
const uint8_t mtLDir2Pin = 27; //L wheel用方向出力ピンをGPIO27に設定
const uint8_t mtLPwmPin = 14; //L wheel用PWM出力ピンをGPIO14に設定

void setup() {
  //motor settings
  ledcSetup(0, PWM_FREQ, 8); //PWM幅分解能8bitで指定
  ledcAttachPin(mtRPwmPin, 0); //mtRPwmPinピンにチャネル0を設定
  ledcSetup(1, PWM_FREQ, 8); //PWM幅分解能8bitで指定
  ledcAttachPin(mtLPwmPin, 1); //mtLPwmPinピンにチャネル1を設定
  pinMode(mtRDir2Pin,OUTPUT);
  pinMode(mtRDir1Pin,OUTPUT);
  pinMode(mtLDir1Pin,OUTPUT);
  pinMode(mtLDir2Pin,OUTPUT);
  //R wheel output
  ledcWrite(0, 150); //PWMデューティ比:100/255
  digitalWrite(mtRDir2Pin, HIGH);
  digitalWrite(mtRDir1Pin, LOW);
  //L wheel output
  ledcWrite(1, 150); //PWMデューティ比:100/255
  digitalWrite(mtLDir1Pin, HIGH);
  digitalWrite(mtLDir2Pin, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
}

