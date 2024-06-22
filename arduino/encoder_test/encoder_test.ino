//encoder pin settings
const uint8_t encRAPin = 36; //エンコーダ入力ピン
const uint8_t encRBPin = 39; //エンコーダ入力ピン
const uint8_t encLAPin = 34; //エンコーダ入力ピン
const uint8_t encLBPin = 35; //エンコーダ入力ピン
volatile int32_t countR = 0; //右エンコーダカウント
volatile int32_t countL = 0; //左エンコーダカウント
volatile bool encFlag = false; //エンコーダ変化あり

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //外部（ピン変化）割り込み設定
  attachInterrupt(digitalPinToInterrupt(encRAPin), isr_RA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encRBPin), isr_RB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encLAPin), isr_LA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encLBPin), isr_LB, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(encFlag == true){ //カウント数を表示
    Serial.print("countR : ");
    Serial.print(countR);
    Serial.print(" countL : ");
    Serial.println(countL);
    encFlag = false;
  }
}

void isr_RA(){ //右エンコーダA信号割り込み関数
  encFlag = true;
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
  encFlag = true;
  if(digitalRead(encRBPin) == HIGH){ //pin B rose
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
  encFlag = true;
  if(digitalRead(encLAPin) == HIGH){ //pin A rose
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
  encFlag = true;
  if(digitalRead(encLBPin) == HIGH){ //pin B rose
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