const uint8_t TrigPin = 12; //超音波送信ピン
const uint8_t EchoPin = 13; //超音波受信ピン  
volatile long echoStart = 0;
volatile long echoEnd = 0;
volatile long echoDuration = 0;
float travelTime = 0; //超音波往復時間
float distance = 0; //距離
volatile bool distUpdated = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  //ultrasonic sensor
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(EchoPin), isr_EC, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  distUpdated = false;
  digitalWrite(TrigPin, LOW); 
  delayMicroseconds(2); 
  digitalWrite(TrigPin, HIGH); //超音波送信
  delayMicroseconds( 10 );
  digitalWrite(TrigPin, LOW);
  while (!distUpdated) {} //測定完了
  Serial.print("Distance : ");
  Serial.println(distance);
  delay(1000);
}

void isr_EC(){
  if(digitalRead(EchoPin) == HIGH){
    echoEnd = 0;
    echoStart = micros();
  }else{
    echoEnd = micros();
    echoDuration = echoEnd - echoStart;
    distance = echoDuration / 2.0 * 340.0 / 1000000.0;
    distUpdated = true;
  }
}
