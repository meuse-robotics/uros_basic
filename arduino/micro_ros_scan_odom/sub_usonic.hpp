#pragma once

const uint8_t TrigPin = 12;
const uint8_t EchoPin = 13;
volatile long echoStart = 0;
volatile long echoEnd = 0;
volatile long echoDuration = 0;
volatile float travelTime = 0;
volatile float distance[91];
volatile bool distUpdated = false;
volatile uint8_t datCount = 0;

void isr_EC();

void set_usonic(){
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(EchoPin), isr_EC, CHANGE);
}

void sense_dist(int cnt){
  datCount = cnt;
  digitalWrite(TrigPin, LOW); 
  delayMicroseconds(2); 
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds( 10 );
  digitalWrite(TrigPin, LOW);
}

void isr_EC(){
  if(digitalRead(EchoPin) == HIGH){
    echoEnd = 0;
    echoStart = micros();
  }else{
    echoEnd = micros();
    echoDuration = echoEnd - echoStart;
    distance[datCount] = echoDuration / 2.0 * 340.0 / 1000000.0;
    distUpdated = true;
    
  }
}