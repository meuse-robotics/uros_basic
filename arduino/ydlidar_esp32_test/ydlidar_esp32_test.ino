void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200,SERIAL_8N1,12,15);
  ledcSetup(2,10000,8);
  ledcAttachPin(13,2);
  ledcWrite(2,90);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial1.available()){
    unsigned char dist = Serial1.read();
    Serial.println(dist,HEX);
    
  }
}
