#pragma once

const uint16_t LASER_MSG_SIZE = 360;
volatile int dataQty = 0;
volatile float distance[360];
volatile float tempDistance[50];
volatile unsigned char readDataState = 0;
volatile float firstAngle = 0.0;
volatile float lastAngle = 0.0;
volatile int subCount = 0;
volatile bool byteLM = false;
volatile uint8_t tempByte = 0;
volatile uint8_t lastByte = 0;
volatile uint16_t checkSum = 0;
volatile uint16_t exor = 0;

void init_lidar(){
  Serial1.begin(115200,SERIAL_8N1,12,15);
  ledcSetup(0,10000,8);
  ledcAttachPin(13,0);
  ledcWrite(0,90);

  for(int i=0;i<LASER_MSG_SIZE;i++){
    distance[i] = 0;
  }
}

void read_data(){
  tempByte = Serial1.read();
    if(tempByte == 0x55 && lastByte == 0xAA){ // received packet header
      readDataState = 1;
      subCount = 0;
      byteLM = false;
      exor = (tempByte<<8) | lastByte;
    }else if(byteLM == false){  //read least byte
      byteLM = true;
    }else if(byteLM == true){   //read most byte
      byteLM = false;
      uint16_t value = (tempByte<<8) + lastByte;
      if(readDataState == 1){ //if already read packet header
        exor = exor^value;
        readDataState = 2;
        dataQty = tempByte; //read sample quantity (LSN)
      }else if(readDataState == 2){
        exor = exor^value;
        firstAngle = (value>>1) / 64; //calc start angle (FSA)
        readDataState = 3;
      }else if(readDataState == 3){
        exor = exor^value;
        lastAngle = (value>>1) / 64;  //calc end angle (LSA)
        readDataState = 4;
      }else if(readDataState == 4){
        checkSum = value;             //read checksum (CS)
        readDataState = 5;
      }else if(readDataState == 5){
        exor = exor^value;
        if(subCount < dataQty-1){
          if(value != 0){
            tempDistance[subCount] = float(value) / 4;  //calc sample data
          }
          subCount++;
        }else if(subCount == dataQty-1){  //end of data
          if(exor == checkSum){
            tempDistance[subCount] = float(value) / 4;
            float angCorrect = atan2(21.8*(155.3-tempDistance[0])/(155.3*tempDistance[0]),1.0);
            firstAngle += angCorrect;
            angCorrect = atan2(21.8*(155.3-tempDistance[subCount])/(155.3*tempDistance[subCount]),1.0);
            lastAngle += angCorrect;
            for(int i=0;i<subCount;i++){
              float tempAngle = firstAngle + fmod((lastAngle - firstAngle + 360.0),360.0) * i / subCount;
              angCorrect = atan2(21.8*(155.3-tempDistance[i])/(155.3*tempDistance[i]),1.0);
              tempAngle += angCorrect;
              distance[int(tempAngle)%360] = tempDistance[i] / 1000.0;
            }
          }
        }
      }
    }
    lastByte = tempByte;
}