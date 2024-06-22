#include <Ticker.h>
#include "sub_lidar.hpp"
Ticker tickerSet;

void tick(){
  for(int i=0;i<LASER_MSG_SIZE-1;i++){
    Serial.print(distance[i]);
    Serial.print(",");
  }
  Serial.println(distance[LASER_MSG_SIZE-1]);
}

void setup() {
  Serial.begin(115200);
  init_lidar();
  tickerSet.attach_ms(1000, tick);
}

void loop() {
  if(Serial1.available()){
    read_data();
  }
}
