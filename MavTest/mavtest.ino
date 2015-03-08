#include <FastSerial.h>
#include <GCS_MAVLink.h>
#include "Vars.h"

#define TELEMETRY_SPEED  57600  // How fast our MAVLink telemetry is coming to Serial port, 57600 for sure, 115200 better

FastSerialPort0(Serial);

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(TELEMETRY_SPEED,256,0);
  Serial.println("Init");

}

  unsigned long time = 0;

void loop() {
  // put your main code here, to run repeatedly: 
  time = millis();
  
  while (Serial.available() < 5) {} //at least one message is there
  
  Serial.print("loop start:");Serial.println(millis() - time);
  time = millis();
  
  read_mavlink(10);

  Serial.print("read mavlink time:");Serial.println(millis() - time);
  
  
}
