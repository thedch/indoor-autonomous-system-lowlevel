#include "Motors.h"
#include <ros.h>
#include <std_msgs/Int16.h>

Motors motors(32,33,31,34);
 
void setup() {
  Serial.begin(9600);
}

void loop() {
  motors.readEncoders();
  
}
