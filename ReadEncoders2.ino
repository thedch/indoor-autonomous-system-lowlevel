// Encoder Interface //
// Kevin Beher, Juan Huerta//

#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
Encoder myEnc1(32, 33);
Encoder myEnc2(31, 34);

//   avoid using pins with LEDs attached
int Angle1 = 0;
int ticks1 = 0;
int Angle2 = 0;
int ticks2 = 0;
int resetFlag = 0; // To reset the cumulative angle when a new geo. twist message is received. 

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
}

long oldPosition1  = -999;
long oldPosition2  = -999;


void loop() {
  if(resetFlag == 0){
  long newPosition1 = myEnc1.read();
  long newPosition2 = myEnc2.read();
  if (newPosition1 != oldPosition1) {
    oldPosition1 = newPosition1;
    ticks1 = newPosition1  / 4;
    Angle1 = ticks1 * 15;
//    Serial.print("Encoder A angle: ");
//    Serial.print(Angle1);
    
  }
  if (newPosition2 != oldPosition2) {
    oldPosition2 = newPosition2;
    ticks2 = newPosition2  / 4;
    Angle2 = ticks2 * 15;
//    Serial.print("Encoder A angle: ");
//    Serial.println(Angle1);
    
  }
  }
  Serial.print("Encoder A angle: ");
  Serial.print(Angle1);
  Serial.print(", Encoder B angle: ");
  Serial.print(Angle2);
  Serial.println();
  
  if(resetFlag == 1){ // This portion of code resets the cumulative angle when the new geo. twist message is received.
    // Here we will read and publish encoder data. 
    ticks1 = 0;           
    ticks2 = 0;
    Angle1 = 0;
    Angle2 = 0;
    resetFlag = 1; // Goes back to keeping track of encoders. 
  }
}
