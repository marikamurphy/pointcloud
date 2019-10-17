#include <Stepper.h>

#define STEPS 2038 // the number of steps in one revolution of your motor (28BYJ-48)
	
Stepper stepper(STEPS, 8, 10, 9, 11);
char received;
void setup() {
  // set up Serial library at 9600 bps
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Native USB only
  }  
  // prints to console that turntable connection established
  Serial.println("Connection to turntable established");  
}

void loop() {
  stepper.setSpeed(1); // 1 rpm
  //rotate five degre
  if(Serial.available() > 0){
    // read the data
    received = Serial.read();
    if(received == 'r')
      stepper.step(25);
  }
  
  Serial.print("omg, omg, omg\n");
  
  // stepper.setSpeed(6); // 6 rpm
  // stepper.step(-2038); // do 2038 steps in the other direction with faster speed -- corresponds to one revolution in 10 seconds
}


