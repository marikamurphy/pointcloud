
int smDirectionPin = 8; //Direction pin
int smStepPin = 9; //Stepper pin
char received; 
void setup(){
  /*Sets all pin to output; the microcontroller will send them(the pins) bits, it will not expect to receive any bits from thiese pins.*/
  pinMode(smDirectionPin, OUTPUT);
  pinMode(smStepPin, OUTPUT);
   
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Native USB only
  }  
  Serial.println("Connection established");
}
 
void loop(){

  digitalWrite(smDirectionPin, LOW); //Writes the direction to the EasyDriver DIR pin. (LOW is counter clockwise).
  /*Turns the motor fast 1600 steps*/
  if(Serial.available() > 0){
    received = Serial.read();
    if(received == 'r'){
      for (int i = 0; i < 1600; i++){
        digitalWrite(smStepPin, HIGH);
        delayMicroseconds(700);
        digitalWrite(smStepPin, LOW);
        delayMicroseconds(700);
      }
      received = 's';
    }
  }
 
}



