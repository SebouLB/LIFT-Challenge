//Centrale inertielle
#include <Wire.h>

#define CMPS12_ADDRESS 0x60  // Address of CMPS12 shifted right one bit for arduino wire library
#define ANGLE_8  1           // Register to read 8bit angle from
#define PITCH_ 4

unsigned char high_byte, low_byte, angle8;
char pitch, roll;
unsigned int angle16;

//Moteur 
const int motorPin1 = 9; 
const int motorPin2 = 8; 
int speed;
String input;


void setup(){ 
//Set pins as outputs 
  pinMode(motorPin1, OUTPUT); 
  pinMode(motorPin2, OUTPUT);
  analogWrite(motorPin1, 0); 
  analogWrite(motorPin2, 0);
  speed=255;
  
  Serial.begin(9600);  // Start serial port
  Wire.begin();
} 
void loop(){
//Motor Control A in both directions
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(PITCH_);                     //Sends the register we wish to start reading from
  Wire.endTransmission();
 
  // Request 5 bytes from the CMPS12
  // this will give us the 8 bit bearing, 
  // both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(CMPS12_ADDRESS, 1);
  //Serial.print("1");      
  
  while(Wire.available() < 1);        // Wait for all bytes to come back
  
  pitch = Wire.read();
  roll = Wire.read();
  
  Serial.print("pitch: ");          // Display pitch data
  Serial.println(pitch, DEC);
    
  if (pitch>1){
    analogWrite(motorPin2, 0); 
    analogWrite(motorPin1, speed);
  }
  else if (pitch<-1){
    analogWrite(motorPin1, 0); 
    analogWrite(motorPin2, speed);
  }
  else {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 0);
  }
  
  delay(100);                           // Short delay before next loop
}
