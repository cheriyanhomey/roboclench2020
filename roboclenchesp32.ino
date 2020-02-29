#include <analogWrite.h>


#define leftpwm 14
#define leftb 26
#define leftf 27
#define rightpwm 32
#define rightb 25
#define rightf 33

#define armpwm 14
#define up 26
#define down 27
float deg;
static const int servoPin = 33;



TaskHandle_t arm;
int motorSpeedl = 0;
int motorSpeedr = 0;
int motorSpeedarm = 400;
Servo servo1;



void codeForarm( void * parameter )
{
  for (;;) {
  int joyx2 = analogRead(35); // Read Joysticks X-axis
  int joyy2 = analogRead(12); // Read Joysticks Y-axis
   Serial.print("core:");
  Serial.print( xPortGetCoreID());
  Serial.print("\n");
    delay(1000);                                       //code for arm

 if (joyy2 < 1600) {
    Serial.print("up");  
      Serial.print("\n");
    // Set Motor A forward
    digitalWrite(down, LOW);
    digitalWrite(up, HIGH);
    // Set Motor B forward
  
 }

 else if (joyy2 > 2100) 
 {
    Serial.print("down");  
      Serial.print("\n");
    // Set Motor A forward
    digitalWrite(up, LOW);
    digitalWrite(down, HIGH);
    // Set Motor B forward
    digitalWrite(opn, LOW);
    digitalWrite(clse, LOW);
    
    
  }
 else if (joyx2 < 1600) 
 {
    Serial.print("close "); 
    Serial.print(joyx2); 
      Serial.print("\n");
    deg = 180-joyx2*0.1125;
      servo1.write(deg);
        Serial.println(deg);
        delay(200);}
 }
  else if (joyx2 > 2100) 
 {
    
    Serial.print("open "); 
   Serial.print(joyx2); 
     Serial.print("\n");
    deg = 90-((joyx2-2100)*0.090225563);
      servo1.write(deg);
        Serial.println(deg);
        delay(200);
    
  }
  analogWrite(armpwm, motorSpeedarm);           // Send PWM signal to motor A
  

  }
}

void setup() {
  pinMode(rightpwm, OUTPUT);
  pinMode(leftpwm, OUTPUT);
  pinMode(leftb, OUTPUT);
  pinMode(leftf, OUTPUT);
  pinMode(rightb, OUTPUT);
  pinMode(rightf, OUTPUT);
  Serial.begin(9600);
  
    Serial.begin(115200);
    servo1.attach(servoPin);

  
  xTaskCreatePinnedToCore(
    codeForarm,
    "arm",
    1000,
    NULL,
    1,
    &arm,
    0);
}

void loop() {
 Serial.print("core:");
  Serial.print( xPortGetCoreID());
  Serial.print("\n");
  delay(1000); 
  
  int joyx = analogRead(35); // Read Joysticks X-axis
  int joyy = analogRead(12); // Read Joysticks Y-axis
   
   Serial.print("VRX_value=");
  Serial.print(joyx);
  Serial.print("\n");

  Serial.print("VRY_value=");
  Serial.print(joyy);
  Serial.print("\n");
  //delay(500);

  // Y-axis used for forward and backward control
  if (joyy < 1600) {
    Serial.print("front");  
      Serial.print("\n");
    // Set Motor A forward
    digitalWrite(leftb, LOW);
    digitalWrite(leftf, HIGH);
    // Set Motor B forward
    digitalWrite(rightb, LOW);
    digitalWrite(rightf, HIGH);
    // Convert the declining Y-axis readings for going backward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedl = map(joyy, 1600, 0, 0, 1023);
    motorSpeedr = map(joyy, 1600, 0, 0, 1023);
  }
  else if (joyy > 2100) {
    Serial.print("back");  
      Serial.print("\n");
      // Set Motor A backward
    digitalWrite(leftb, HIGH);
    digitalWrite(leftf, LOW);
    // Set Motor B backward
    digitalWrite(rightb, HIGH);
    digitalWrite(rightf, LOW);
  
    
    // Convert the increasing Y-axis readings for going forward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedl = map(joyy, 2100, 4095, 0, 1023);
    motorSpeedr = map(joyy, 2100, 4095, 0, 1023);
  }
  // If joystick stays in middle the motors are not moving
  else {
    motorSpeedl = 0;
    motorSpeedr = 0;
  }

  // X-axis used for left and right control
  if (joyx < 1600) {
     Serial.print("left");
     Serial.print("\n");
    // Convert the declining X-axis readings from 470 to 0 into increasing 0 to 255 value  left
    int xMapped = map(joyx, 1600, 0, 0, 1023);
    // Move to left - decrease left motor speed, increase right motor speed
    motorSpeedl = motorSpeedl - xMapped;
    motorSpeedr = motorSpeedr + xMapped;
    // Confine the range from 0 to 1023
    if (motorSpeedl < 0) {
      motorSpeedl = 0;
    }
    if (motorSpeedr > 1023) {
      motorSpeedr = 1023;
    }
  }                                            //l is a r is b
  if (joyx > 2100) {
    Serial.print("right");
    Serial.print("\n");
    // Convert the increasing X-axis readings from 550 to 1023 into 0 to 255 value         right
    int xMapped = map(joyx, 2100, 4095, 0, 1023);
    // Move right - decrease right motor speed, increase left motor speed
    motorSpeedl = motorSpeedl + xMapped;
    motorSpeedr = motorSpeedr - xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedl > 1023) {
      motorSpeedl = 1023;
    }
    if (motorSpeedr < 0) {
      motorSpeedr = 0;
    }
  }
  // Prevent buzzing at low speeds (Adjust according to your motors.
  if (motorSpeedl < 100) {
    motorSpeedl = 0;
  }
  if (motorSpeedr < 100) {
    motorSpeedr = 0;
  }
  analogWrite(leftpwm, motorSpeedl); // Send PWM signal to motor A
  analogWrite(rightpwm, motorSpeedr); // Send PWM signal to motor B

}
