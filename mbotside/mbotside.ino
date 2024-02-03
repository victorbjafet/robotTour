#include <MeMCore.h>

MeDCMotor motor1(M2); //left motor
MeDCMotor motor2(M1); //right motor


void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2); //dont keep checking for more serial
  pinMode(A7,INPUT);
}

bool prevPressed = 0;

void loop() {
  if(Serial.available()) {
    String recieved = Serial.readString();
    String leftMotorStr;
    String rightMotorStr;
    for (int i = 0; i < 5; i++) {
      leftMotorStr += recieved[i];
    }
    for (int i = 6; i < 11; i++) {
      rightMotorStr += recieved[i];
    }

    //format of incoming serial values:
    //11 characters
    //5 characters per motor seperated with a space
    //first value is left motor, second value is second motor
    //max value is 1023 (01023)
    //min value is -1023 (-1023)
    
    motor1.run(leftMotorStr.toInt());
    motor2.run(rightMotorStr.toInt());
  
  }

  if (analogRead(A7) < 10) {
    if (prevPressed == 0) {
      Serial.println("start");
    }
    prevPressed = 1;
  }
  else {
    prevPressed = 0;
  }
}
