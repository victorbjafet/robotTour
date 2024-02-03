const int leftP1 = 9; //pin 1 is the signal pin for the wire closer to the front/more to the middle of the encoder
const int leftP2 = 8; //pin 2 is the signal pin for the wire further back/closer to the encoders screw holes

const int rightP1 = 11;
const int rightP2 = 10;


void setup() {
  Serial.begin(115200);

  pinMode(leftP1, INPUT);
  pinMode(leftP2, INPUT);

  digitalWrite(leftP1, HIGH);
  digitalWrite(leftP2, HIGH);


  pinMode(rightP1, INPUT);
  pinMode(rightP2, INPUT);

  digitalWrite(rightP1, HIGH);
  digitalWrite(rightP2, HIGH);
}

//encoders will always be relative, always starts at 0 from whatever position it begins in
//180 ticks is a full rotation

//init left encoder variables
volatile long leftEncoderValue = 0; //keeps track of encoder value
int lastLeftEncoderState;

int oldlP1;
int oldlP2;

int lP1;
int lP2;



//init right encoder variables
volatile long rightEncoderValue = 0; //keeps track of encoder value
int lastRightEncoderState;

int oldrP1;
int oldrP2;

int rP1;
int rP2;



// int change; //variable to keep track of change in encoder values (not in use rn)


void loop() {

  //get current left signal pin values
  lP1 = digitalRead(leftP1); 
  lP2 = digitalRead(leftP2);

  if (lP1 != oldlP1 || lP2 != oldlP2) { //if there is any change in value in the left signal pins

    // change = 1;

    int leftEncoderState = lP1 * 2 + lP2; // 0 0 = 0, 0 1 = 1, 1 0 = 2, 1 1 = 3

    switch (leftEncoderState) { //basically increments the encoder value accordingly based on the change that happened
      case 0:
        if (lastLeftEncoderState == 1) leftEncoderValue++;
        else if (lastLeftEncoderState == 2) leftEncoderValue--;
      case 3:
        if (lastLeftEncoderState == 2) leftEncoderValue++;
        else if (lastLeftEncoderState == 1) leftEncoderValue--;
      case 1: 
        if (lastLeftEncoderState == 0) leftEncoderValue--;
        else if (lastLeftEncoderState == 3) leftEncoderValue++;
      case 2:
        if (lastLeftEncoderState == 0) leftEncoderValue++;
        else if (lastLeftEncoderState == 3) leftEncoderValue--;
    }

    lastLeftEncoderState = leftEncoderState; //saves last encoder state for next change
    //save last pin states for pseudo interrupt
    oldlP1 = lP1;
    oldlP2 = lP2;
  }






  //get current right signal pin values
  rP1 = digitalRead(rightP1);
  rP2 = digitalRead(rightP2);

  if (rP1 != oldrP1 || rP2 != oldrP2) { //|| rP2 != orP2

    // change = 1;

    int rightEncoderState = rP1 * 2 + rP2; // 0 0 = 0, 0 1 = 1, 1 0 = 2, 1 1 = 3

    switch (rightEncoderState) { //basically increments the encoder value accordingly based on the change that happened
      case 0:
        if (lastRightEncoderState == 1) rightEncoderValue++;
        else if (lastRightEncoderState == 2) rightEncoderValue--;
      case 3:
        if (lastRightEncoderState == 2) rightEncoderValue++;
        else if (lastRightEncoderState == 1) rightEncoderValue--;
      case 1: 
        if (lastRightEncoderState == 0) rightEncoderValue--;
        else if (lastRightEncoderState == 3) rightEncoderValue++;
      case 2:
        if (lastRightEncoderState == 0) rightEncoderValue++;
        else if (lastRightEncoderState == 3) rightEncoderValue--;
    }

    lastRightEncoderState = rightEncoderState; //saves last encoder state for next change
    //save last pin states for pseudo interrupt
    oldrP1 = rP1;
    oldrP2 = rP2;
  }

  



  // if (change == 1) {
  Serial.println(leftEncoderValue);
  Serial.println(rightEncoderValue);

  // change = 0;
  // }
}
