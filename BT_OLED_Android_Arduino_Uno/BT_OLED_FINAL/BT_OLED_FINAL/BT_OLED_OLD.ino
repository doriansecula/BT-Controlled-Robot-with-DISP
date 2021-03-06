#define VERSION    

// Demo setup:
// Button #1 controls pin #13 LED
// Button #4 toggle datafield display rate
// Button #5 configured as "push" button (momentary)
// Other buttons display demo message

// Arduino pin#2 to TX BlueTooth module
// Arduino pin#3 to RX BlueTooth module
// BT board is set @57600 bps
// better remove SoftSerial for PWM based projects


#include "SoftwareSerial.h"

#define    STX          0x02
#define    ETX          0x03
#define    ledPin       13
#define    SLOW         750                            // Datafields refresh rate (ms)
#define    FAST         250                             // Datafields refresh rate (ms)
//#define    SPEEDLEFT    160   //160
//#define    SPEEDRIGHT   128    //128
#define    LOWSPEED     100    //128



SoftwareSerial mySerial(4,2);

int SPEEDLEFT  = 130;  // normal speed
int SPEEDRIGHT = 130;  //

int MOTOR2_PIN1 = 3;
int MOTOR2_PIN2 = 5;
int MOTOR1_PIN1 = 6;
int MOTOR1_PIN2 = 9;

byte cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};                 // bytes received
byte buttonStatus = 0;                                  // first Byte sent to Android device
long previousMillis = 0;                                // will store last time Buttons status was updated
long sendInterval = SLOW;                               // interval between Buttons status transmission (milliseconds)
String displayStatus = "lol";                          // message to Android device

void setup()  {
  
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);

  Serial.begin(9600);
  mySerial.begin(115200);
  mySerial.print("$");
  mySerial.print("$");
  mySerial.print("$");
  delay(100);
  mySerial.println("U,9600,N");
  mySerial.begin(9600);  // Start bluetooth serial at 9600                            // 57600 = max value for softserial
  pinMode(ledPin, OUTPUT);
  Serial.println(VERSION);
  while (mySerial.available())  mySerial.read();        // empty RX buffer
}

void loop() {
  if (mySerial.available())  {                          // data received from smartphone
    delay(2);
    cmd[0] =  mySerial.read();
    if (cmd[0] == STX)  {
      int i = 1;
      while (mySerial.available())  {
        delay(1);
        cmd[i] = mySerial.read();
        if (cmd[i] > 127 || i > 7)                 break; // Communication error
        if ((cmd[i] == ETX) && (i == 2 || i == 7))   break; // Button or Joystick data
        i++;
      }
      if     (i == 2)          getButtonState(cmd[1]);  // 3 Bytes  ex: < STX "C" ETX >
      else if (i == 7)          getJoystickState(cmd);  // 6 Bytes  ex: < STX "200" "180" ETX >
    }
  }
  sendBlueToothData();
}

void sendBlueToothData()  {
  static long previousMillis = 0;
  long currentMillis = millis();
  if (currentMillis - previousMillis > sendInterval) {  // send data back to smartphone
    previousMillis = currentMillis;

    // Data frame transmitted back from Arduino to Android device:
    // < 0X02   Buttons state   0X01   DataField#1   0x04   DataField#2   0x05   DataField#3    0x03 >
    // < 0X02      "01011"      0X01     "120.00"    0x04     "-4500"     0x05  "Motor enabled" 0x03 >    // example

    mySerial.print((char)STX);                                             // Start of Transmission
    mySerial.print(getButtonStatusString());  mySerial.print((char)0x1);   // buttons status feedback
    mySerial.print(GetdataInt1());            mySerial.print((char)0x4);   // datafield #1
    mySerial.print(GetdataFloat2());          mySerial.print((char)0x5);   // datafield #2
    mySerial.print(displayStatus);                                         // datafield #3
    mySerial.print((char)ETX);                                             // End of Transmission
  }
}

String getButtonStatusString()  {
  String bStatus = "";
  for (int i = 0; i < 6; i++)  {
    if (buttonStatus & (B100000 >> i))      bStatus += "1";
    else                                  bStatus += "0";
  }
  return bStatus;
}

int GetdataInt1()  {              // Data dummy values sent to Android device for demo purpose
  static int i = -30;             // Replace with your own code
  i ++;
  if (i > 0)    i = -30;
  return i;
}

float GetdataFloat2()  {           // Data dummy values sent to Android device for demo purpose
  static float i = 50;             // Replace with your own code
  i -= .5;
  if (i < -50)    i = 50;
  return i;
}

void getJoystickState(byte data[8])    {
  int joyX = (data[1] - 48) * 100 + (data[2] - 48) * 10 + (data[3] - 48); // obtain the Int from the ASCII representation
  int joyY = (data[4] - 48) * 100 + (data[5] - 48) * 10 + (data[6] - 48);
  joyX = joyX - 200;                                                  // Offset to avoid
  joyY = joyY - 200;                                                  // transmitting negative numbers

  if (joyX < -100 || joyX > 100 || joyY < -100 || joyY > 100)     return; // commmunication error

  // Your code here ...
  Serial.print("Joystick position:  ");
  Serial.print(joyX);
  Serial.print(", ");
  Serial.println(joyY);


  if (joyY >= 90) {

    //md.setM1Speed(SPEED);
    //stopIfFault();
    //md.setM2Speed(-SPEED);
    //stopIfFault();
    
    go(SPEEDLEFT,SPEEDRIGHT);

    Serial.println("inainte");

  } else if (joyX == 0 && joyY == 0) {
    //md.setM1Speed(0);
    //stopIfFault();
    //md.setM2Speed(0);
    //stopIfFault();
    
    go(0,0);
    Serial.println("stop");

  }
  if (joyY < -90) {

    //md.setM1Speed(-SPEED);
    //stopIfFault();
    //md.setM2Speed(SPEED);
    //stopIfFault();
    go(-SPEEDLEFT,-SPEEDRIGHT);
    Serial.println("inapoi");

  } else if (joyX == 0 && joyY == 0) {
    //md.setM1Speed(0);
    //stopIfFault();
    //md.setM2Speed(0);
    //stopIfFault();
    go(0,0);
    Serial.println("stop");

  }

  if (joyX >= 90) {

    //md.setM1Speed(SPEED);
    //stopIfFault();
    //md.setM2Speed(SPEED);
    //stopIfFault();

    go(SPEEDLEFT,-SPEEDRIGHT);
    Serial.println("dreapta");

  } else if (joyX == 0 && joyY == 0) {
    //md.setM1Speed(0);
    //stopIfFault();
    //md.setM2Speed(0);
    //stopIfFault();
    go(0,0); 
    Serial.println("stop");
  }
  if (joyX < -90) {

    //md.setM1Speed(-SPEED);
    //stopIfFault();
    //md.setM2Speed(-SPEED);
    //stopIfFault();

    go(-SPEEDLEFT,SPEEDRIGHT);
    Serial.println("stanga");

  } else if (joyX == 0 && joyY == 0) {
    //md.setM1Speed(0);
    //stopIfFault();
    //md.setM2Speed(0);
    //stopIfFault();
    go(0,0);
    Serial.println("stop");
  }

}

void getButtonState(int bStatus)  {
  switch (bStatus) {
      // -----------------  BUTTON #1  -----------------------
    case 'A':
      buttonStatus |= B000001;        // ON
      Serial.println("\n** Button_1: ON **");
      // your code...
      displayStatus = "LED <ON>";
      Serial.println(displayStatus);
      digitalWrite(ledPin, HIGH);
      
      SPEEDLEFT  = 255; // High Speed
      SPEEDRIGHT = 255;
      break;
      
    case 'B':
      buttonStatus &= B111110;        // OFF
      Serial.println("\n** Button_1: OFF **");
      // your code...
      displayStatus = "LED <OFF>";
      Serial.println(displayStatus);
      digitalWrite(ledPin, LOW);

      SPEEDLEFT = 130;
      SPEEDRIGHT = 130;
     
      
      break;

      // -----------------  BUTTON #2  -----------------------
    case 'C':
      buttonStatus |= B000010;        // ON
      Serial.println("\n** Button_2: ON **");
      // your code...
      displayStatus = "Button2 <ON>";
      Serial.println(displayStatus);

      SPEEDLEFT  = 156; // Traction Control 
      SPEEDRIGHT = 168;
      
      break;
    case 'D':
      buttonStatus &= B111101;        // OFF
      Serial.println("\n** Button_2: OFF **");
      // your code...
      displayStatus = "Button2 <OFF>";
      Serial.println(displayStatus);

      SPEEDLEFT = 130;
      SPEEDRIGHT = 130;
      
      break;

      // -----------------  BUTTON #3  -----------------------
    case 'E':
      buttonStatus |= B000100;        // ON
      Serial.println("\n** Button_3: ON **");
      // your code...
      displayStatus = "Motor #1 enabled"; // Demo text message
      Serial.println(displayStatus);
      break;
    case 'F':
      buttonStatus &= B111011;      // OFF
      Serial.println("\n** Button_3: OFF **");
      // your code...
      displayStatus = "Motor #1 stopped";
      Serial.println(displayStatus);
      break;

      // -----------------  BUTTON #4  -----------------------
    case 'G':
      buttonStatus |= B001000;       // ON
      Serial.println("\n** Button_4: ON **");
      // your code...
      displayStatus = "Datafield update <FAST>";
      Serial.println(displayStatus);
      sendInterval = FAST;
      break;
    case 'H':
      buttonStatus &= B110111;    // OFF
      Serial.println("\n** Button_4: OFF **");
      // your code...
      displayStatus = "Datafield update <SLOW>";
      Serial.println(displayStatus);
      sendInterval = SLOW;
      break;

      // -----------------  BUTTON #5  -----------------------
    case 'I':           // configured as momentary button
      //      buttonStatus |= B010000;        // ON
      Serial.println("\n** Button_5: ++ pushed ++ **");
      // your code...
      displayStatus = "Button5: <pushed>";
      break;
      //   case 'J':
      //     buttonStatus &= B101111;        // OFF
      //     // your code...
      //     break;

      // -----------------  BUTTON #6  -----------------------
    case 'K':
      buttonStatus |= B100000;        // ON
      Serial.println("\n** Button_6: ON **");
      // your code...
      displayStatus = "Button6 <ON>"; // Demo text message
      break;
    case 'L':
      buttonStatus &= B011111;        // OFF
      Serial.println("\n** Button_6: OFF **");
      // your code...
      displayStatus = "Button6 <OFF>";
      break;
  }
  // ---------------------------------------------------------------
}

void go(int speedLeft, int speedRight) {
  if (speedLeft > 0) {
    analogWrite(MOTOR1_PIN1, speedLeft);
    analogWrite(MOTOR1_PIN2, 0);
  }
  else {
    analogWrite(MOTOR1_PIN1, 0);
    analogWrite(MOTOR1_PIN2, -speedLeft);
  }

  if (speedRight > 0) {
    analogWrite(MOTOR2_PIN1, speedRight);
    analogWrite(MOTOR2_PIN2, 0);
  } else {
    analogWrite(MOTOR2_PIN1, 0);
    analogWrite(MOTOR2_PIN2, -speedRight);
  }
}

void go2(int speedLeft, int speedRight) {
  
  if (speedLeft > 0) {
    analogWrite(MOTOR1_PIN1, speedLeft);
    analogWrite(MOTOR1_PIN2, 0);
  }
  else {
    analogWrite(MOTOR1_PIN1, 0);
    analogWrite(MOTOR1_PIN2, -speedLeft);
  }

  if (speedRight > 0) {
    analogWrite(MOTOR2_PIN1, speedRight);
    analogWrite(MOTOR2_PIN2, 0);
  } else {
    analogWrite(MOTOR2_PIN1, 0);
    analogWrite(MOTOR2_PIN2, -speedRight);
  }
}
