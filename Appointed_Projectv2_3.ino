#include <SoftwareSerial.h>
#include <Wire.h>
#include "ServoControl.h"
#include "MotorControl.h"
#include "GravityTDS.h"
#include "LiquidCrystal_I2C.h"

#define TDSPIN A0
#define PUMP A1
#define firstPin 4
#define secondPin 5
#define thirdPin 6

// Instantiate objects
ServoControl servoControl;
MotorControl motorControl;
GravityTDS gravityTds;
LiquidCrystal_I2C lcd(0x27,16,2); // LCD
SoftwareSerial mySerial(2, 3);


// const String phoneNumber = "+639998206762";
const String phoneNumber = "+639625412588";

int prevTemp = 0, temperature = 25;
int potCounter = 0;
float prevTDS = 0, tdsValue = 0;
int centerSensor;
int leftSensor;
int rightSensor;
bool isOnBlackLine = false;
int displaySegment = 3;

////non-blocking delay
//unsigned long prevMillis = 0
//const long LCDInterval = 2000;

String baseMessage = "The reading is: ";

int speed = 130;  // Adjust this value to reduce the speed (range: 0-255)

void setupTDS() {
  gravityTds.setPin(TDSPIN);
  gravityTds.setAref(5.0);
  gravityTds.setAdcRange(1024);
  gravityTds.begin();
  gravityTds.setTemperature(temperature);
  gravityTds.update();
}

void updateLCD()
{
  if (prevTemp == temperature){
    lcd.setCursor(0,0);
    lcd.print("Temperature:");
    lcd.print(temperature);
    lcd.print("C");
    prevTemp = temperature;
  }
  if (prevTDS == tdsValue){
    lcd.setCursor(0,1);
    lcd.print("Gravity:");
    lcd.print(tdsValue);
    lcd.print("ppm");
    prevTDS = tdsValue;
  }
}

void readTDS() {
  tdsValue = gravityTds.getTdsValue();
}

void sendSMS(String message) {
  mySerial.print("AT+CMGS=\"");
  mySerial.print(phoneNumber);
  mySerial.println("\"");
  updateSerial();
  mySerial.print(message);
  updateSerial();
  mySerial.write(26);
}

void updateSerial() {
  delay(500);
  while (Serial.available()) {
    mySerial.write(Serial.read());
  }
  while (mySerial.available()) {
    Serial.write(mySerial.read());
  }
}

void displayBCD(int number) {

  if (number < 0 || number > 99) {
    return;
  }

  switch (number) {
    case 0:
      digitalWrite(firstPin, LOW);
      digitalWrite(secondPin, LOW);
      digitalWrite(thirdPin, LOW);
      break;
    case 1:
      digitalWrite(firstPin, HIGH);
      digitalWrite(secondPin, LOW);
      digitalWrite(thirdPin, LOW);
      break;
    case 2:
      digitalWrite(firstPin, LOW);
      digitalWrite(secondPin, HIGH);
      digitalWrite(thirdPin, LOW);
      break;
    case 3:
      digitalWrite(firstPin, HIGH);
      digitalWrite(secondPin, HIGH);
      digitalWrite(thirdPin, LOW);
      break;
    case 4:
      digitalWrite(firstPin, LOW);
      digitalWrite(secondPin, LOW);
      digitalWrite(thirdPin, HIGH);
      break;
    case 5:
      digitalWrite(firstPin, HIGH);
      digitalWrite(secondPin, LOW);
      digitalWrite(thirdPin, HIGH);
      break;
    case 6:
      digitalWrite(firstPin, LOW);
      digitalWrite(secondPin, HIGH);
      digitalWrite(thirdPin, HIGH);
      break;
    case 7:
      digitalWrite(firstPin, HIGH);
      digitalWrite(secondPin, HIGH);
      digitalWrite(thirdPin, HIGH);
      break;
    default:
      digitalWrite(firstPin, LOW);
      digitalWrite(secondPin, LOW);
      digitalWrite(thirdPin, LOW);
      break;
  }
}

void activatePump() {
  digitalWrite(PUMP, LOW);  // Pump on
  delay(3000);
  digitalWrite(PUMP, HIGH);  // Pump off
  delay(100);
}



void setup() {
  Serial.begin(9600);
  setupTDS();
  servoControl.setupServos();
  motorControl.setupMotors();

  mySerial.begin(115200);
  mySerial.println("AT");
  updateSerial();
  mySerial.println("AT+CMGF=1");
  updateSerial();

  pinMode(PUMP, OUTPUT);
  digitalWrite(PUMP, HIGH);  // Pump off

  pinMode(secondPin, OUTPUT);
  pinMode(firstPin, OUTPUT);
  pinMode(thirdPin, OUTPUT);

  displayBCD(3);
  lcd.init();
  lcd.backlight();
}

void loop() {
  //unsigned long currentTime = millis();
  //if (currentTime - prevMillis >= LCDInterval){
  updateLCD();
    //prevMillis = currentTime;
  //}
  if (potCounter < 3) {
    motorControl.initSensors(centerSensor, leftSensor, rightSensor);

    if (centerSensor == HIGH && leftSensor == HIGH && rightSensor == HIGH && isOnBlackLine == false) {
      motorControl.moveStop();

      servoControl.moveToActivePosition();
      delay(5000);
      readTDS();
      String msg = baseMessage + String(tdsValue);
      activatePump();
      sendSMS(msg);
      servoControl.moveToSleepPosition();

      potCounter++;

      int result = displaySegment - potCounter;
      displayBCD(result);
      isOnBlackLine = true;

      // if (potCounter < 3) {
      //   motorControl.moveForward();
      //   delay(1000);
      // }
    } else {
      // Line-following logic
      if (centerSensor == LOW && leftSensor == HIGH && rightSensor == LOW) {
        // Turn left
        motorControl.moveLeft(speed);
        isOnBlackLine = false;
      } else if (centerSensor == LOW && leftSensor == LOW && rightSensor == HIGH) {
        // Turn right
        motorControl.moveRight(speed);
        isOnBlackLine = false;
      } else if (centerSensor == HIGH && leftSensor == LOW && rightSensor == LOW) {
        // Move forward
        motorControl.moveForward(speed);
        isOnBlackLine = false;
      } else {
        // No line detected, stop or search for line
        motorControl.moveForward(speed);
      }
    }
  } else {
    motorControl.moveStop();
  }

  // Servo control based on serial commands
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.equalsIgnoreCase("active")) {
      servoControl.moveToActivePosition();
    } else if (input.equalsIgnoreCase("sleep")) {
      servoControl.moveToSleepPosition();
    }
  }
}
