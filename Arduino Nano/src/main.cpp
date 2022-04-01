#include <Arduino.h>
#include <Arduino_LSM6DS3.h>
#include <SNU.h>
// #include "music.h"

float calibratieZ;
float calibratieX;
float calibratieY;
unsigned long startTijd;
unsigned long stopTijd;
unsigned long wachtTijd;
unsigned long zitTijd;
unsigned long time;
bool semafoor = false;
bool majorEventSemafoor = false;

int buzzer = 3;
String message;

float X, Y, Z; //metingen op x-, y- en z-as

float drempelwaarde;
float majorDrempelwaarde;
float vermenigvuldigingswaarde;
int delayValue;

bool majorEventDetected;
enum States{
  // Als de metingen binnen de threshold blijft zitten, blijft het in REST.
  // Houdt de tijd op 30 minuten.
  // Zodra metingen buiten de threshold vallen, dan gaan we naar SITTING.
  // Start de tijd.
  // Semafoor wordt True.
  REST,
  // Wanneer SITTING wordt onderbroken voor x aantal sec dan wordt Semafoor False en gaat de state naar REST. 
  // Wanneer SITTING al 30 minuten actief is gaat het naar PROMPTING.
  SITTING,
  //wanneer er in sitting een "Major Event" word gedetecteerd gaan we hier kijken of de gebruiker is opgestaan.
  STANDING,
  // Bij PROMPTING stuur een bericht naar de user, bij reactie zet Semafoor naar False en reset Timer.
  PROMPTING
};

States state = REST;



void calibratie();
void getValues();
bool checkThreshold();
void sittingEntry();
void readtextfromkeyboard();
void getThresholdStatus();
bool checkMajorEventThreshold();
bool checkMeetwaardeProximity();

void setup() {
  pinMode(buzzer, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  Serial.begin(115200);
  // while (!Serial);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
  }

  vermenigvuldigingswaarde = 200;
  drempelwaarde = 3;
  majorDrempelwaarde = 15;
  semafoor = 0;
  majorEventSemafoor = 0;
  majorEventDetected = false;

  calibratie();
  delay(1000);
  tone(buzzer, 2900, 200);
  delay(500);
  tone(buzzer, 2900, 200);
  delay(100);
}

void loop() {
  getValues();
  getThresholdStatus();

  readtextfromkeyboard();
  if(message == "rest" || message == "r" || message == "R" || message == "REST"){
    state = REST;
    Serial.println("state switched to REST");
  }
  else if(message == "sitting" || message == "s" || message == "S" || message == "SITTING"){
    state = SITTING;
    Serial.println("state switched to SITTING");
  }
  else if(message == "prompting" || message == "p" || message == "P" || message == "PROMPTING"){
    state = PROMPTING;
    Serial.println("state switched to PROMPTING");
  } else if (message != ""){
    Serial.println("We don't recognize this command please check for spelling errors");
  }
  message = "";
  switch(state) 
  {
  case REST:
    majorEventSemafoor = false;
    delayValue = 250;
    digitalWrite(6, HIGH);
    digitalWrite(5, LOW);
    digitalWrite(4, LOW);
    stopTijd = millis();
    wachtTijd = millis();
    zitTijd = millis();
    time = millis();
    if(checkThreshold()) {
      state = SITTING;
    }
    break;
  case SITTING:
    sittingEntry();
    digitalWrite(6, LOW);
    digitalWrite(5, HIGH);
    digitalWrite(4, LOW);

    if(checkMajorEventThreshold() && checkThreshold()) {
      wachtTijd = millis();
    }
    if(checkMajorEventThreshold()) {
      state = STANDING;
      delayValue = 500;
    }
    if(checkThreshold() && millis() >= zitTijd + 1800000) {
      state = PROMPTING;
    }
    if (checkMeetwaardeProximity() && millis() >= wachtTijd + 300000) {
      state = REST;
    }
    break;
  case STANDING:
  if(!majorEventSemafoor) {
    wachtTijd = millis();
    majorEventSemafoor = 1;
  }
  delay(10000);

  if (checkMeetwaardeProximity() && millis() >= wachtTijd + 10000) {
    Serial.println("standing -> rest");
    state = REST;
  }
  if (checkThreshold()) {
    Serial.println("Standing -> sitting");
    state = SITTING;
  }
    break;
  case PROMPTING:
    // prompt
    digitalWrite(6, LOW);
    digitalWrite(5, LOW);
    digitalWrite(4, HIGH);
    Serial.println("PROMPTING:: Het is tijd om op te staan");
    semafoor = 0;
    majorEventSemafoor = 0;
    zitTijd = millis();
    wachtTijd = millis();
    tone(buzzer, 2900, 200);
    delay(500);
    tone(buzzer, 2900, 200);
    delay(500);
    tone(buzzer, 2900, 200);
    delay(500);
    tone(buzzer, 2900, 200);
    // music();
    delay(10000);
    state = REST;
    break;
  default:
    // exception block
    break;
  }
  delay(delayValue);
}

bool checkThreshold() 
{
  if (abs(X - calibratieX) > drempelwaarde || abs(Y - calibratieY) > drempelwaarde || abs(Z - calibratieZ) > drempelwaarde) {
    return true;
  }
  return false;
}

bool checkMajorEventThreshold() 
{
  if (abs(X - calibratieX) > majorDrempelwaarde || abs(Y - calibratieY) > majorDrempelwaarde || abs(Z - calibratieZ) > majorDrempelwaarde) {
    return true;
  }
  return false;
}

bool checkMeetwaardeProximity() 
{
  if ((abs(X - calibratieX) < 2 && abs(Y - calibratieY) < 2 && abs(Z - calibratieZ) < 2) && millis() >= wachtTijd + 10000) {
    return true;
  }
  return false;
}

void calibratie() {
  tone(buzzer, 2900, 300);
  calibratieX = 0;
  calibratieY = 0;
  calibratieZ = 0;
  float maxRuisZ = 0;
  float minRuisZ = 1000;
  for(int i = 0; i < 50; i++) {
    IMU.readAcceleration(X, Y, Z);
    maxRuisZ = max(Z, maxRuisZ);
    minRuisZ = min(Z, minRuisZ);
    X = X * vermenigvuldigingswaarde;
    Y = Y * vermenigvuldigingswaarde;
    Z = Z * vermenigvuldigingswaarde;
    calibratieX += X;
    calibratieY += Y;
    calibratieZ += Z;
    delay(250);
  };
  calibratieX = calibratieX / 50;
  calibratieY = calibratieY / 50;
  calibratieZ = calibratieZ / 50;
}

void getValues() {
 if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(X, Y, Z);
    X = X * vermenigvuldigingswaarde;
    Y = Y * vermenigvuldigingswaarde;
    Z = Z * vermenigvuldigingswaarde;
  }
  Serial.print("Meetwaarde: ");
  Serial.print(X);
  Serial.print(" ");
  Serial.print(Y);
  Serial.print(" ");
  Serial.println(Z);
  Serial.print("Calibratiewaarde: ");
  Serial.print(calibratieX);
  Serial.print(" ");
  Serial.print(calibratieY);
  Serial.print(" ");
  Serial.println(calibratieZ);
  Serial.print("Verschil: ");
  Serial.print(abs(calibratieX - X));
  Serial.print(" ");
  Serial.print(abs(calibratieY - Y));
  Serial.print(" ");
  Serial.println(abs(calibratieZ - Z));
  Serial.print("State: ");
  Serial.println(state);
}

void sittingEntry() {
  if(!semafoor) {
    zitTijd = millis();
    wachtTijd = millis();
    semafoor = !semafoor;
  }
}

void readtextfromkeyboard() {
 // Read serial input:
  String inString="";
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n' && (inString.length()) > 0) {
      message = inString;
      tone(3,1000,200);
    }
    else {
      inString += (char)inChar;
      delayMicroseconds(600);
    }
  }
}

void getThresholdStatus() {
  stopTijd = millis();
  if(checkThreshold()) {
    time = millis();
    Serial.print("Above threshold: ");
  } else {
    Serial.print("Under threshold: ");
  }
  Serial.println((stopTijd - wachtTijd) / 1000);
  Serial.println("------------------------------------------------------");
}