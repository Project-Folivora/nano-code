#include <Arduino.h>
#include <SimpleKalmanFilter.h>
#include <Arduino_LSM6DS3.h>
#include <SNU.h>
// #include "music.h"

bool semafoor = false;
bool majorEventSemafoor = false;
float calibratieZ;
float calibratieX;
float calibratieY;
float calibratieGyroscopeZ;
float calibratieGyroscopeX;
float calibratieGyroscopeY;
float ArrayXYZ [3];
unsigned long startTijd;
unsigned long stopTijd;
unsigned long wachtTijd;
unsigned long zitTijd;
unsigned long time;
String message;
int buzzer = 3;

float X, Y, Z; //metingen op x-, y- en z-as
float gyroX, gyroY, gyroZ; // metingen op de x-, y- en z-as van de gyroscope
int vertical;
float verticalWaarde;
float verticalCalibratie;
float drempelwaarde;
float majorDrempelwaarde;
float vermenigvuldigingswaarde;
int delayValue;
float versnelling;

float vtold;
float vt;
float vttotaal;

float stold;
float st;
float stotaal;

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
//Kalman Filter
SimpleKalmanFilter simpleKalmanFilter(0.05, 1, 0.1);
const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

// --------------Function Declarations---------------------

States state = REST;

bool checkThreshold();
bool checkMajorEventThreshold();
bool checkMeetwaardeProximity();
void calibratie();
void getValues();
void sittingEntry();
void readtextfromkeyboard();
void getThresholdStatus();
void sensorLightRest();
void sensorLightSitting();
void sensorLightPrompting();
void standingEntry();
void plotterDebugCommands();
void restEntry();
void calibratieNotificatie();
void setupConfiguratie();
void loopReset();
void presentieGebruiker();
void restExit();
void promptingExit();
void sittingExit();
void checkStanding();
void piepjesMaker(int aantal);
void getGyroscopeValues();
void kalmanWaardes();
void verplaatsingBerekenen();

// ------------------------------Setup Configuration----------------------------------


void setup() {
  setupConfiguratie();
  calibratie();
  calibratieNotificatie();
}


// -----------------------------Main Loop-----------------------------------


void loop() {
  getValues();
  // getGyroscopeValues();
  verplaatsingBerekenen();
  getThresholdStatus();
  readtextfromkeyboard();
  plotterDebugCommands();
  kalmanWaardes();
 
  // ?
  message = "";

  switch(state) 
  {
  case REST:

    restEntry();
    sensorLightRest();
    restExit();

    break;
  case SITTING:

    sittingEntry();
    sensorLightSitting();
    checkStanding();
    sittingExit();

    break;
  case STANDING:
    standingEntry();
    presentieGebruiker();

    break;
  case PROMPTING:

    sensorLightPrompting();
    loopReset();
    piepjesMaker(4);
    promptingExit();

    break;
  default:
    // exception block
    break;
  }
  delay(delayValue);
}


// -----------------------------Functions----------------------------------------


// Checkt of de meetwaarde de drempelwaarde doorbreekt om van REST naar SITTING te gaan.
bool checkThreshold() 
{
  if (abs(verticalWaarde - verticalCalibratie) > drempelwaarde) {
    return true;
  }
  return false;
}

// Doet hetzelfde als checkThreshold maar de drempelwaarde is groter om STANDING te detecteren.
bool checkMajorEventThreshold() 
{
  if (abs(verticalWaarde - verticalCalibratie) > majorDrempelwaarde) {
    return true;
  }
  return false;
}

// ?
bool checkMeetwaardeProximity() 
{
  if (abs(verticalWaarde - verticalCalibratie)  < 1) {
    return true;
  }
  return false;
}

// Calibratie procedure van de sensor.
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
    calibratieX += X;
    calibratieY += Y;
    calibratieZ += Z;
    delay(250);
  };
  calibratieX = calibratieX / 50;
  calibratieY = calibratieY / 50;
  calibratieZ = calibratieZ / 50;

  // bepaling van de grootste waarde
  Serial.print("de grootste waarde is: ");
  if(abs(X) >= abs(Y) && abs(X) >= abs(Z)) {vertical = 0; verticalCalibratie = X; Serial.println("X");}
  if(abs(Y) >= abs(X) && abs(Y) >= abs(Z)) {vertical = 1; verticalCalibratie = Y; Serial.println("Y");}
  if(abs(Z) >= abs(X) && abs(Z) >= abs(Y)) {vertical = 2; verticalCalibratie = Z; Serial.println("Z");}
}

// Haalt de waardes op van de accelerometer.
void getValues() {
 if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(X, Y, Z);
    ArrayXYZ[0] = X;
    ArrayXYZ[1] = Y;
    ArrayXYZ[2] = Z;
    // Serial.println(ArrayXYZ[vertical]);
    verticalWaarde = ArrayXYZ[vertical];
  }
  // Serial.print("Meetwaarde: ");
  // Serial.print(X);
  // Serial.print(" ");
  // Serial.print(Y);
  // Serial.print(" ");
  // Serial.println(Z);
  // Serial.print("Calibratiewaarde: ");
  // Serial.print(calibratieX);
  // Serial.print(" ");
  // Serial.print(calibratieY);
  // Serial.print(" ");
  // Serial.println(calibratieZ);
  // Serial.print("Verschil: ");
  // Serial.print(abs(calibratieX - X));
  // Serial.print(" ");
  // Serial.print(abs(calibratieY - Y));
  // Serial.print(" ");
  // Serial.println(abs(calibratieZ - Z));
  // Serial.print("State: ");
  // Serial.println(state);
}
// void getGyroscopeValues() {
//   if (IMU.gyroscopeAvailable()) {
//     IMU.readGyroscope(gyroX, gyroY, gyroZ);
//   }
//   Serial.print("Gyroscope Meetwaarde: ");
//   Serial.print(gyroX);
//   Serial.print(" ");
//   Serial.print(gyroY);
//   Serial.print(" ");
//   Serial.println(gyroZ);
// }

// Checkt of de gebruiker voor het eerst is in een case cyclus zit.
// Alle Entries zetten en/of resetten de semaforen om te checken voor first occurence.
void sittingEntry() {
  // Semafoor wordt getriggered.
  if(!semafoor) {
    zitTijd = millis();
    wachtTijd = millis();
    semafoor = true;
    piepjesMaker(2);
  }
  // tijd wordt gestart
  stopTijd = millis();
  wachtTijd = millis();
  time = millis();
}
void restEntry(){
  semafoor = 0;
  majorEventSemafoor = 0;
  delayValue = 250;
  zitTijd = millis();
}
void standingEntry(){
  if(!majorEventSemafoor) {
    wachtTijd = millis();
    majorEventSemafoor = 1;
    piepjesMaker(3);
  }
  // Deze delay zit erin om bewegingen DIRECT na de major Event te negeren.
  delay(10000);
}

// Zorgt ervoor dat devs via de plotter commando's kunnen geven aan de sensor. 
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

// Geeft de status van de sensor weer in de plotter.
void getThresholdStatus() {
  // stopTijd = millis();
  // if(checkThreshold()) {
  //   time = millis();
  //   Serial.print("Above threshold: ");
  // } else {
  //   Serial.print("Under threshold: ");
  // }
  // Serial.println((stopTijd - zitTijd) / 1000);
}

// Deze lampen geven de STATES weer voor de gebruiker.
void sensorLightRest(){
  digitalWrite(6, HIGH);
  digitalWrite(5, LOW);
  digitalWrite(4, LOW);
}
void sensorLightSitting(){
  digitalWrite(6, LOW);
  digitalWrite(5, HIGH);
  digitalWrite(4, LOW);
}
void sensorLightPrompting(){
  digitalWrite(6, LOW);
  digitalWrite(5, LOW);
  digitalWrite(4, HIGH);
}

// Configuratie voor de plotter commands.
void plotterDebugCommands(){
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
}

// geeft de gebruiker een notificatie dat de calibratie.
void calibratieNotificatie(){
  delay(1000);
  tone(buzzer, 2900, 200);
  delay(500);
  tone(buzzer, 2900, 200);
  delay(100);
}

// setup configuratie voor de hardware, semaforen, drempelwaardes en ?
void setupConfiguratie(){
  // hardware configuratie
  pinMode(buzzer, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  // baudrate configuratie.
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
}

// Reset de cyclus door de semaforen en tijd te resetten.
void loopReset(){
    Serial.println("PROMPTING:: Het is tijd om op te staan");
    semafoor = 0;
    majorEventSemafoor = 0;
    zitTijd = millis();
    wachtTijd = millis();
}

// checkt of de gebruiker zit op de stoel of niet.
// afhankelijk daarvan switched de state van STANDING naar REST of SITTING.
void presentieGebruiker(){
  // Checkt of de gebruiker afwezig is na de majorEvent.
  if (checkMeetwaardeProximity()) {
    Serial.println("standing -> rest");
    state = REST;
  }

  // Checkt of de gebruiker aanwezig is na de majorEvent.
  if (checkThreshold()) {
    Serial.println("Standing -> sitting");
    state = SITTING;
  }
}

// Als de threshold wordt overschreden gaat state van REST naar SITTING.
// Alle exit functions geven state switches aan.
void restExit(){
  if(checkThreshold()) {
    state = SITTING;
  }
}
void promptingExit(){
  delay(10000);
  state = REST;
}
void sittingExit(){
  // Na 30 minuten gaat de state van 
  if(millis() >= zitTijd + 1800000) {
    state = PROMPTING;
  }
}

// Zodra er een majorEvent is gedetecteerd gaat de state van SITTING naar STANDING.
void checkStanding(){
  if(checkMajorEventThreshold() && checkThreshold()) {
    wachtTijd = millis();
  }
  if(checkMajorEventThreshold()) {
    state = STANDING;
    delayValue = 500;
  }
}

//buzzer functie
void piepjesMaker(int aantal) {
  for (int i = 0; i < aantal; i++) {
    tone(buzzer, 2900, 200);
    delay(500); 
  }
}

void kalmanWaardes() {
  // float estimated_value = simpleKalmanFilter.updateEstimate(Z);
  // // Serial.print("Kalman Waardes: ");
  // Serial.print(estimated_value);
  // Serial.print(",");
  // Serial.println(Z);
  Serial.println("--------------------------------------");
}

void verplaatsingBerekenen() {
versnelling = (verticalWaarde - verticalCalibratie);

vtold = vt;
vt = vtold + (versnelling * 0.25);
vttotaal = vtold += vt;

stold = st;
st = (vtold * 0.25) + (versnelling * 0.03125);
stotaal = stold += st;


Serial.println(stotaal * 100);
}