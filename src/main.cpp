#include <Arduino.h>
#include <Arduino_LSM6DS3.h>

float calibratieZ;
float calibratieX;
float calibratieY;
unsigned long startTijd;
unsigned long stopTijd;
unsigned long wachtTijd;
unsigned long zitTijd;
bool semafoor = false;

float X, Y, Z; //metingen op x-, y- en z-as

float drempelwaarde;
float vermenigvuldigingswaarde;

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
  // Bij PROMPTING stuur een bericht naar de user, bij reactie zet Semafoor naar False en reset Timer.
  PROMPTING
};

States state = REST;



void calibratie();
void getValues();
bool checkThreshold();
void sittingEntry();


void setup() {
  Serial.begin(115200);
  while (!Serial);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
  }

  vermenigvuldigingswaarde = 200;
  drempelwaarde = 5;


  calibratie();

  Serial.print("CALIBRATIE WAARDE");
  Serial.print(calibratieX);
  Serial.print('\t');
  Serial.print(calibratieY);
  Serial.print('\t');
  Serial.println(calibratieZ);
  delay(100);
}

void loop() {
  getValues();

    switch(state) 
  {
  case REST:
    digitalWrite(13, HIGH);
    stopTijd = millis();
    wachtTijd = millis();
    zitTijd = millis();
    if(checkThreshold()) {
      digitalWrite(13, LOW);
      digitalWrite(12, HIGH);
      state = SITTING;
    }
    break;
  case SITTING:
    sittingEntry();
    Serial.print(X);
    Serial.print(" ");
    Serial.print(Y);
    Serial.print(" ");
    Serial.println(Z);
    if (!checkThreshold() && millis() >= wachtTijd + 6000) {
      digitalWrite(13, HIGH);
      digitalWrite(12, LOW);
      state = REST;
      Serial.println("Sitting --> Rest");
      calibratie();
    }
    if(checkThreshold() && millis() >= zitTijd + 12000) {
      digitalWrite(11, HIGH);
      digitalWrite(12, LOW);
      state = PROMPTING;
      Serial.println("Sitting --> Prompting");
      calibratie();
    }
    break;
  case PROMPTING:
    // prompt
    digitalWrite(11, LOW);
    digitalWrite(13, HIGH);
    Serial.println("PROMPTING:: Het is tijd om op te staan");
    semafoor = !semafoor;
    calibratie();
    delay(10000);
    state = REST;
    break;
  default:
    // exception block
    break;
  }
  delay(250);
}

bool checkThreshold() 
{
  if (Z > (calibratieZ + drempelwaarde) || Z < (calibratieZ - drempelwaarde) 
  || X > calibratieX + drempelwaarde || X < calibratieX - drempelwaarde 
  || Y > calibratieY + drempelwaarde || Y < calibratieY - drempelwaarde) {
    return true;
  }
  return false;
}

void calibratie() {
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
  };
  Serial.print(maxRuisZ);
  Serial.println(minRuisZ);
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
    
    Serial.print(X);
    Serial.print('\t');
    Serial.print(Y);
    Serial.print('\t');
    Serial.println(Z);
  }
}

void sittingEntry() {
  if(!semafoor) {
    zitTijd = millis();
    wachtTijd = millis();
    Serial.print("SITTING ENTRY ");
    semafoor = !semafoor;
  }
  if(checkThreshold()) {
    wachtTijd = millis();
    calibratie();
  } 
  if (!checkThreshold()) {
    calibratie();
  }
}
