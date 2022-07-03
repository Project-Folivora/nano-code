
#include <Arduino_LSM6DS3.h>
#include <SNU.h>

float x, y, z;
int X, Y, Z;
int caliX, caliY, caliZ;
int multiplication = 256;
int acceleration;
int vertical;
int verticalCalibration;
int verticalValue;
int ArrayXYZ [3];
int velocity;
int velocityTimer;

void calibration();
void values();
void printValues();

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
  }
  calibration();
}

void loop() {
  values();
  printValues();
  delay(25);
}

void calibration() {
  caliX = 0;
  caliY = 0;
  caliZ = 0;
  for(int i = 0; i < 64; i++) {
    IMU.readAcceleration(x, y, z);
    X = int(x * multiplication);
    Y = int(y * multiplication);  
    Z = int(z * multiplication);
    caliX += X;
    caliY += Y;
    caliZ += Z;
    delay(250);
  };
  caliX = caliX / 64;
  caliY = caliY / 64;
  caliZ = caliZ / 64;

  // bepaling van de grootste waarde
  Serial.print("de grootste waarde is: ");
  if(abs(X) >= abs(Y) && abs(X) >= abs(Z)) {vertical = 0; verticalCalibration = X; Serial.println("X");}
  if(abs(Y) >= abs(X) && abs(Y) >= abs(Z)) {vertical = 1; verticalCalibration = Y; Serial.println("Y");}
  if(abs(Z) >= abs(X) && abs(Z) >= abs(Y)) {vertical = 2; verticalCalibration = Z; Serial.println("Z");}
}

void values() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    X = int(x * multiplication);
    Y = int(y * multiplication);  
    Z = int(z * multiplication);

    //getting the vertical value;
    ArrayXYZ[0] = X;
    ArrayXYZ[1] = Y;
    ArrayXYZ[2] = Z;
    verticalValue = ArrayXYZ[vertical];
  }

    //acceleration
    acceleration = (verticalValue - verticalCalibration);
    if (acceleration <= 3 && acceleration >= -3) {
      acceleration = 0;
    }

    if(acceleration == 0) {velocityTimer++;} 
    else { velocityTimer = 0; }

    if (velocityTimer >= 25) {
      velocity = 0;
    }
    
}

void printValues() {
    Serial.println("-------------------------");
    Serial.println("Values in G:");
    Serial.print(X);
    Serial.print('\t');
    Serial.print(Y);
    Serial.print('\t');
    Serial.println(Z);
    Serial.println("Acceleration:");
    Serial.println(acceleration);
    Serial.println("Velocity:");
    Serial.println(velocity);
    Serial.println("-------------------------");
  }