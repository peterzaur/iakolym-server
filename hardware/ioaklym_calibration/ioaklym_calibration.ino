#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_LSM9DS0.h>

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

/* Assign a unique ID to these sensors */
Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);

float GyroMinX, GyroMaxX;
float GyroMinY, GyroMaxY;
float GyroMinZ, GyroMaxZ;

float AccelMinX, AccelMaxX;
float AccelMinY, AccelMaxY;
float AccelMinZ, AccelMaxZ;

float MagMinX, MagMaxX;
float MagMinY, MagMaxY;
float MagMinZ, MagMaxZ;

long lastDisplayTime;

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void setup(void) 
{
  Serial.println("LSM9DS0 Calibration"); Serial.println("");
  
  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM9DS0 detected ... Check your wiring!");
    while(1);
  }
  
  /* Setup the sensor gain and integration time */
  configureSensor();

  lastDisplayTime = millis();
  Serial.begin(9600);
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t accel, mag, gyro, temp; 
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  
  if (gyro.gyro.x < GyroMinX) GyroMinX = gyro.gyro.x;
  if (gyro.gyro.x > GyroMaxX) GyroMaxX = gyro.gyro.x;
  
  if (gyro.gyro.y < GyroMinY) GyroMinY = gyro.gyro.y;
  if (gyro.gyro.y > GyroMaxY) GyroMaxY = gyro.gyro.y;

  if (gyro.gyro.z < GyroMinZ) GyroMinZ = gyro.gyro.z;
  if (gyro.gyro.z > GyroMaxZ) GyroMaxZ = gyro.gyro.z;

  if (accel.acceleration.x < AccelMinX) AccelMinX = accel.acceleration.x;
  if (accel.acceleration.x > AccelMaxX) AccelMaxX = accel.acceleration.x;
  
  if (accel.acceleration.y < AccelMinY) AccelMinY = accel.acceleration.y;
  if (accel.acceleration.y > AccelMaxY) AccelMaxY = accel.acceleration.y;

  if (accel.acceleration.z < AccelMinZ) AccelMinZ = accel.acceleration.z;
  if (accel.acceleration.z > AccelMaxZ) AccelMaxZ = accel.acceleration.z;

  if (mag.magnetic.x < MagMinX) MagMinX = mag.magnetic.x;
  if (mag.magnetic.x > MagMaxX) MagMaxX = mag.magnetic.x;
  
  if (mag.magnetic.y < MagMinY) MagMinY = mag.magnetic.y;
  if (mag.magnetic.y > MagMaxY) MagMaxY = mag.magnetic.y;

  if (mag.magnetic.z < MagMinZ) MagMinZ = mag.magnetic.z;
  if (mag.magnetic.z > MagMaxZ) MagMaxZ = mag.magnetic.z;

  if ((millis() - lastDisplayTime) > 1000)  // display once/second
  {
    Serial.print("Gyro Minimums: "); Serial.print(GyroMinX); Serial.print("  ");Serial.print(GyroMinY); Serial.print("  "); Serial.print(GyroMinZ); Serial.println();
    Serial.print("Gyro Maximums: "); Serial.print(GyroMaxX); Serial.print("  ");Serial.print(GyroMaxY); Serial.print("  "); Serial.print(GyroMaxZ); Serial.println();
    Serial.print("Accel Minimums: "); Serial.print(AccelMinX); Serial.print("  ");Serial.print(AccelMinY); Serial.print("  "); Serial.print(AccelMinZ); Serial.println();
    Serial.print("Accel Maximums: "); Serial.print(AccelMaxX); Serial.print("  ");Serial.print(AccelMaxY); Serial.print("  "); Serial.print(AccelMaxZ); Serial.println();
    Serial.print("Mag Minimums: "); Serial.print(MagMinX); Serial.print("  ");Serial.print(MagMinY); Serial.print("  "); Serial.print(MagMinZ); Serial.println();
    Serial.print("Mag Maximums: "); Serial.print(MagMaxX); Serial.print("  ");Serial.print(MagMaxY); Serial.print("  "); Serial.print(MagMaxZ); Serial.println(); Serial.println();
    lastDisplayTime = millis();
  }
}
