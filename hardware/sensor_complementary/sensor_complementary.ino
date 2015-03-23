#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_LSM9DS0.h>
#include <RunningMedian.h>

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11
#define mySerial Serial1                                                        // RX, TX for BlueTooth

const int analogInPin = A9;                                                     // Use pin 9
int veloValue = 0;                                                              // Velostat value normally
int onVeloValue = 0;                                                            // Velostat when stepped on
int LED = 7;

float deltat = 0.0f;                                                            // integration interval for both filter schemes
uint32_t lastUpdate = 0;                                                        // for integration interval
uint32_t Now = 0;                                                               // for integration interval
float swingTimer = 0.0f;                                                        // counter for swing time

int velo_OC;                                                                    // Velostat offset calibration
float gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z;   // Variables to hold latest sensor data values
float gx_OC, gy_OC, gz_OC;                                                      // Gyro offset calibration value
float acc_roll;                                                                 // Roll measurement from Accelerometer
float acc_pitch;                                                                // Pitch measurement from Accelerometer
float gyro_yaw;                                                                 // Yaw Measurement from Gyro
float mag_yaw;                                                                  // Yaw Measurement from Accelerometer
float tot_yaw;                                                                  // Complementary filtered Yaw measurement

// Magnetometer hard and soft iron compensation - needs to run the calibration script
// Reference: http://www.camelsoftware.com/firetail/blog/uavs/3-axis-magnetometer-calibration-a-simple-technique-for-hard-soft-errors/
float MagMinX, MagMaxX;
float MagMinY, MagMaxY;
float MagMinZ, MagMaxZ;
float magtot_avg;
float magx_scale;
float magy_scale;
float magz_scale;

Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

RunningMedian X_RunningMedian = RunningMedian(10);
RunningMedian Y_RunningMedian = RunningMedian(10);
RunningMedian Z_RunningMedian = RunningMedian(10);
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
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

/**************************************************************************/
/*
    Gyro offset calibration and Magnetometer hard / soft iron compensation
*/
/**************************************************************************/

void offsetCal(){
  Serial.println("Calibrating gyroscope... don't touch or move shit");
  
  float gx = 0.0, gy = 0.0, gz = 0.0, i;
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp); 
  
  gx = gyro.gyro.x;
  gy = gyro.gyro.y;
  gz = gyro.gyro.z;
  
  for (i=1; i<50; i++){
    gx = (gx + gyro.gyro.x)/2;
    gy = (gy + gyro.gyro.y)/2;
    gz = (gz + gyro.gyro.z)/2;
    Serial.print(".");
    delay(100);
  }
  Serial.println(".");
  
  gx_OC = gx;
  gy_OC = gy;
  gz_OC = gz;
  
  Serial.print("gx register offset = ");
  Serial.println(gx);
  
  Serial.print("gy register offset = ");
  Serial.println(gy);
  
  Serial.print("gz register offset = ");
  Serial.println(gz);
  
  Serial.println("Calibrating Magnetometer... move shit");
  while (millis() < 75000) {
    if (mag.magnetic.x < MagMinX) MagMinX = mag.magnetic.x;
    if (mag.magnetic.x > MagMaxX) MagMaxX = mag.magnetic.x;
    
    if (mag.magnetic.y < MagMinY) MagMinY = mag.magnetic.y;
    if (mag.magnetic.y > MagMaxY) MagMaxY = mag.magnetic.y;
  
    if (mag.magnetic.z < MagMinZ) MagMinZ = mag.magnetic.z;
    if (mag.magnetic.z > MagMaxZ) MagMaxZ = mag.magnetic.z;
    Serial.print(".");
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
  }
  Serial.println(".");
  
  magtot_avg = ((MagMaxX - ((MagMinX + MagMaxX)*0.5) + (-1*(MagMinX - ((MagMinX + MagMaxX)*0.5))))*0.5 + \
                (MagMaxY - ((MagMinY + MagMaxY)*0.5) + (-1*(MagMinY - ((MagMinY + MagMaxY)*0.5))))*0.5 + \
                (MagMaxZ - ((MagMinZ + MagMaxZ)*0.5) + (-1*(MagMinZ - ((MagMinZ + MagMaxZ)*0.5))))*0.5)/3;
  magx_scale = magtot_avg/((MagMaxX - ((MagMinX + MagMaxX)*0.5) + (-1*(MagMinX - ((MagMinX + MagMaxX)*0.5))))*0.5);
  magy_scale = magtot_avg/((MagMaxY - ((MagMinY + MagMaxY)*0.5) + (-1*(MagMinY - ((MagMinY + MagMaxY)*0.5))))*0.5);
  magz_scale = magtot_avg/((MagMaxZ - ((MagMinZ + MagMaxZ)*0.5) + (-1*(MagMinZ - ((MagMinZ + MagMaxZ)*0.5))))*0.5);
  
  Serial.print("Magnetometer X Scale = ");
  Serial.println(magx_scale);
  
  Serial.print("Magnetometer Y Scale = ");
  Serial.println(magy_scale);
  
  Serial.print("Magnetometer Z Scale = ");
  Serial.println(magz_scale);
  
  
}

/**************************************************************************/
/*
    Velostat offset calibration
*/
/**************************************************************************/

void offsetVelo(){
  Serial.println("Calibrating velostat... don't touch or move shit");
  
  float velo, i;
  
  velo = analogRead(analogInPin);
  
  for (i=1; i<100; i++){
    velo = (velo + analogRead(analogInPin))/2;
    Serial.print(".");
    delay(100);
  }
  Serial.println(".");
  
  velo_OC = round(velo);

  Serial.print("Velostat register offset = ");
  Serial.println(velo);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup()
{
  Serial.begin(14400);
//  mySerial.begin(9600);
  
  pinMode(9, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  
  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  offsetVelo();
  offsetCal();
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop()
{  
  // Instantiate sensor objects
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp); 
  
  // Calculate roll and pitch using accelerometer measurements
  acc_roll = atan2(accel.acceleration.y, accel.acceleration.z) * 180 / PI;
  if (accel.acceleration.y * sin(acc_roll) + accel.acceleration.z * cos(acc_roll) == 0)
    acc_pitch = accel.acceleration.x > 0 ? (PI / 2) : (-PI / 2);
  else
    acc_pitch = atan(-accel.acceleration.x / (accel.acceleration.y * sin(acc_roll) + accel.acceleration.z * cos(acc_roll)));
  
  // Getting magnetometer measurements
  mag_x = mag.magnetic.x;
  mag_y = mag.magnetic.y;
  mag_z = mag.magnetic.z;
  
  // Hard iron compensation
  mag_x -= (MagMinX + MagMaxX)*0.5;
  mag_y -= (MagMinY + MagMaxY)*0.5;
  mag_z -= (MagMinZ + MagMinZ)*0.5;
  
  // Soft iron compensation
  mag_x *= magx_scale;
  mag_y *= magy_scale;
  mag_z *= magz_scale;
  
  // Calculating magnetometer yaw angle
  mag_yaw = atan2(mag_z * sin(acc_roll) - mag_y * cos(acc_roll), \
                  mag_x * cos(acc_pitch) + \
                  mag_y * sin(acc_pitch) * sin(acc_roll) + \
                  mag_z * sin(acc_pitch) * cos(acc_roll));
 
  // Convert magnetometer yaw into degrees                  
  mag_yaw = (mag_yaw * 180.0f / PI) + 180.0f;
  
  // Getting z-axis rate of rotation
  Z_RunningMedian.add(gyro.gyro.z);
  
  // Apply offset (based on if it's -ve, +ve, or 0)
  if (gz_OC < 0.0f) {
    gyro_z = Z_RunningMedian.getMedian() + (-1.0f * gz_OC);
  } else if (gz_OC == 0.0f) {
    gyro_z = Z_RunningMedian.getMedian();
  } else {
    gyro_z = Z_RunningMedian.getMedian() - gz_OC;
  }
  
  // Use complementary filter for gyro / magnetometer yaw
  tot_yaw = (0.9f) * (tot_yaw + gyro_z * deltat) + (0.1f) * (mag_yaw);
  
  // Get the time in milliseconds (this goes back to 0 after 50 days apparently)
  Now = millis();
  
  // Apply offset to velostat measurement
  if (velo_OC < 0) {
    veloValue = analogRead(analogInPin) + (-1 * velo_OC);
  } else if (velo_OC == 0) {
    veloValue = analogRead(analogInPin);
  } else {
    veloValue = analogRead(analogInPin) - velo_OC;
  }
  
  // If step (threshold is currently set to -30, depends on the initial velostat value is
  //
  // todo, potential: Find out one-time, what the low value is after a step (ie. -15 or -20)
  // then base it off that
  // potential solution to that: nest if below for a certain amount of time 
  // if condition is currVeloValue - stepVeloValue < -x
  // set stepVeloValue to currVeloValue, if it's lower than it
  // stop condition when it's a difference of -15 or -20 or something that is beyond drifting limits
  if (veloValue < -90) {
    // onVeloValue not being used?
    onVeloValue = veloValue;
    deltat = ((Now - lastUpdate) / 1000.0f);
    lastUpdate = Now;
    mySerial.print("Foot_angle: ");
    mySerial.print(tot_yaw);
    mySerial.print(", Stride_time: ");
    mySerial.print(deltat);
    mySerial.print(", Swing_time: ");
    mySerial.print(swingTimer);
    mySerial.print(", Stance_time: ");
    mySerial.print(deltat - swingTimer);
    mySerial.println("");
    swingTimer = 0.0;
  } else {
    swingTimer = swingTimer + 0.1;
  }
  
  delay(100);
}
