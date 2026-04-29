#include "FastIMU.h"
#include <Wire.h>
#include "WiFi.h"
#include <WiFiUdp.h>
#include <math.h>

#define IMU_ADDRESS 0x68    
#define PERFORM_CALIBRATION 
#define N_PARTICLES 1000
#define DEG_TO_RAD 0.01745329252f

MPU6500 IMU;               
WiFiUDP udp;

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;

typedef struct {
  float ax, ay, az;
  float gx, gy, gz;
  uint32_t t_us;
} imu_packet_t;

typedef struct {
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch;
  uint32_t t_us;
  uint32_t proc_time_us;   
} imu_shared_t;

imu_shared_t sharedData;
SemaphoreHandle_t dataMutex;

struct Particle {
  float roll;
  float pitch;
  float w;
};

Particle particles[N_PARTICLES];

String ssid = "Archer";
String pass = "Ymst_SKZ46";
String serverIP = "10.13.2.135";
int serverPort = 5005;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); 

  WiFi.begin(ssid, pass);
  udp.begin(5005);

  dataMutex = xSemaphoreCreateMutex();

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  
#ifdef PERFORM_CALIBRATION
  Serial.println("FastIMU calibration");

  delay(5000);
  Serial.println("Keep IMU level.");
  delay(5000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  delay(1000);
  IMU.init(calib, IMU_ADDRESS);
#endif

  
  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }


  Serial.println("Program Start");
}

void loop() {
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);
  
  float gx = gyroData.gyroX;
  float gy = gyroData.gyroY;
  float gz = gyroData.gyroZ;
  
  imu_packet_t pkt;

  pkt.ax = accelData.accelX;
  pkt.ay = accelData.accelY;
  pkt.az = accelData.accelZ;

  gx *= DEG_TO_RAD;
  gy *= DEG_TO_RAD;
  gz *= DEG_TO_RAD;

  pkt.gx = gx;
  pkt.gy = gy;
  pkt.gz = gz;

  pkt.t_us = micros();

  udp.beginPacket(serverIP.c_str(), serverPort);
  udp.write((uint8_t*)&pkt, sizeof(pkt));
  udp.endPacket();
}
