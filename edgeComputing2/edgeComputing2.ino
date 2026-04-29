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
  float roll;
  float pitch;
  uint32_t t_us;
  uint32_t proc_time_us;
} imu_packet_t;

typedef struct {
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

  Serial.println("Initialize PF");
  initPF();

  // PF on Core 1 (APP)
  xTaskCreatePinnedToCore(
    taskPF,
    "PF Task",
    20000,
    NULL,
    2,
    NULL,
    1
  );

  // UDP on Core 0 (PRO)
  xTaskCreatePinnedToCore(
    taskUDP,
    "UDP Task",
    4000,
    NULL,
    1,
    NULL,
    0
  );
  Serial.println("Program Start");
}

void loop() {
  // put your main code here, to run repeatedly:

}

void initPF() {
  for (int i = 0; i < N_PARTICLES; i++) {
    particles[i].roll  = ((float)random(-100,100)) * 0.01; 
    particles[i].pitch = ((float)random(-100,100)) * 0.01;
    particles[i].w = 1.0f / N_PARTICLES;
  }
}

void predict(float gx, float gy, float gz, float dt)
{
  const float noise = 0.002f;

  for (int i = 0; i < N_PARTICLES; i++) {
    particles[i].roll  += gx * dt + ((float)random(-100,100)/100.0f)*noise;
    particles[i].pitch += gy * dt + ((float)random(-100,100)/100.0f)*noise;
  }
}

void update(float ax, float ay, float az)
{
  // normalize accel ONCE
  float norm = sqrtf(ax*ax + ay*ay + az*az);
  if (norm < 1e-6) return;

  ax /= norm;
  ay /= norm;
  az /= norm;

  float sum = 0;

  for (int i = 0; i < N_PARTICLES; i++) {
    float r = particles[i].roll;
    float p = particles[i].pitch;

    // expected gravity vector
    float gx = -sinf(p);
    float gy = sinf(r) * cosf(p);
    float gz = cosf(r) * cosf(p);

    float dx = ax - gx;
    float dy = ay - gy;
    float dz = az - gz;

    float err = dx*dx + dy*dy + dz*dz;

    // Gaussian likelihood
    float w = expf(-err * 5.0f);  // tune "5.0"
    particles[i].w = w;
    sum += w;
  }

  // normalize weights
  if (sum < 1e-12) {
    // fallback (avoid collapse)
    for (int i = 0; i < N_PARTICLES; i++) {
      particles[i].w = 1.0f / N_PARTICLES;
    }
    return;
  }

  for (int i = 0; i < N_PARTICLES; i++) {
    particles[i].w /= sum;
  }
}

void resample()
{
  Particle newParticles[N_PARTICLES];

  float step = 1.0f / N_PARTICLES;
  float r = ((float)random(0,10000)/10000.0f) * step;

  float c = particles[0].w;
  int i = 0;

  for (int m = 0; m < N_PARTICLES; m++) {
    float U = r + m * step;

    while (U > c && i < N_PARTICLES-1) {
      i++;
      c += particles[i].w;
    }

    newParticles[m] = particles[i];
    newParticles[m].w = 1.0f / N_PARTICLES;
  }

  // copy back
  for (int k = 0; k < N_PARTICLES; k++) {
    particles[k] = newParticles[k];
  }
}

void estimate(float &roll, float &pitch)
{
  roll = 0;
  pitch = 0;

  for (int i = 0; i < N_PARTICLES; i++) {
    roll  += particles[i].roll  * particles[i].w;
    pitch += particles[i].pitch * particles[i].w;
  }
}

void taskPF(void *pvParameters)
{
  float ax, ay, az, gx, gy, gz;
  uint32_t last_t = micros();

  while (true)
  {
    IMU.update();
    IMU.getAccel(&accelData);
    IMU.getGyro(&gyroData);
    
    ax = accelData.accelX;
    ay = accelData.accelY;
    az = accelData.accelZ;
    gx = gyroData.gyroX;
    gy = gyroData.gyroY;
    gz = gyroData.gyroZ;

    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;

    uint32_t now = micros();
    float dt = (now - last_t) / 1e6;
    last_t = now;

    // PF time calculation
    uint32_t t0 = micros();

    // ---- PARTICLE FILTER ----
    predict(gx, gy, gz, dt);
    update(ax, ay, az);
    resample();

    float roll, pitch;
    estimate(roll, pitch);

    // PF time calculation
    uint32_t t1 = micros();
    uint32_t proc_time = t1 - t0;

    // ---- SHARE DATA ----
    if (xSemaphoreTake(dataMutex, portMAX_DELAY))
    {
      sharedData.roll = roll;
      sharedData.pitch = pitch;
      sharedData.t_us = now;
      sharedData.proc_time_us = proc_time;

      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(1); // ~1ms → ~1kHz loop
  }
}

void taskUDP(void *pvParameters)
{
  imu_packet_t pkt;

  while (true)
  {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY))
    {
      pkt.roll = sharedData.roll;
      pkt.pitch = sharedData.pitch;
      pkt.t_us = sharedData.t_us;
      pkt.proc_time_us = sharedData.proc_time_us;

      xSemaphoreGive(dataMutex);
    }

    udp.beginPacket(serverIP.c_str(), serverPort);
    udp.write((uint8_t*)&pkt, sizeof(pkt));
    udp.endPacket();

    vTaskDelay(10); // ~100 Hz send rate
  }
}