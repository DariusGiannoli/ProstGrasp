#include <Wire.h>
#include <mpu9250.h>
#include <MadgwickAHRS.h>

// IMU 1 pins (Wrist - moving sensor)
#define SDA_PIN_1 19
#define SCL_PIN_1 18

// IMU 2 pins (Forearm - reference sensor)
#define SDA_PIN_2 17
#define SCL_PIN_2 16

// Flex sensor
#define FLEX_PIN 34

// Built-in LED
#define LED_PIN 2

// Sample rate (Hz)
#define SAMPLE_RATE 100

// Create two IMU objects on different I2C buses
bfs::Mpu9250 imu1(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);   // Wrist
bfs::Mpu9250 imu2(&Wire1, bfs::Mpu9250::I2C_ADDR_PRIM);  // Forearm (reference)

// Create Madgwick filters for both IMUs
Madgwick filter1;  // Wrist
Madgwick filter2;  // Forearm (reference)

// Output angles
float roll1, pitch1;  // Wrist (IMU 1)
float roll2, pitch2;  // Forearm reference (IMU 2)
int flexValue = 0;

// Timing
unsigned long lastUpdate = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(FLEX_PIN, INPUT);
  
  // Initialize both I2C buses with custom pins
  Wire.begin(SDA_PIN_1, SCL_PIN_1);    // I2C bus 0 - Wrist
  Wire1.begin(SDA_PIN_2, SCL_PIN_2);   // I2C bus 1 - Forearm
  
  delay(100);
  
  // Initialize IMU 1 (Wrist)
  if (!imu1.Begin()) {
    Serial.println("ERROR: IMU 1 (wrist) initialization failed!");
    while(1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  
  // Initialize IMU 2 (Forearm - Reference)
  if (!imu2.Begin()) {
    Serial.println("ERROR: IMU 2 (forearm) initialization failed!");
    while(1) {
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
  }
  
  // Initialize Madgwick filters
  filter1.begin(SAMPLE_RATE);
  filter2.begin(SAMPLE_RATE);
  
  digitalWrite(LED_PIN, HIGH);
  
  delay(1000);
  Serial.println("READY");
  
  lastUpdate = micros();
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastUpdate) / 1000000.0f;
  lastUpdate = now;
  
  // === Process IMU 1 (Wrist) ===
  if (imu1.Read()) {
    // Update Madgwick filter
    filter1.updateIMU(
      imu1.gyro_x_radps() * 57.2958,  // Convert rad/s to deg/s
      imu1.gyro_y_radps() * 57.2958,
      imu1.gyro_z_radps() * 57.2958,
      imu1.accel_x_mps2(),
      imu1.accel_y_mps2(),
      imu1.accel_z_mps2()
    );
    
    // Get orientation
    roll1 = filter1.getRoll();
    pitch1 = filter1.getPitch();
  }
  
  // === Process IMU 2 (Forearm - Reference) ===
  if (imu2.Read()) {
    // Update Madgwick filter
    filter2.updateIMU(
      imu2.gyro_x_radps() * 57.2958,  // Convert rad/s to deg/s
      imu2.gyro_y_radps() * 57.2958,
      imu2.gyro_z_radps() * 57.2958,
      imu2.accel_x_mps2(),
      imu2.accel_y_mps2(),
      imu2.accel_z_mps2()
    );
    
    // Get orientation
    roll2 = filter2.getRoll();
    pitch2 = filter2.getPitch();
  }
  
  // Read flex sensor
  flexValue = analogRead(FLEX_PIN);
  
  // Output format: roll1,pitch1,roll2,pitch2,flex
  // This matches the original format for Python processing
  Serial.print(roll1, 2);
  Serial.print(",");
  Serial.print(pitch1, 2);
  Serial.print(",");
  Serial.print(roll2, 2);
  Serial.print(",");
  Serial.print(pitch2, 2);
  Serial.print(",");
  Serial.println(flexValue);
  
  // Maintain ~100Hz update rate
  delayMicroseconds(10000 - (micros() - now));
}