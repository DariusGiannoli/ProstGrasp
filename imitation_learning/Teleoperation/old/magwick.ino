#include <Wire.h>
#include <mpu9250.h>
#include <MadgwickAHRS.h>

// IMU 1 pins
#define SDA_PIN_1 19
#define SCL_PIN_1 18

// IMU 2 pins
#define SDA_PIN_2 17
#define SCL_PIN_2 16

// Built-in LED
#define LED_PIN 2

// Sample rate (Hz)
#define SAMPLE_RATE 100

// Create two IMU objects on different I2C buses
bfs::Mpu9250 imu1(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);   // First IMU
bfs::Mpu9250 imu2(&Wire1, bfs::Mpu9250::I2C_ADDR_PRIM);  // Second IMU

// Create Madgwick filters for both IMUs
Madgwick filter1;
Madgwick filter2;

// Timing variables
unsigned long lastUpdate = 0;
float deltaTime = 1.0 / SAMPLE_RATE;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize both I2C buses with custom pins
  Wire.begin(SDA_PIN_1, SCL_PIN_1);    // I2C bus 0
  Wire1.begin(SDA_PIN_2, SCL_PIN_2);   // I2C bus 1
  
  // Initialize IMU 1
  if (!imu1.Begin()) {
    Serial.println("IMU 1 initialization failed!");
    while(1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  Serial.println("IMU 1 initialized!");
  
  // Initialize IMU 2
  if (!imu2.Begin()) {
    Serial.println("IMU 2 initialization failed!");
    while(1) {
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
  }
  Serial.println("IMU 2 initialized!");
  
  // Initialize Madgwick filters
  filter1.begin(SAMPLE_RATE);
  filter2.begin(SAMPLE_RATE);
  
  digitalWrite(LED_PIN, HIGH);  // Turn LED on when both ready
  Serial.println("Both IMUs ready with Madgwick filter!");
  
  lastUpdate = millis();
}

void loop() {
  // Calculate time since last update
  unsigned long now = millis();
  deltaTime = (now - lastUpdate) / 1000.0;
  lastUpdate = now;
  
  // Read and process IMU 1
  if (imu1.Read()) {
    // Update Madgwick filter with IMU 1 data
    // Note: Gyro in degrees/s, Accel in m/s²
    filter1.updateIMU(
      imu1.gyro_x_radps() * 57.2958,  // Convert rad/s to deg/s
      imu1.gyro_y_radps() * 57.2958,
      imu1.gyro_z_radps() * 57.2958,
      imu1.accel_x_mps2(),
      imu1.accel_y_mps2(),
      imu1.accel_z_mps2()
    );
    
    // Get orientation from filter
    float roll1 = filter1.getRoll();
    float pitch1 = filter1.getPitch();
    float yaw1 = filter1.getYaw();
    
    Serial.println("=== IMU 1 (Filtered) ===");
    Serial.print("Roll: ");  Serial.print(roll1);
    Serial.print(" | Pitch: "); Serial.print(pitch1);
    Serial.print(" | Yaw: "); Serial.println(yaw1);
  }
  
  // Read and process IMU 2
  if (imu2.Read()) {
    // Update Madgwick filter with IMU 2 data
    filter2.updateIMU(
      imu2.gyro_x_radps() * 57.2958,  // Convert rad/s to deg/s
      imu2.gyro_y_radps() * 57.2958,
      imu2.gyro_z_radps() * 57.2958,
      imu2.accel_x_mps2(),
      imu2.accel_y_mps2(),
      imu2.accel_z_mps2()
    );
    
    // Get orientation from filter
    float roll2 = filter2.getRoll();
    float pitch2 = filter2.getPitch();
    float yaw2 = filter2.getYaw();
    
    Serial.println("=== IMU 2 (Filtered) ===");
    Serial.print("Roll: ");  Serial.print(roll2);
    Serial.print(" | Pitch: "); Serial.print(pitch2);
    Serial.print(" | Yaw: "); Serial.println(yaw2);
  }
  
  Serial.println("---");
  delay(10);  // ~100Hz update rate
}
