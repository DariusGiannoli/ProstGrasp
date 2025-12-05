#include <Wire.h>
#include <mpu9250.h>

// IMU 1 pins
#define SDA_PIN_1 19
#define SCL_PIN_1 18

// IMU 2 pins
#define SDA_PIN_2 17
#define SCL_PIN_2 16

// Built-in LED
#define LED_PIN 2

// Create two IMU objects on different I2C buses
bfs::Mpu9250 imu1(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);   // First IMU
bfs::Mpu9250 imu2(&Wire1, bfs::Mpu9250::I2C_ADDR_PRIM);  // Second IMU

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
  
  digitalWrite(LED_PIN, HIGH);  // Turn LED on when both ready
  Serial.println("Both IMUs ready!");
}

void loop() {
  // Read IMU 1
  if (imu1.Read()) {
    Serial.println("=== IMU 1 ===");
    Serial.print("Accel X: "); Serial.print(imu1.accel_x_mps2());
    Serial.print(" | Y: "); Serial.print(imu1.accel_y_mps2());
    Serial.print(" | Z: "); Serial.println(imu1.accel_z_mps2());
    
    Serial.print("Gyro X: "); Serial.print(imu1.gyro_x_radps());
    Serial.print(" | Y: "); Serial.print(imu1.gyro_y_radps());
    Serial.print(" | Z: "); Serial.println(imu1.gyro_z_radps());
  }
  
  // Read IMU 2
  if (imu2.Read()) {
    Serial.println("=== IMU 2 ===");
    Serial.print("Accel X: "); Serial.print(imu2.accel_x_mps2());
    Serial.print(" | Y: "); Serial.print(imu2.accel_y_mps2());
    Serial.print(" | Z: "); Serial.println(imu2.accel_z_mps2());
    
    Serial.print("Gyro X: "); Serial.print(imu2.gyro_x_radps());
    Serial.print(" | Y: "); Serial.print(imu2.gyro_y_radps());
    Serial.print(" | Z: "); Serial.println(imu2.gyro_z_radps());
  }
  
  Serial.println("---");
  delay(500);
}