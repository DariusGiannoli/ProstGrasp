#include <Wire.h>
#include <mpu9250.h>

// Use the enum constant instead of 0x68
bfs::Mpu9250 imu(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);  // For 0x68
// OR
// bfs::Mpu9250 imu(&Wire, bfs::Mpu9250::I2C_ADDR_SEC);  // For 0x69

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  if (!imu.Begin()) {
    Serial.println("IMU initialization failed");
    while(1) {}
  }
  
  Serial.println("IMU initialized successfully");
}

void loop() {
  if (imu.Read()) {
    float accelX = imu.accel_x_mps2();
    float accelY = imu.accel_y_mps2();
    float accelZ = imu.accel_z_mps2();
    
    Serial.print("Accel X: "); Serial.println(accelX);
  }
  
  delay(100);
}