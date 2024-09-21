// #include <Wire.h>
// #include <Adafruit_LSM6DSOX.h>

// Adafruit_LSM6DSOX sox;

// // Variables to track time and position
// float z_rotation = 0;  // Absolute Z rotation in degrees
// unsigned long lastUpdateTime;

// void setup() {
//   Serial.begin(115200);
//   // Initialize I2C
//   if (!sox.begin_I2C()) {
//     Serial.println("Failed to find LSM6DSOX chip");
//     while (1);
//   }
//   Serial.println("LSM6DSOX Found!");

//   // Set initial time
//   lastUpdateTime = millis();
// }

// void loop() {
//   sensors_event_t accel, gyro, temp;

//   // Get sensor events for accelerometer, gyroscope, and temperature
//   sox.getEvent(&accel, &gyro, &temp);

//   // Calculate time elapsed since the last update (in seconds)
//   unsigned long currentTime = millis();
//   float deltaTime = (currentTime - lastUpdateTime) / 1000.0;  // convert ms to seconds
//   lastUpdateTime = currentTime;

//   // Update the absolute Z rotation based on the gyroscope's Z axis (angular velocity in rad/s)
//   float z_angular_velocity_deg = gyro.gyro.z * 180.0 / PI + 0.35;
//   if(z_angular_velocity_deg > 0.15 || z_angular_velocity_deg < -0.15 ){
//     if(lastUpdateTime > 1000){ //first second of readings seem to be shit
//       z_rotation += z_angular_velocity_deg * deltaTime;  // θ = θ0 + ω * t
//     }
//   }


//   Serial.println(z_rotation);

// }