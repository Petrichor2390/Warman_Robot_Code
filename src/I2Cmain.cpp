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





//old velocity calc from inside velPID keeping here for safekeeping
    // int M1_deltaEncPos = M1_encoderPos - prev_M1_enc_Pos;
    // int M2_deltaEncPos = M2_encoderPos - prev_M2_enc_Pos;

    // // Serial.print("time_Passed: ");
    // // Serial.println(time_Passed);

    // // Serial.print("M1_deltaEncPos: ");
    // // Serial.println(M1_deltaEncPos);

    // //set prev enc pos
    // prev_M1_enc_Pos = M1_encoderPos;
    // prev_M2_enc_Pos = M2_encoderPos;

    // //calculate vel and convert from enc ticks/ms to rotations/s
    // double M1_Vel = ((double)M1_deltaEncPos / time_Passed) * 1000 * 60 / enc_Ticks_Per_Rot;
    // double M2_Vel = ((double)M2_deltaEncPos / time_Passed) * 1000 * 60 / enc_Ticks_Per_Rot;


    // //saving vel for print and data logging
    // M1_Vel_Save = M1_Vel;
    // M2_Vel_Save = M2_Vel;