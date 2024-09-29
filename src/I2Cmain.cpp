// #include <Wire.h>
// #include <Adafruit_LSM6DSOX.h>

// Adafruit_LSM6DSOX sox;

// // Variables to track time and position
// float z_rotation = 0;  // Absolute Z rotation in degrees
// unsigned long lastUpdateTime;
// #define SDA_PIN 47  // Change this to your preferred SDA pin
// #define SCL_PIN 21  // 


// void setup() { 
//     Serial.begin(115200);
//     Wire.begin(SDA_PIN, SCL_PIN);
//     // Initialize I2C
//     while(!sox.begin_I2C(0x6A)){
//     Serial.println("Failed to find LSM6DSOX chip");
//     delay(10);
//     // while (1);
//     }
//     Serial.println("LSM6DSOX Found!");

//     // Set initial time
//     lastUpdateTime = millis();
// }

// void loop() {
//     sensors_event_t accel, gyro, temp;

//     delay(5);

//     // Get sensor events for accelerometer, gyroscope, and temperature
//     sox.getEvent(&accel, &gyro, &temp);

//     float z_angular_velocity_deg = gyro.gyro.z * 180.0 / PI +1.4;

//     Serial.print(">z_angular_velocity_deg:");
//     Serial.print(z_angular_velocity_deg);
//     Serial.print("\r\n");

// }





// // //old velocity calc from inside velPID keeping here for safekeeping
// //     // int M1_deltaEncPos = M1_encoderPos - prev_M1_enc_Pos;
// //     // int M2_deltaEncPos = M2_encoderPos - prev_M2_enc_Pos;

// //     // // Serial.print("time_Passed: ");
// //     // // Serial.println(time_Passed);

// //     // // Serial.print("M1_deltaEncPos: ");
// //     // // Serial.println(M1_deltaEncPos);

// //     // //set prev enc pos
// //     // prev_M1_enc_Pos = M1_encoderPos;
// //     // prev_M2_enc_Pos = M2_encoderPos;

// //     // //calculate vel and convert from enc ticks/ms to rotations/s
// //     // double M1_Vel = ((double)M1_deltaEncPos / time_Passed) * 1000 * 60 / enc_Ticks_Per_Rot;
// //     // double M2_Vel = ((double)M2_deltaEncPos / time_Passed) * 1000 * 60 / enc_Ticks_Per_Rot;


// //     // //saving vel for print and data logging
// //     // M1_Vel_Save = M1_Vel;
// //     // M2_Vel_Save = M2_Vel;


// //     //saving code for limiting PWM increases
// //         //if the robot is moving but is below 100 RPM limit pwm increases
// //     // double allowed_PWM_Change = 1;
// //     // int accelCuttoff = 100;
// //     // if(abs(M1_Rolling_Avg_Vel) < accelCuttoff && abs(M2_Rolling_Avg_Vel) < accelCuttoff){
// //     //   // Serial.println("triggered limiting PWM");
// //     //   double M1_PWM_Change = M1_PWM_Out - M1_Current_PWM;
// //     //   double M2_PWM_Change = M2_PWM_Out - M2_Current_PWM;
      
// //     //   if(abs(M1_PWM_Change) > allowed_PWM_Change){ //if PWM changes too much on M1
// //     //     if(M1_PWM_Out > 0 && M1_PWM_Change > 0){
// //     //       M1_PWM_Out = M1_Current_PWM + allowed_PWM_Change;
// //     //     }
// //     //     if(M1_PWM_Out < 0 && M1_PWM_Change < 0){
// //     //       M1_PWM_Out = M1_Current_PWM - allowed_PWM_Change;
// //     //     }
// //     //   }

// //     //   if(abs(M2_PWM_Change) > allowed_PWM_Change){ //if PWM changes too much on M1
// //     //     if(M2_PWM_Out > 0 && M2_PWM_Change > 0){
// //     //       M2_PWM_Out = M2_Current_PWM + allowed_PWM_Change;
// //     //     }
// //     //     if(M2_PWM_Out < 0 && M2_PWM_Change < 0){
// //     //       M2_PWM_Out = M2_Current_PWM - allowed_PWM_Change;
// //     //     }
// //     //   }
// //     // }
















// // #include <Wire.h>

// // #define SDA_PIN 47  // Your SDA pin
// // #define SCL_PIN 21  // Your SCL pin

// // void setup() {
// //   Wire.begin(SDA_PIN, SCL_PIN);
// //   Serial.begin(115200);
// //   Serial.println("\nI2C Scanner");
// // }

// // void loop() {
// //   byte error, address;
// //   int nDevices = 0;

// //   Serial.println("Scanning...");

// //   for (address = 1; address < 127; address++) {
// //     Wire.beginTransmission(address);
// //     error = Wire.endTransmission();

// //     if (error == 0) {
// //       Serial.print("I2C device found at address 0x");
// //       if (address < 16)
// //         Serial.print("0");
// //       Serial.print(address, HEX);
// //       Serial.println(" !");
// //       nDevices++;
// //     } else if (error == 4) {
// //       Serial.print("Unknown error at address 0x");
// //       if (address < 16)
// //         Serial.print("0");
// //       Serial.println(address, HEX);
// //     }
// //   }
// //   if (nDevices == 0)
// //     Serial.println("No I2C devices found\n");
// //   else
// //     Serial.println("done\n");

// //   delay(5000);
// // }
