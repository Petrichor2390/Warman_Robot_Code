#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <PID_v1.h>

//ENCODERs###################################################################
//M1
#define M1_ENCODER_PIN_A 15 // Encoder pin A for motor 1
#define M1_ENCODER_PIN_B 16 // Encoder pin B for motor 1
//M2
#define M2_ENCODER_PIN_A 17 // Encoder pin A for motor 2
#define M2_ENCODER_PIN_B 18 // Encoder pin B for motor 2

//encoder variables that carry over through interrupts
//M1
volatile int M1_encoderPos = 0;
volatile bool M1_lastA = LOW;
volatile bool M1_lastB = LOW;
//M2
volatile int M2_encoderPos = 0;
volatile bool M2_lastA = LOW;
volatile bool M2_lastB = LOW;

//DC MOTOR CONTROL #############################################################
//M1
#define M1_IN1 13
#define M1_IN2 12
#define M1_EN 14 //ENA

//M2
#define M2_IN1 10 //on motor controller this is IN3
#define M2_IN2 9 //on motor controller this in IN4
#define M2_EN 11//ENB

//ACTION buton
#define ACTION_BUTTON_PIN 40
int action_ButtonState = LOW;
int action_LastButtonState = 0;
//debounce varaibles
unsigned long action_LastDebounceTime = 0;
unsigned long action_DebounceDelay = 50;

//characteristic motor variables
int enc_Ticks_Per_Rot = 961;
const int TRANSLATE_MAX_SPEED = 300; //RPM for moving motors
//max acceleration for motors - post PID
double max_Acceleration = 100; //max change in PWM value output to the motors
double low_Speed_Acceleration = 100;
double low_Speed_Cutoff_ForAcceleration_Top = 100;
double low_Speed_Cutoff_ForAcceleration_Bottom = 0;

//Velocity PID ##########################################################
long int prev_Vel_PID_Time = micros();
int prev_M1_enc_Pos = 0;
int prev_M2_enc_Pos = 0;

double M1_Vel_input, M1_Vel_output, M1_Vel_setpoint;
double M2_Vel_input, M2_Vel_output, M2_Vel_setpoint;
double Vel_Kp = 0.3, Vel_Ki = 0.8, Vel_Kd = 0.0; //0.15 //i0.8 //p 1.5 //d 0.05

PID M1_Vel_PID(&M1_Vel_input, &M1_Vel_output, &M1_Vel_setpoint, Vel_Kp, Vel_Ki, Vel_Kd, DIRECT);
PID M2_Vel_PID(&M2_Vel_input, &M2_Vel_output, &M2_Vel_setpoint, Vel_Kp, Vel_Ki, Vel_Kd, DIRECT);

double M1_Vel_Save;
double M2_Vel_Save;

double M1_Prev_PWM_Output = 0;
double M2_Prev_PWM_Output = 0;

double M1_Current_PWM = 0;
double M2_Current_PWM = 0;


//rolling average velocity
const int roll_N = 10;
double M1_Rolling_Avg_Vel_Sum = 0;
double M2_Rolling_Avg_Vel_Sum = 0;
double M1_Rolling_Avg_Vel_Values[roll_N];
double M2_Rolling_Avg_Vel_Values[roll_N];
double M1_Rolling_Avg_Vel = 0;
double M2_Rolling_Avg_Vel = 0;
int roll_Index = 0;
bool roll_Avg_Filled = false;


//Position PID################################################################# 
//input = position from encoders, output = setpoint for Velocity PID (range from -300 to 300), setpoint = targets on the track
double M1_Pos_input, M1_Pos_output, M1_Pos_setpoint;
double M2_Pos_input, M2_Pos_output, M2_Pos_setpoint;
double Pos_Kp = 0.35, Pos_Ki = 0, Pos_Kd = 0.0; //p0.15

PID M1_Pos_PID(&M1_Pos_input, &M1_Pos_output, &M1_Pos_setpoint, Pos_Kp, Pos_Ki, Pos_Kd, DIRECT);
PID M2_Pos_PID(&M2_Pos_input, &M2_Pos_output, &M2_Pos_setpoint, Pos_Kp, Pos_Ki, Pos_Kd, DIRECT);


//temporary variables used for testing purposes
int loopNo = 0;
double testTargetVelPID = 150;
double testTargetPOSPID = 961*5;
bool testingVelPID = false; //true if testing vel pid false if running position PIDs
long int brakeTime = 3500; //2 seconds after the button is pressed brake
long int buttonPressTime = 0;

//M1 interupts
void IRAM_ATTR M1handleEncoderA() {
  bool newA = digitalRead(M1_ENCODER_PIN_A);
  bool newB = digitalRead(M1_ENCODER_PIN_B);

  if (newA != M1_lastA) {
    if (newA == newB) {
      M1_encoderPos--;  // CW rotation
    } else {
      M1_encoderPos++;  // CCW rotation
    }
    M1_lastA = newA;
  }
}

void IRAM_ATTR M1handleEncoderB() {
  bool newA = digitalRead(M1_ENCODER_PIN_A);
  bool newB = digitalRead(M1_ENCODER_PIN_B);

  if (newB != M1_lastB) {
    if (newA != newB) {
      M1_encoderPos--;  // CW rotation
    } else {
      M1_encoderPos++;  // CCW rotation
    }
    M1_lastB = newB;
  }
}

//M2 Interrupts
void IRAM_ATTR M2handleEncoderA() {
  bool newA = digitalRead(M2_ENCODER_PIN_A);
  bool newB = digitalRead(M2_ENCODER_PIN_B);

  if (newA != M2_lastA) {
    if (newA == newB) {
      M2_encoderPos++;  // CW rotation
    } else {
      M2_encoderPos--;  // CCW rotation
    }
    M2_lastA = newA;
  }
}

void IRAM_ATTR M2handleEncoderB() {
  bool newA = digitalRead(M2_ENCODER_PIN_A);
  bool newB = digitalRead(M2_ENCODER_PIN_B);

  if (newB != M2_lastB) {
    if (newA != newB) {
      M2_encoderPos++;  // CW rotation
    } else {
      M2_encoderPos--;  // CCW rotation
    }
    M2_lastB = newB;
  }
}


void printPID(bool bypass = false){
    if(bypass){
      Serial.print(">M1_Vel:");
      Serial.print(M1_Vel_Save);
      Serial.print(",Target:");
      Serial.print(testTargetVelPID);
      Serial.print(",Output:");
      Serial.print(M1_Current_PWM);
      Serial.print("\r\n");
    }else{
      Serial.print(">M1_Vel:");
      Serial.print(M1_Vel_Save);
      Serial.print(",POSPID Target:");
      Serial.print(testTargetPOSPID/10);
      Serial.print(",POSPID_Out:");
      Serial.print(M1_Pos_output);
      Serial.print(",M1encPos:");
      Serial.print(M1_encoderPos/10);
      Serial.print(",M2encPos:");
      Serial.print(M2_encoderPos/10);
      Serial.print(",M1PWM:");
      Serial.print(M1_Vel_output);





      Serial.print("\r\n");
    }
}

void brakeMotors(){
  // digitalWrite(M1_IN1, LOW);
  // digitalWrite(M1_IN2, LOW);
  // digitalWrite(M1_IN1, LOW);
  // digitalWrite(M1_IN2, LOW);
  analogWrite(M1_EN, 0);
  analogWrite(M2_EN, 0);

  delay(5000);
}

void DCMotorTest(){
  analogWrite(M1_EN, 100);
  analogWrite(M2_EN, 100);

  int del = 10000;
  
  //M1
  Serial.println("M1 FORWARDS");
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);
  delay(del);
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);

  Serial.println("M1 BACKWARDS");
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, HIGH);
  delay(del);
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);

  //M2
  Serial.println("M2 FORWARDS");
  digitalWrite(M2_IN1, HIGH);
  digitalWrite(M2_IN2, LOW);
  delay(del);
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);

  Serial.println("M1 BACKWARDS");
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, HIGH);
  delay(del);
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
}


void PosPIDCalculation(bool bypass = false){

  if(!bypass){ //standard operation
    //set Pos PID input to encoder values
    M1_Pos_input = M1_encoderPos;
    M2_Pos_input = M2_encoderPos;

    //compute PID output
    M1_Pos_PID.Compute();
    M2_Pos_PID.Compute();

    //set Vel PID setpoint to Pos PID output
    M1_Vel_setpoint = M1_Pos_output;
    M2_Vel_setpoint = M2_Pos_output;
  }else{
    //used for testing velocity PID
    M1_Vel_setpoint = testTargetVelPID;
    M2_Vel_setpoint = testTargetVelPID;
  }
}

void VelPIDCalculation(){
  //velocuty PID
  //calculate time passed
  long int current_Time = micros();
  double time_Passed = (current_Time - prev_Vel_PID_Time)/1000; //convert from micros to millis

  //avoid div 0 and too small increments
  if (time_Passed < 5) {
    return; 
  }else{
    PosPIDCalculation(testingVelPID); //bypass = true - skipping for vel pid calc

    prev_Vel_PID_Time = current_Time;

    int M1_deltaEncPos = M1_encoderPos - prev_M1_enc_Pos;
    int M2_deltaEncPos = M2_encoderPos - prev_M2_enc_Pos;

    // Serial.print("time_Passed: ");
    // Serial.println(time_Passed);

    // Serial.print("M1_deltaEncPos: ");
    // Serial.println(M1_deltaEncPos);

    //set prev enc pos
    prev_M1_enc_Pos = M1_encoderPos;
    prev_M2_enc_Pos = M2_encoderPos;

    //calculate vel and convert from enc ticks/ms to rotations/s
    double M1_Vel = ((double)M1_deltaEncPos / time_Passed) * 1000 * 60 / enc_Ticks_Per_Rot;
    double M2_Vel = ((double)M2_deltaEncPos / time_Passed) * 1000 * 60 / enc_Ticks_Per_Rot;


    //saving vel for print and data logging
    M1_Vel_Save = M1_Vel;
    M2_Vel_Save = M2_Vel;
    
    //calculate pid output values
    M1_Vel_input = M1_Vel;
    M2_Vel_input = M2_Vel;
    M1_Vel_PID.Compute();
    M2_Vel_PID.Compute();

    //logic for setting max increase in PWM values
    //choosing between max acceleration and low acceleration
    // double local_Max_Acceleration = 0;
    // if((abs(M1_Current_PWM) < low_Speed_Cutoff_ForAcceleration_Top) && (abs(M1_Current_PWM) > low_Speed_Cutoff_ForAcceleration_Bottom)){
    //   local_Max_Acceleration = low_Speed_Acceleration;
    // }else{
    //   local_Max_Acceleration = max_Acceleration;
    // }

    // //M1
    // if(abs(M1_Vel_output - M1_Current_PWM) > local_Max_Acceleration){
    //   if(M1_Vel_output > M1_Current_PWM){
    //     M1_Current_PWM += local_Max_Acceleration;
    //   }else{
    //     M1_Current_PWM -= local_Max_Acceleration;
    //   }
    // }else{
    //   M1_Current_PWM = M1_Vel_output;
    // }

    // //M2
    // if(abs(M2_Vel_output - M2_Current_PWM) > local_Max_Acceleration){
    //   if(M2_Vel_output > M2_Current_PWM){
    //     M2_Current_PWM += local_Max_Acceleration;
    //   }else{
    //     M2_Current_PWM -= local_Max_Acceleration;
    //   }
    // }else{
    //   M2_Current_PWM = M2_Vel_output;
    // }

    // PWM deadband filter
    // double deadbandPWM = 25;
    // double noiseAllowance = 5;
    // if(M1_Current_PWM > noiseAllowance || M1_Current_PWM < -noiseAllowance){
    //   if(M1_Current_PWM > 0 && M1_Current_PWM < deadbandPWM){
    //     M1_Current_PWM = deadbandPWM;
    //   }
    //   if(M1_Current_PWM < 0 && M1_Current_PWM > -deadbandPWM){
    //     M1_Current_PWM = -deadbandPWM;
    //   }
    // }
    // if(M2_Current_PWM > noiseAllowance || M2_Current_PWM < -noiseAllowance){
    //   if(M2_Current_PWM > 0 && M2_Current_PWM < deadbandPWM){
    //     M2_Current_PWM = deadbandPWM;
    //   }
    //   if(M2_Current_PWM < 0 && M2_Current_PWM > -deadbandPWM){
    //     M2_Current_PWM = -deadbandPWM;
    //   }
    // }

    //M1 motor controll logic
    if (M1_Vel_output > 0) {
      digitalWrite(M1_IN1, HIGH);
      digitalWrite(M1_IN2, LOW);
    } else if (M1_Vel_output < 0) {
      digitalWrite(M1_IN1, LOW);
      digitalWrite(M1_IN2, HIGH);
    }
    // Set motor speed (output should be PWM signal to ENA)
    analogWrite(M1_EN, int(abs(M1_Vel_output)));
      
    //M2 motor control logic
    if (M2_Vel_output > 0) {
      digitalWrite(M2_IN1, HIGH);
      digitalWrite(M2_IN2, LOW);
    } else if (M2_Vel_output < 0) {
      digitalWrite(M2_IN1, LOW);
      digitalWrite(M2_IN2, HIGH);
    }
    // Set motor speed (output should be PWM signal to ENA)
    analogWrite(M2_EN, int(abs(M2_Vel_output)));

    // Serial.print("M1 output");
    // Serial.println(M1_Vel_output);

    // delay(100);
    printPID(testingVelPID);
    // delay(5);
  }
}



void actionButtonWait(){
  //wait for the button to be pressed
  bool pressed = false;
  while(!pressed){

    int reading = digitalRead(ACTION_BUTTON_PIN);
    printPID(testingVelPID);

    if (reading != action_ButtonState) {
      action_LastDebounceTime = millis();
      action_ButtonState = reading;
    }

    if ((millis() - action_LastDebounceTime) > action_DebounceDelay) {
      if (reading == LOW) {
        // If the button is pressed, exit the while loop
        // Serial.print("Button Pressed");
        pressed = true;
      }
    }
  }

  action_ButtonState = HIGH;
  buttonPressTime = millis();
}

void calcRollingAvg(){
  // Subtract the oldest value from the sum
  M1_Rolling_Avg_Vel_Sum -= M1_Rolling_Avg_Vel_Values[roll_Index];
  M2_Rolling_Avg_Vel_Sum -= M2_Rolling_Avg_Vel_Values[roll_Index];

  // Add the new value to the array and the sum
  M1_Rolling_Avg_Vel_Values[roll_Index] = M1_Vel_Save;
  M2_Rolling_Avg_Vel_Values[roll_Index] = M2_Vel_Save;
  M1_Rolling_Avg_Vel_Sum += M1_Vel_Save;
  M2_Rolling_Avg_Vel_Sum += M2_Vel_Save;

  roll_Index = (roll_Index + 1) % roll_N;

  if (roll_Index == 0) {
    roll_Avg_Filled = true;
  }

  if (roll_Avg_Filled) {
    M1_Rolling_Avg_Vel =  M1_Rolling_Avg_Vel_Sum/roll_N;
    M2_Rolling_Avg_Vel =  M2_Rolling_Avg_Vel_Sum/roll_N;
  } else {
    M1_Rolling_Avg_Vel =  M1_Rolling_Avg_Vel_Sum/(roll_Index + 1);
    M2_Rolling_Avg_Vel =  M2_Rolling_Avg_Vel_Sum/(roll_Index + 1);
  }
}


void setup() {
  Serial.begin(115200);

  //encoder pin setup
  //M1
  pinMode(M1_ENCODER_PIN_A, INPUT);
  pinMode(M1_ENCODER_PIN_B, INPUT);
  //M2
  pinMode(M2_ENCODER_PIN_A, INPUT);
  pinMode(M2_ENCODER_PIN_B, INPUT);

  // Attach interrupts to the encoder pins
  //M1
  attachInterrupt(digitalPinToInterrupt(M1_ENCODER_PIN_A), M1handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENCODER_PIN_B), M1handleEncoderB, CHANGE);
  //M2
  attachInterrupt(digitalPinToInterrupt(M2_ENCODER_PIN_A), M2handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENCODER_PIN_B), M2handleEncoderB, CHANGE);

  //Motor Control Pins setup
  //M1
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_EN, OUTPUT);
  //set to low for braking init
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  digitalWrite(M1_EN, LOW);
  //M2
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M2_EN, OUTPUT);
  //set to low for braking init
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
  digitalWrite(M2_EN, LOW);

  //PID SETUP
  M1_Vel_PID.SetMode(AUTOMATIC);
  M2_Vel_PID.SetMode(AUTOMATIC);
  M1_Vel_PID.SetOutputLimits(-230, 230);
  M2_Vel_PID.SetOutputLimits(-230, 230);

  M1_Pos_PID.SetMode(AUTOMATIC);
  M2_Pos_PID.SetMode(AUTOMATIC);
  M1_Pos_PID.SetOutputLimits(-TRANSLATE_MAX_SPEED, TRANSLATE_MAX_SPEED);
  M2_Pos_PID.SetOutputLimits(-TRANSLATE_MAX_SPEED, TRANSLATE_MAX_SPEED);

  //Action button setup
  pinMode(ACTION_BUTTON_PIN, INPUT);

  M1_Vel_setpoint = 0;
  M1_Vel_setpoint = 0;

  M1_Pos_setpoint = 0;
  M2_Pos_setpoint = 0;

  //rolling average velocity calculation
  // Initialize the array with zeros
  for (int i = 0; i < roll_N; i++) {
    M1_Rolling_Avg_Vel_Values[i] = 0;
    M2_Rolling_Avg_Vel_Values[i] = 0;
  }


}

void loop() {
  calcRollingAvg();
  if(loopNo==0){
    actionButtonWait();

    //PID target for testing
    M1_Pos_setpoint = testTargetPOSPID;
    M2_Pos_setpoint = testTargetPOSPID;
  }

  VelPIDCalculation();

  // delay(100);
  loopNo++;  
}
