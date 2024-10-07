#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <PID_v1.h>
#include <FastAccelStepper.h>
#include <ESP32Servo.h>

//ENCODERs###################################################################
//M1
#define M1_ENCODER_PIN_A 15 // Encoder pin A for motor 1
#define M1_ENCODER_PIN_B 16 // Encoder pin B for motor 1
//M2
#define M2_ENCODER_PIN_A 17 // Encoder pin A for motor 2
#define M2_ENCODER_PIN_B 18 // Encoder pin B for motor 2

//STEPPER###################################################
#define STEP_PIN 7 //called PUL on the TB6600
#define DIR_PIN 6
// Create an instance of the FastAccelStepperEngine
FastAccelStepperEngine engine;
// Create a pointer to the stepper
FastAccelStepper *stepper = NULL;

//SERVOS#########################################################
#define SERVO_LINK1_PIN 39
#define SERVO_LINK2_PIN 38
Servo servoLink1;
Servo servoLink2;

int servoLink1_Starting_Angle = 180;
int servoLink2_Starting_Angle = 5;

int servoLink1_Finish_Angle = 17;
int servoLink2_Finish_Angle = 180;

int servoLink1_Maintain_Angle = 18;
int servoLink2_Maintain_Angle = 177;

int servoLink1_startL2_Angle = 40;


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

//IMU pins
#define SDA_PIN 47
#define SCL_PIN 21

//DUAL CORE operation varaibles
TaskHandle_t Core0Task;

//IMU variables
Adafruit_LSM6DSOX sox;
double z_Vel = 0; //negative when M2 too fast, positive when M2 too slow vise versa for reverse
sensors_event_t accel, gyro, temp;
double z_Error = 0;

//ACTION buton
#define ACTION_BUTTON_PIN 40
int action_ButtonState = LOW;
int action_LastButtonState = 0;
//debounce varaibles
unsigned long action_LastDebounceTime = 0;
unsigned long action_DebounceDelay = 50;

//characteristic motor variables
const double GEAR_RATIO = 20.4086;
const int TICKS_PER_MOTOR_TURN = 48;
double enc_Ticks_Per_Rot = GEAR_RATIO*TICKS_PER_MOTOR_TURN;
const int TRANSLATE_MAX_SPEED = 450; //RPM for moving motors
const int TRANSLATE_MIN_SPEED = 100; //otherwise velocity PID fucks up becuase of deadzone
int min_Takeoff_PWM = 70; //was 63 70 with new wheels
const double WHEEL_CIRCUMFERANCE_M1 = 0.1394867; //in meters
const double WHEEL_CIRCUMFERANCE_M2 = WHEEL_CIRCUMFERANCE_M1;//0.138;
double targetBrakeVel = 125; //velocity target for braking

// const double WHEEL_CIRCUMFERANCE_M2 = 0.1385442; //in meters
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
double Vel_Kp = 0.5, Vel_Ki = 0, Vel_Kd = 0.001; //Vel_Kp = 1, Vel_Ki = 3, Vel_Kd = 0.000001;

PID M1_Vel_PID(&M1_Vel_input, &M1_Vel_output, &M1_Vel_setpoint, Vel_Kp, Vel_Ki, Vel_Kd, DIRECT);
PID M2_Vel_PID(&M2_Vel_input, &M2_Vel_output, &M2_Vel_setpoint, Vel_Kp, Vel_Ki, Vel_Kd, DIRECT);

double M1_Vel_Save;
double M2_Vel_Save;

double M1_Prev_PWM_Output = 0;
double M2_Prev_PWM_Output = 0;

double M1_Current_PWM = 0;
double M2_Current_PWM = 0;

//rolling average velocity
const int roll_N = 3;
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
double Pos_Kp = 0.20, Pos_Ki = 0, Pos_Kd = 0.02; //p = 0.16

PID M1_Pos_PID(&M1_Pos_input, &M1_Pos_output, &M1_Pos_setpoint, Pos_Kp, Pos_Ki, Pos_Kd, DIRECT);
PID M2_Pos_PID(&M2_Pos_input, &M2_Pos_output, &M2_Pos_setpoint, Pos_Kp, Pos_Ki, Pos_Kd, DIRECT);

double prev_M1_Pos_output = 0;
double prev_M2_Pos_output = 0;
double max_Pos_Output_change = 5; //20
bool targetBrakeVelExceeded = false; //true if in the current movement the velocity request has reached targetBrakeVel

//process operation variables i.e. goals and order of operation ##############################
//movement positions
bool posGoalReached = true; //true if the current goal has been reached - starts as true to avoid startup issues
int currentPosGoalIndex = -1; //-1 for no goal assigned yet, the index of the current goal in the goal list
bool robotStopped = false;
double posTargetPosition[] = {
  1,
  0,
  // 0
};

//arm positions
bool armGoalReached = true; //true if the arm goal has been reached - starts as true to avoid startup issues
int currentArmGoalIndex = -1; //-1 for no goal assigned yet
int pillarLowPos = 0; //find these values through testing
int pillarHighPos = 0;
int floorRHS = 0;
int floorLHS = 0;
int armTargetPosition[] = { //example values
  pillarLowPos,
  pillarHighPos,
  floorRHS
};

//main instructions
int currentInstructionIndex = 0;
bool currentInstructionStarted = false;
int instructionRegistry[2][2] = {
  {1,0},
  {1,0},
  // {1,0}
};

//instruction registry legend
//first number
  //1 = move to next position in goal list
  //2 = actuate servos
  //3 = delay
//second number
  //1 = move arm to next position in goal list


//temporary variables used for testing purposes
int loopNo = 0;
double testTargetVelPID = 200; //in rpm
double testTargetPOSPID = -0.6; //in meters
bool testingVelPID = false; //true if testing vel pid false if running position PIDs
bool printPIDtesting = true; //true if you want PID to print, false if you want no pid to print
bool IMUTesting = false; //true if you only want IMU to print
long int brakeTime = 3500; //2 seconds after the button is pressed brake
long int buttonPressTime = 0;
bool printMotorVelError = false;
double velError = 0;
double encDiffPrint = 0;

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

void actuateDriveTrain(int M1PWM, int M2PWM, bool brake = false){ //negative PWM values are backwards
  if(brake){ //braking
    //M1
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, LOW);
    analogWrite(M1_EN, 0);

    //M2
    digitalWrite(M2_IN1, LOW);
    digitalWrite(M2_IN2, LOW);
    analogWrite(M2_EN, 0);
  }else{ //driving
    //normalise PWM if it comes in as invalid
    if(M1PWM > 255){
      M1PWM = 255;
    }else if(M1PWM < -255){
      M1PWM = -255;
    }

    if(M2PWM > 255){
      M2PWM = 255;
    }else if(M2PWM < -255){
      M2PWM = -255;
    }

    //M1
    if (M1PWM > 0) {
      digitalWrite(M1_IN1, HIGH);
      digitalWrite(M1_IN2, LOW);
    } else if (M1PWM < 0) {
      digitalWrite(M1_IN1, LOW);
      digitalWrite(M1_IN2, HIGH);
    }
    analogWrite(M1_EN, int(abs(M1PWM)));

    //M2
    if (M2PWM > 0) {
      digitalWrite(M2_IN1, HIGH);
      digitalWrite(M2_IN2, LOW);
    } else if (M2PWM < 0) {
      digitalWrite(M2_IN1, LOW);
      digitalWrite(M2_IN2, HIGH);
    }
    analogWrite(M2_EN, int(abs(M2PWM)));
  }
}

double metersToEncTicks(double m, bool wheel = true){
  double rotations = 0;
  if(wheel){ //true for M1
    rotations = m/WHEEL_CIRCUMFERANCE_M1;
  }else{ //false for M2
    rotations = m/WHEEL_CIRCUMFERANCE_M2;
  }
  double ticks = rotations*enc_Ticks_Per_Rot;
  return ticks;
}

double encTicksToMeters(double ticks, bool wheel = true){
  double meters = 0;
  double rotations = ticks/enc_Ticks_Per_Rot;
  if(wheel){ //true for M1
    meters = rotations*WHEEL_CIRCUMFERANCE_M1;
  }else{
    meters = rotations*WHEEL_CIRCUMFERANCE_M2;
  }
  return meters;
}

int velocityToPWM(double velocity){
  //input velocity in rpm

  bool posVelocityRequest;
  if(velocity > 0){
    posVelocityRequest = true;
  }else{
    posVelocityRequest = false;
    velocity = abs(velocity);
  }
  
  double PWMReturn;
  double relationCutoff = 60; //the point at which the PWM relationship doesn't hold //was 32 but the robot would sometimes stop at this PWM

  //quadratic formula co-efficients
  float a = -0.0108;
  float b = 4.9083;
  float c = -74.459 - velocity;

  // Calculate discriminant
  float discriminant = b * b - 4 * a * c;

  if (discriminant < 0) {
    Serial.println("No real solutions to PWM conversion returning 0 PWM");
    return 0;
  } else {
    // Two possible solutions for x
    float x1 = (-b + sqrt(discriminant)) / (2 * a);
    float x2 = (-b - sqrt(discriminant)) / (2 * a);

    if(x1 < x2){
      PWMReturn = x1;
    }else{
      PWMReturn = x2;
    }
  }

  //if the PWM value is less than the cutoff point set it to the cutoff point
  if(PWMReturn < relationCutoff){
    PWMReturn = relationCutoff;
  }

  //if the velocity request was negative we need to flip the calculation back to negative PWM
  if(!posVelocityRequest){
    PWMReturn = -PWMReturn;
  }

  return int(PWMReturn);
}

void printPID(bool bypass = false){
  if(!IMUTesting){
    if(!printMotorVelError){
      if(printPIDtesting){
        if(bypass){
          Serial.print(">M1_Vel:");
          Serial.print(M1_Vel_Save);
          Serial.print(",M2_Vel:");
          Serial.print(M2_Vel_Save);
          Serial.print(",Target:");
          Serial.print(testTargetVelPID);
          Serial.print(",PWMM1:");
          Serial.print(M1_Current_PWM);
          Serial.print(",PWMM2:");
          Serial.print(M2_Current_PWM);
          Serial.print(",M1VelPIDOut:");
          Serial.print(M1_Vel_output);
          Serial.print(",M2VelPIDOut:");
          Serial.print(M2_Vel_output);
          Serial.print(",M1_Rolling_Avg_Vel:");
          Serial.print(M1_Rolling_Avg_Vel);
          Serial.print(",z_Vel:");
          Serial.print(z_Vel);
          Serial.print("\r\n");
        }else{
          Serial.print(">M1_Vel:");
          Serial.print(M1_Vel_Save);
          Serial.print(",POSPID_Target_M1:");
          double printTargetM1 = M1_Pos_setpoint/10;
          Serial.print(printTargetM1);
          Serial.print(",POSPID_Target_M2:");
          double printTargetM2 = M2_Pos_setpoint/10;
          Serial.print(printTargetM2);
          Serial.print(",M2_POSPID_Out:");
          Serial.print(M2_Pos_output);
          Serial.print(",M1_POSPID_Out:");
          Serial.print(M1_Pos_output);
          Serial.print(",M1encPos:");
          Serial.print(M1_encoderPos/10);
          Serial.print(",M2encPos:");
          Serial.print(M2_encoderPos/10);
          Serial.print(",M1PWM:");
          Serial.print(M1_Current_PWM);
          Serial.print(",M2PWM:");
          Serial.print(M2_Current_PWM);
          Serial.print(",VelPIDOut:");
          Serial.print(M1_Vel_output);
          Serial.print(",z_Vel:");
          Serial.print(z_Vel*10);
          Serial.print(",encDiffPrint:");
          Serial.print(encDiffPrint);
          Serial.print("\r\n");
        }
      }
    }else{
      velError = M1_Vel_Save - M2_Vel_Save;
      Serial.print(">VelError:");
      Serial.print(velError);
      Serial.print("\r\n");
    }
  }else{
      Serial.print(">z_Vel:");
      Serial.print(z_Vel);
      Serial.print("\r\n");
  }
}

void DCMotorTest(){
  int del = 5000;
  
  //M1
  Serial.println("M1 FORWARDS");
  actuateDriveTrain(255, 0);
  delay(del);

  Serial.println("M1 BACKWARDS");
  actuateDriveTrain(-255, 0);
  delay(del);

  //M2
  Serial.println("M2 FORWARDS");
  actuateDriveTrain(0, 255);
  delay(del);

  Serial.println("M1 BACKWARDS");
  actuateDriveTrain(0, -255);
  delay(del);
}

void StepperTest(){
  // stepper.moveTo(1450);
  // delay(2000);
  // stepper.moveTo(-1450);
  // delay(2000);
  // stepper.moveTo(0);
  // delay(2000);

  stepper->moveTo(1450);
  delay(5000);
  stepper->moveTo(-1450);
  delay(5000);
  stepper->moveTo(0);
  delay(5000);


}

void ServoTest(){
  //L1 start at 180
  //L1 finish at 17
  //L1 maintain at 18

  //L2 start at 5
  //L2 finish at 177
  //L2 maintain at 176


  //move L1
  // int angleL1;
  // int angleL2;
  // for (angleL1 = servoLink1_Starting_Angle; angleL1 >= servoLink1_Finish_Angle; angleL1 -= 1) {  
  //   servoLink1.write(angleL1);  // Set the servo to the current angle
  //   delay(10);  // Wait for the servo to reach the position
  // }
  
  //move L2
  // for (angleL2 = servoLink2_Starting_Angle; angleL2 <= servoLink2_Finish_Angle; angleL2 += 1) {  
  //   servoLink2.write(angleL2);  // Set the servo to the current angle
  //   delay(3);  // Wait for the servo to reach the position
  // }

  int angleL1 = servoLink1_Starting_Angle;
  int angleL2 = servoLink2_Starting_Angle;

  bool L1_Reached = false;
  bool L2_Reached = false;
  bool L2_Active = false;

  int loopCount = 0;
  while(!L1_Reached || !L2_Reached){
    //condition for if L2 is active
    if(angleL1 <= servoLink1_startL2_Angle){
      L2_Active = true;
    }

    //move target angle L1
    if(!L1_Reached){
      if(loopCount % 3 == 0){ //every 3rd loop move the L1 link
        angleL1 -= 1;
      }
    }

    //move target angle L1
    if(!L2_Reached && L2_Active){
      angleL2 += 1;
    }

    //actuate servos
    servoLink1.write(angleL1);
    //only actuate L2 if active
    if(L2_Active){
      servoLink2.write(angleL2);
    }


    //condition for if servo's have reached their positions
    if(angleL1 <= servoLink1_Finish_Angle){
      L1_Reached = true;
    }
    if(angleL2 >= servoLink2_Finish_Angle){
      L2_Reached = true;
    }

    delayMicroseconds(3300); //equal to delay(3.3)
    loopCount++;
  }



  // servoLink1.write(servoLink1_Maintain_Angle);
  // servoLink2.write(servoLink2_Maintain_Angle);

  // delay(10000);
  // servoLink1.write(servoLink1_Starting_Angle);
  // servoLink2.write(servoLink2_Starting_Angle);
}

void PosPIDCalculation(bool bypass = false){

  if(!bypass){ //standard operation
    //set Pos PID input to encoder values
    M1_Pos_input = M1_encoderPos;
    M2_Pos_input = M2_encoderPos;

    //compute PID output
    M1_Pos_PID.Compute();
    M2_Pos_PID.Compute();

    //logic for modifing position increases to limit acceleration

    if(M1_Pos_setpoint > M1_Pos_input){
      //forward case
      if(abs(M1_Pos_output-prev_M1_Pos_output) > max_Pos_Output_change){
        if(M1_Pos_output > prev_M1_Pos_output){
          M1_Pos_output = prev_M1_Pos_output + max_Pos_Output_change;
        }
      }
      if(abs(M2_Pos_output-prev_M2_Pos_output) > max_Pos_Output_change){
        if(M2_Pos_output > prev_M2_Pos_output){
          M2_Pos_output = prev_M2_Pos_output + max_Pos_Output_change;
        }
      }
    }else{
      //backwards case
      if(abs(M1_Pos_output-prev_M1_Pos_output) > max_Pos_Output_change){
        if(M1_Pos_output < prev_M1_Pos_output){
          M1_Pos_output = prev_M1_Pos_output - max_Pos_Output_change;
        }
      }
      if(abs(M2_Pos_output-prev_M2_Pos_output) > max_Pos_Output_change){
        if(M2_Pos_output < prev_M2_Pos_output){
          M2_Pos_output = prev_M2_Pos_output - max_Pos_Output_change;
        }
      }
    }

    //check for slowdown brake prep condition
    //condition for if vel request drops below target after acceleration
    if(targetBrakeVelExceeded){
      //M1
      if(abs(M1_Pos_output) < targetBrakeVel){
        if(M1_Pos_output < 0){
          M1_Pos_output = -targetBrakeVel;
        }else{
          M1_Pos_output = targetBrakeVel;
        }
      }
      //M2
      if(abs(M2_Pos_output) < targetBrakeVel){
        if(M2_Pos_output < 0){
          M2_Pos_output = -targetBrakeVel;
        }else{
          M2_Pos_output = targetBrakeVel;
        }
      }
    }

    double pos_ToleranceM1 = metersToEncTicks(0.06, true); //60mm
    double pos_ToleranceM2 = metersToEncTicks(0.06, false);

    //when close enforce targetBrakeVel
    if(abs(M1_encoderPos - M1_Pos_setpoint) < pos_ToleranceM1 && abs(M2_encoderPos - M2_Pos_setpoint) < pos_ToleranceM2){
      //if we are within range of the goal modify the posPid output
      if(M1_Pos_output < 0){ //negative case
        M1_Pos_output = targetBrakeVel;  
        M2_Pos_output = targetBrakeVel;
      }else{ //positive case
        M1_Pos_output = -targetBrakeVel;  
        M2_Pos_output = -targetBrakeVel;
      }
    }
  
    //set Vel PID setpoint to Pos PID output
    M1_Vel_setpoint = M1_Pos_output;
    M2_Vel_setpoint = M2_Pos_output;

    prev_M1_Pos_output = M1_Pos_output;
    prev_M2_Pos_output = M2_Pos_output;

    if(abs(M1_Pos_output) > targetBrakeVel || abs(M2_Pos_output) > targetBrakeVel){
      //if in neg or pos direction the PWM request goes above targetBrakeVel remember so that it never goes below targetBrakeVel before stopping
      targetBrakeVelExceeded = true;
    }

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

  //avoid to small increments and increase repeatability
  if (time_Passed < 10) {
    return; 
  }else{
    PosPIDCalculation(testingVelPID); //bypass = true - skipping for vel pid calc
    prev_Vel_PID_Time = current_Time;

    //check for too small velocity request
    double velCuttoff = 2;
    bool M1_cutt = false;
    bool M2_cutt = false;
    if(abs(M1_Vel_setpoint) < velCuttoff){
      M1_cutt = true;
    }
    if(abs(M2_Vel_setpoint) < velCuttoff){
      M2_cutt = true;
    }
    
    //input velocity from secondary core calculations
    M1_Vel_input = double(M1_Vel_Save);
    M2_Vel_input = double(M2_Vel_Save);

    //calculate pid output values
    M1_Vel_PID.Compute();
    M2_Vel_PID.Compute();

    double ENC_p = 10000; //test
    double encDiff = encTicksToMeters(M1_encoderPos, true) - encTicksToMeters(M2_encoderPos, false); //for forward movement -right turn = positive, -left turn = negative
    double ENC_Kp = encDiff*ENC_p;
    encDiffPrint = encDiff*10000;

    double M1_mod = 0;
    double M2_mod = 0;

    // if(ENC_Kp > 0){
    //   M1_mod = - ENC_Kp;
    //   M2_mod = ENC_Kp;
    // }else{
    //   M2_mod = ENC_Kp;
    //   M1_mod = -ENC_Kp;
    // }

    //convert velocity request to corresponding PWM reponse
    int M1_PWM_Out = M1_Vel_output + velocityToPWM(M1_Vel_setpoint + M1_mod);
    int M2_PWM_Out = M2_Vel_output + velocityToPWM(M2_Vel_setpoint + M2_mod);

    // Serial.println(M1_PWM_Out);
    // Serial.println(M2_PWM_Out);

    //check for takeoff condition
    double avgPWM = (M1_PWM_Out+M2_PWM_Out)/2;
    if(abs(M1_Rolling_Avg_Vel) < 2 && abs(M2_Rolling_Avg_Vel) < 2 && abs(avgPWM) < min_Takeoff_PWM){ //not moving and too low PWM to start
      if(avgPWM > 0){
        M1_PWM_Out = min_Takeoff_PWM;
        M2_PWM_Out = min_Takeoff_PWM;
      }else{
        M1_PWM_Out = -min_Takeoff_PWM;
        M2_PWM_Out = -min_Takeoff_PWM;
      }
    }

    //check for PWM going the wrong direction - if it is set to previuous PWM value
    if(M1_Pos_setpoint > M1_Pos_input){ //forward direction - positive PWM
      if(M1_PWM_Out < 0){
        M1_PWM_Out = M1_Prev_PWM_Output;
      }
      if(M2_PWM_Out < 0){
        M2_PWM_Out = M2_Prev_PWM_Output;
      }
    }else{ //backwards
      if(M1_PWM_Out > 0){
        M1_PWM_Out = M1_Prev_PWM_Output;
      }
      if(M2_PWM_Out > 0){
        M2_PWM_Out = M2_Prev_PWM_Output;
      }
    }

    //send PWM to drivetrain
    if(!robotStopped){
      actuateDriveTrain(M1_PWM_Out, M2_PWM_Out);
    }else{
      actuateDriveTrain(0,0,true);
    }

    //save PWM values for logging and printing
    M1_Current_PWM = M1_PWM_Out;
    M2_Current_PWM = M2_PWM_Out;
    
    printPID(testingVelPID);
  }
}

bool posGoalManager(){

  //run the PID
  VelPIDCalculation();

  //check if the robot is within tolerance of the goal
  double pos_ToleranceM1 = metersToEncTicks(0.005, true); //2mm tolerance
  double pos_ToleranceM2 = metersToEncTicks(0.005, false); //2mm tolerance

  bool ret = false;
  //if both motors are within tolerance then brake and consider the goal reached
  if(abs(M1_encoderPos - M1_Pos_setpoint) < pos_ToleranceM1 || abs(M2_encoderPos - M2_Pos_setpoint) < pos_ToleranceM2){ //was an and but now or - this assumes that the robot is straight when it approaches the goal
    actuateDriveTrain(0,0,true);
    delay(1000);
    robotStopped = true;
    ret = true;
  }else{
    robotStopped = false;
  }
  Serial.println(robotStopped);
  return ret;
}

bool armGoalManager(){
  return true; //placeholder - plan to return true only if the arm has reached it's final position otherwise move the arm one step towards the position
}

void actionButtonWait(bool print = true){
  //wait for the button to be pressed
  bool pressed = false;
  while(!pressed){

    int reading = digitalRead(ACTION_BUTTON_PIN);
    if(print){
      printPID(testingVelPID);
    }

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

void takeOffTest(){
  int PWMStartValue = 50;
  int PWMinc = 1;
  int millisTestTime = 1500;

  while(true){ //turn off robot to end test
    actuateDriveTrain(PWMStartValue,PWMStartValue);
    Serial.print("testing takeoff PWM val: ");
    Serial.println(PWMStartValue);
    delay(millisTestTime);
    actuateDriveTrain(0,0); //prevent too much strain
    Serial.println("waiting for button press");
    actionButtonWait();
    PWMStartValue += PWMinc;
  }
}

void brakeTest(){
  actuateDriveTrain(velocityToPWM(150), velocityToPWM(150));
  delay(2000);
  actuateDriveTrain(0,0, true);
  delay(3000);
}

void velocityTest(){
  int PWMTestVal = 60;
  int PWMinc = 5;
  int PWMminInc = 1;

  int takeoffPWM = 60; //need to run takeoff test to validate this number
  int takeOffMillis = 250; //how long the robot is given to takeoff from standstill

  int accelMillis = 1500; //how long the robot is given to speed up to vel
  int velSampleMillis = 500; //sampling period once at speed

  Serial.println("PWM value,velocityM1,velocityM2");

  //main while loop for testing PWM values
  while(true){ //turn off robot to stop test
    //takeoff
    if(abs(PWMTestVal) < takeoffPWM){
      actuateDriveTrain(takeoffPWM, takeoffPWM);
      delay(takeOffMillis);
    }

    //running velocity test
    actuateDriveTrain(PWMTestVal, PWMTestVal);
    delay(accelMillis);

    //save encPos and time
    long int saveTime = millis();
    int M1_enc_save = M1_encoderPos;
    int M2_enc_save = M2_encoderPos;
    //sampling time
    delay(velSampleMillis);

    int deltaEncM1 = M1_encoderPos - M1_enc_save;
    int deltaEncM2 = M2_encoderPos - M2_enc_save;

    actuateDriveTrain(0, 0, true); //brake

    double M1_Vel = ((double)deltaEncM1 / velSampleMillis) * 1000 * 60 / enc_Ticks_Per_Rot;
    double M2_Vel = ((double)deltaEncM2 / velSampleMillis) * 1000 * 60 / enc_Ticks_Per_Rot;

    //printing data for csv format
    Serial.print(PWMTestVal);
    Serial.print(",");
    Serial.print(M1_Vel, 2);
    Serial.print(",");
    Serial.println(M2_Vel, 2);

    if(abs(PWMTestVal) < takeoffPWM){
      PWMTestVal += PWMminInc;
    }else{
      PWMTestVal += PWMinc;
    }
    //wait for the button to be pressed
    actionButtonWait();
  }
}

void IMUUpdate(){
  sox.getEvent(&accel, &gyro, &temp);
  z_Vel = gyro.gyro.z * 180.0 / PI - z_Error;
  // Serial.println(z_Vel);
}

void IMUerrorCalc(){
  delay(200);
  double sum = 0;
  int tests = 10;
  z_Error = 0;

  //grab test data
  for(int i = 0; i < tests; i++){
    IMUUpdate();
    sum += z_Vel;
    delay(10);
  }

  //calculating error
  z_Error = sum/tests;
}

void instructionRegisterManager(){
  if(!currentInstructionStarted){
    //run the current instruction
    if(currentInstructionIndex < (sizeof(instructionRegistry) / sizeof(instructionRegistry[0]))){ //avoiding going out of bonds of the array
      switch (instructionRegistry[currentInstructionIndex][0]){
        case 1:
          currentPosGoalIndex += 1;
          posGoalReached = false;
          Serial.println("targetPos: ");
          Serial.println(posTargetPosition[currentPosGoalIndex]);
          Serial.println("index: ");
          Serial.println(currentPosGoalIndex);
          Serial.println(sizeof(instructionRegistry) / sizeof(instructionRegistry[0]));
          M1_Pos_setpoint = metersToEncTicks(posTargetPosition[currentPosGoalIndex], true);
          M2_Pos_setpoint = metersToEncTicks(posTargetPosition[currentPosGoalIndex], false);
          targetBrakeVelExceeded = false; //resetting this variable whenever moving to a new goal REMEMBER
          break;

        case 2:
          //run servo unfolding function
          break;

        case 3:
          //non-blocking delay
          break;
        
        default:
          break;
      }

      //tell armGoalManager to start moving the arm
      if(instructionRegistry[currentInstructionIndex][1] == 1){
        currentArmGoalIndex += 1;
        armGoalReached = false;
      }

      currentInstructionStarted = true;
    }
  }

  //move the robot and the arm
  posGoalReached = posGoalManager();
  armGoalReached = armGoalManager();

  //if they have both been reached we can move onto the next instruction
  if(posGoalReached && armGoalReached){
    currentInstructionIndex += 1; //go to the next instruction
    currentInstructionStarted = false;
  }
}

void core0Loop(void * pvParameters) {
  long int current_Time = micros();
  long int prev_Time = micros();
  while (true) {
      // Serial.println("core 0 printing");
      // delay(100);  // Add some delay to prevent watchdog timeout
      IMUUpdate();

      //velocity calculation
      long int current_Time = micros();
      double time_Passed = (current_Time - prev_Time)/1000; //convert from micros to millis
      if (time_Passed > 15){ //enough time has passed to get a decent sample of velocity
        //setting prev_time to time now as calculating is occcuring now
        prev_Time = micros();

        //find change in position  
        int M1_deltaEncPos = M1_encoderPos - prev_M1_enc_Pos;
        int M2_deltaEncPos = M2_encoderPos - prev_M2_enc_Pos;

        //save new enc pos as old for next loop
        prev_M1_enc_Pos = M1_encoderPos;
        prev_M2_enc_Pos = M2_encoderPos;

        // Serial.print("time_Passed: ");
        // Serial.println(time_Passed);

        // Serial.print("M1_deltaEncPos: ");
        // Serial.println(M1_deltaEncPos);

        //calculate velocity and save to global variable
        M1_Vel_Save = ((double)M1_deltaEncPos / time_Passed) * 1000 * 60 / enc_Ticks_Per_Rot;
        M2_Vel_Save = ((double)M2_deltaEncPos / time_Passed) * 1000 * 60 / enc_Ticks_Per_Rot;

        //vel save has changed now calculate the rolling average again with the change
        calcRollingAvg();

        // Serial.print("M1_Vel: ");
        // Serial.println(M1_Vel_Save);
      }

      //stability/not really sure if this is needed
      delay(1);
  }
}

void setup() {
  Serial.begin(115200);

  //IMU SETUP ########################
  Wire.begin(SDA_PIN, SCL_PIN);
  // Initialize I2C
  while(!sox.begin_I2C(0x6A)){
  Serial.println("Failed to find LSM6DSOX chip");
  delay(10);
  // while (1);
  }
  Serial.println("LSM6DSOX Found!");

  //running to find systematic error
  IMUerrorCalc();

  //encoder pin setup#########################
  //M1
  pinMode(M1_ENCODER_PIN_A, INPUT);
  pinMode(M1_ENCODER_PIN_B, INPUT);
  //M2
  pinMode(M2_ENCODER_PIN_A, INPUT);
  pinMode(M2_ENCODER_PIN_B, INPUT);

  // Attach interrupts to the encoder pins###################
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
  M1_Vel_PID.SetOutputLimits(-TRANSLATE_MAX_SPEED, TRANSLATE_MAX_SPEED);
  M2_Vel_PID.SetOutputLimits(-TRANSLATE_MAX_SPEED, TRANSLATE_MAX_SPEED);
  M1_Vel_PID.SetSampleTime(10);
  M2_Vel_PID.SetSampleTime(10);

  M1_Pos_PID.SetMode(AUTOMATIC);
  M2_Pos_PID.SetMode(AUTOMATIC);
  M1_Pos_PID.SetOutputLimits(-TRANSLATE_MAX_SPEED, TRANSLATE_MAX_SPEED);
  M2_Pos_PID.SetOutputLimits(-TRANSLATE_MAX_SPEED, TRANSLATE_MAX_SPEED);
  M1_Pos_PID.SetSampleTime(20);
  M2_Pos_PID.SetSampleTime(20); //was 50

  //Stepper setup ###########################
  // max speed tested 3200, accell 4800
  engine.init(); //Initialize the engine and set up the GPIOs

  // Attach the stepper to a step_pin
  stepper = engine.stepperConnectToPin(STEP_PIN);

  if (stepper) {
    stepper->setDirectionPin(DIR_PIN);  // Set the direction pin
    stepper->setAutoEnable(true);

    // Set acceleration and speed
    stepper->setSpeedInHz(3200);   // Steps per second
    stepper->setAcceleration(4800); // Steps per second squared
  }



  //Servo setup
  //commented out for now to avoid braking the servos
  // servoLink1.attach(SERVO_LINK1_PIN);
  // servoLink2.attach(SERVO_LINK2_PIN);
  // servoLink1.write(servoLink1_Starting_Angle);
  // servoLink2.write(servoLink2_Starting_Angle);

  //Action button setup
  pinMode(ACTION_BUTTON_PIN, INPUT);

  M1_Vel_setpoint = 0;
  M2_Vel_setpoint = 0;

  M1_Pos_setpoint = 0;
  M2_Pos_setpoint = 0;

  //rolling average velocity calculation
  // Initialize the array with zeros
  for (int i = 0; i < roll_N; i++) {
    M1_Rolling_Avg_Vel_Values[i] = 0;
    M2_Rolling_Avg_Vel_Values[i] = 0;
  }

  //initiate core2loop
  xTaskCreatePinnedToCore(
    core0Loop,
    "core0loop",
    2000, //stack size, if we get overflow errors this needs to be increased
    NULL,
    1,
    NULL,
    0 //void loop by deafault runs on core 1 so we use core 0
  );
}

void loop() {
  // calcRollingAvg();
  if(loopNo==0){
    actionButtonWait(false);
    // ServoTest();
    // //PID target for testing
    // testTargetPOSPID = metersToEncTicks(testTargetPOSPID);

    // if(!testingVelPID){
    //   M1_Pos_setpoint = testTargetPOSPID;
    //   M2_Pos_setpoint = testTargetPOSPID;
    // }
  }
  
  //find the current instructions
  instructionRegisterManager();
  // velocityTest();

  // takeOffTest();
  // brakeTest();


  delay(1);
  loopNo++;
}
