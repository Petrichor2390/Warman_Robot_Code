#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <PID_v1.h>
#include <FastAccelStepper.h>
#include <ESP32Servo.h>
#include <vector>

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
long int lastIMUTime = micros();
double z_POS_IMU = 0;

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
const double WHEEL_CIRCUMFERANCE_M1 = 0.141629; //in meters
const double WHEEL_CIRCUMFERANCE_M2 = WHEEL_CIRCUMFERANCE_M1;//0.138;
double targetBrakeVel = 150; //velocity target for braking

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
double Pos_input, Pos_output, Pos_setpoint;
double Pos_Kp = 0.11, Pos_Ki = 0, Pos_Kd = 0.0; //p = 0.22 //d= 0.02

PID pos_PID(&Pos_input, &Pos_output, &Pos_setpoint, Pos_Kp, Pos_Ki, Pos_Kd, DIRECT);

double prev_Pos_output = 0;
double max_Pos_Output_change = 20; //was 10
bool targetBrakeVelExceeded = false; //true if in the current movement the velocity request has reached targetBrakeVel
double M1_Vel_Save = 0;
double M2_Vel_Save = 0;
bool dirReverseFlag = false; //true if the robot has just changes direction

//EncDiff PID ##################################################################
double Enc_input, Enc_output, Enc_setpoint = 0;
double Enc_Kp = 20000*2, Enc_Ki = 0, Enc_Kd = 0; //p = 30000 //i= 500

PID enc_PID(&Enc_input, &Enc_output, &Enc_setpoint, Enc_Kp, Enc_Ki, Enc_Kd, DIRECT);

long int prev_Enc_PID_Time = micros();

double M1_Prev_PWM_Output = 0;
double M2_Prev_PWM_Output = 0;
double M1_Current_PWM = 0;
double M2_Current_PWM = 0;
int prev_M1_enc_Pos = 0;
int prev_M2_enc_Pos = 0;

//process operation variables i.e. goals and order of operation ##############################
//movement positions
bool posGoalReached = true; //true if the current goal has been reached - starts as true to avoid startup issues
int currentPosGoalIndex = -1; //-1 for no goal assigned yet, the index of the current goal in the goal list
bool robotStopped = false;
// double posTargetPosition[] = {
//   0.6,
//   0.1,
// };

double posTargetPosition[] = {
  -0.35,
  -0.45,
  0.4,
  0.5,
  0.20725,
};

//arm positions
bool armGoalReached = true; //true if the arm goal has been reached - starts as true to avoid startup issues
int currentArmGoalIndex = 0; //-1 for no goal assigned yet
int pillarLowPos = 1310; //find these values through testing
int pillarHighPos = -1150;
int floorRHS = -1505;
int floorLHS = 1470;
int currentArmPosition = 0;
int armTargetPosition[] = { //example values
  floorRHS,
  0,
  0,
};

//main instructions
int currentInstructionIndex = 0;
bool currentInstructionStarted = false;
int instructionRegistry[5][2] = {
  // {0,1},
  // {0,1},
  // {0,1},


  {1,0},
  {1,0},
  {1,0},
  {1,0},
  {1,0},
  // {1,0},
  // {1,0},

  // {1,0}
};

//full run example
// int instructionRegistry[10][2] = {
//   {0,1}, //short pillar
//   {0,1}, //tall pillar
//   {1,0}, // move to ball 3
//   {0,1}, // RHS ball
//   {1,0}, // move to ball 4
//   {0,1}, //LHS ball
//   {1,0}, //move to ball 5
//   {0,1}, //LHS ball
//   {1,0}, //move to ball 6
//   {0,1}, //RHS ball
// };

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
float offset = 1;
double maxToleranceTest = 0;

//data from testing -KEEP
std::vector<std::pair<int, float>> offsetResponseNEG = {
  {-70, 1.15},
  {-90, 1.15},
  {-110, 1.17},
  {-130, 1.17},
  {-150, 1.19},
  {-170, 1.23},
  {-190, 1.21},
  {-210, 1.15},  
};

std::vector<std::pair<int, float>> offsetResponsePOS = {
  {70, 0.95},
  {90, 1.0},
  {110, 0.98},
  {130, 1.04},
  {150, 1.03},
  {190, 1.05},
  {210, 1.05},
  {230, 1.05},
};

std::vector<std::vector<double>> brakingProfile = {
  //{start vel applicable, end vel applicable, M1_PWM brake, M2_PWM brake, off wait M1, off wait M2}
  {0, 230, 1, 1, 1, 1}, //0
  {230, 300, 1, 1, 2, 2}, //1
  {300, 350, 1, 1, 3, 3}, //2
  {350, 500, 1, 1, 4, 4}, //3

  //reverse
  {-1, -230, 1, 1, 2, 2}, //4
  {-230, -280, 1, 1, 3, 3}, //5
  {-280, -350, 1, 1, 3, 3}, //6
  {-350, -500, 1, 1, 4, 4}, //7
};

std::vector<std::pair<int, float>> brakeDistanceResponsePOS = {
  //velocity, braking distance m
  {125,	0.01},
  {180,	0.0346698},
  {183,	0.0333971},
  {191,	0.0330792},
  {190,	0.0340329},
  {189,	0.035783},
  {203,	0.035783},
  {206,	0.0373725},
  {207,	0.037532},
  {208,	0.039281},
  {216,	0.0652036},
  {221,	0.0690206},
  {226,	0.0723602},
  {226,	0.0707707},
  {232,	0.0744282},
  {239,	0.0753819},
  {238,	0.0760188},
  {234,	0.0790405},
  {243,	0.0785631},
  {249,	0.0793584},
  {256,	0.0814253},
  {252,	0.0846065},
  {249,	0.0801537},
  {260,	0.0820611},
  {269,	0.0881045},
  {263,	0.086515},
  {273,	0.0895367},
  {280,	0.0947848},
  {282,	0.0903309},
  {273,	0.0909678},
  {278,	0.0952611},
  {291,	0.1173667},
  {275,	0.0957385},
  {293,	0.1194347},
  {292,	0.1149819},
  {306,	0.1267497},
  {295,	0.1210253},
  {313,	0.1312036},
  {307,	0.131362},
  {324,	0.1383602},
  {312,	0.1451978},
  {322,	0.1327931},
  {330,	0.1437667},
  {324,	0.1523544},
  {338,	0.1444025},
  {351,	0.1754148},
  {347,	0.1792318},
  {354,	0.1781186},
  {360,	0.182094},
  {366,	0.1827298},
  {363,	0.1849562},
  {376,	0.1909996},
  {373,	0.1959298},
  {377,	0.20086},
  {382,	0.2048365},
  {366,	0.2104025},
  {377,	0.2126289},
  {377,	0.2167638},
  {391,	0.2162864},
  {385,	0.2121515},
  {390,	0.2134242},
  {388,	0.2177175},
  {388,	0.2177175},
  {383,	0.2170817},
  {384,	0.2108788},
};

std::vector<std::pair<int, float>> brakeDistanceResponseNEG = {
{125,	0.025},
{175,	0.048144},
{171,	0.046698},
{176,	0.048289},
{177,	0.046987},
{172,	0.047421},
{177,	0.048289},
{178,	0.04959},
{175,	0.04959},
{181,	0.052337},
{188,	0.053204},
{189,	0.05306},
{194,	0.054505},
{200,	0.05624},
{209,	0.059566},
{211,	0.062023},
{218,	0.064915},
{228,	0.067951},
{232,	0.082553},
{235,	0.0853},
{236,	0.091951},
{240,	0.087324},
{243,	0.089204},
{252,	0.093975},
{253,	0.091372},
{252,	0.09542},
{264,	0.095276},
{263,	0.100047},
{267,	0.097155},
{268,	0.102938},
{284,	0.103661},
{275,	0.105975},
{282,	0.105685},
{289,	0.11089},
{293,	0.108432},
{299,	0.114794},
{299,	0.110601},
{310,	0.114215},
{304,	0.115227},
{313,	0.11624},
{320,	0.118553},
{314,	0.122312},
{324,	0.118119},
{320,	0.125203},
{325,	0.125059},
{335,	0.156721},
{340,	0.162215},
{346,	0.132577},
{352,	0.168576},
{356,	0.167564},
{363,	0.168432},
};

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

int getM2EncPos(){
  // return M2_encoderPos;
  return int(M2_encoderPos*0.99); //*0.99 perfect
}

float interpolate(int x, int dataSet = 1){
  std::vector<std::pair<int, float>> data;
  x = abs(x);
  switch (dataSet)
  {
  case 1:
    //positive offset response
    data = offsetResponsePOS;
    break;

  case 2:
    //negative offset response
    data = offsetResponseNEG;
    break;

  case 3:
    //
    data = brakeDistanceResponsePOS;
    break;

  case 4:
    //
    data = brakeDistanceResponseNEG;
    break;
  
  default:
    break;
  }

  for(size_t i = 0; i < data.size() - 1; ++i) {
    int x0 = data[i].first;
    int x1 = data[i + 1].first;

    if (x >= x0 && x <= x1) {
      float y0 = data[i].second;
      float y1 = data[i + 1].second;
      // Perform interpolation
      return y0 + (float)(x - x0) * (y1 - y0) / (x1 - x0);
    }
  }
  if (x < data.front().first) return data.front().second;
  if (x > data.back().first) return data.back().second;

  Serial.println("Could not interpolate: returning default");
  if(dataSet == 1 || dataSet == 2){
    return 1.0;
  }
  return 0.0; //failsafe incase the code doesn't work
}

std::vector<double> getBrakingProfile(double velocity){
  //find which braking profile to use
  // Serial.print("choose brake profile from velocity: ");
  // Serial.println(velocity);
  int brakingProfileIndex = -1; //will change if it doesn't it is printed
  //pos case
  for(size_t i = 0; i < brakingProfile.size(); i++){
    if(velocity > brakingProfile.at(i).at(0) && velocity < brakingProfile.at(i).at(1)){
      brakingProfileIndex = i;
    }
  }
  if(brakingProfileIndex == -1 && velocity < 0){
    //neg case
    for(size_t i = 0; i < brakingProfile.size(); i++){
      if(abs(velocity) > abs(brakingProfile.at(i).at(0)) && abs(velocity) < abs(brakingProfile.at(i).at(1))){
        brakingProfileIndex = i;
      }
    }
  }

  if(brakingProfileIndex == -1){
    // Serial.println("Could not find braking profile returning default profile");
  }else{
    // Serial.println("Using braking profile index: ");
    // Serial.println(brakingProfileIndex);
    return brakingProfile.at(brakingProfileIndex);
  }

  //safety default profile if not found
  return brakingProfile.at(0);

}

float getPWMOffsetModifier(int PWM){
  int offSetData = 0;
  if(PWM > 0){ //using positive data
    offSetData = 1;
  }else{
    offSetData = 2;
  }
  return interpolate(PWM, offSetData);
}

void motorPinInterface(int motorID, int PWM){
  //normalise PWM
  if(PWM > 255){
    PWM = 255;
  }else if(PWM < -255){
    PWM = -255;
  }
  
  if(motorID == 1){ //M1
    if (PWM > 0) {
      digitalWrite(M1_IN1, HIGH);
      digitalWrite(M1_IN2, LOW);
    } else if (PWM < 0) {
      digitalWrite(M1_IN1, LOW);
      digitalWrite(M1_IN2, HIGH);
    }
    analogWrite(M1_EN, int(abs(PWM)));
  }else if(motorID == 2){ //M2
    if (PWM > 0) {
      digitalWrite(M2_IN1, HIGH);
      digitalWrite(M2_IN2, LOW);
    } else if (PWM < 0) {
      digitalWrite(M2_IN1, LOW);
      digitalWrite(M2_IN2, HIGH);
    }
    analogWrite(M2_EN, int(abs(PWM)));
  }else{
    Serial.println("motorID invalid");
  }
}

void actuateDriveTrain(int M1PWM, int M2PWM, bool brake = false){ //negative PWM values are backwards
  //modifier to account for difference in motors
  float offSetMod = getPWMOffsetModifier(M1PWM); //based on M1PWM - could be future problem
  // Serial.println("offSetMod: ");
  // Serial.println(offSetMod);
  M2PWM = M2PWM*offSetMod;

  if(brake){ //braking
    //get braking profile to charactise brake
    std::vector<double> brakingProfileApply = getBrakingProfile((M1_Vel_Save+M2_Vel_Save)/2); //use average velocity

    // Serial.println("PWM M1: ");
    // Serial.println(brakingProfileApply.at(2));
    // Serial.println("PWM M2: ");
    // Serial.println(brakingProfileApply.at(3));
    // Serial.println("duty off M1 : ");
    // Serial.println(brakingProfileApply.at(4));
    // Serial.println("duty of M2: ");
    // Serial.println(brakingProfileApply.at(5));

    int PWMbrakeM1_n1 = brakingProfileApply.at(2);
    int PWMbrakeM2_n1 = brakingProfileApply.at(3);

    int PWMOutM1_n1 = 0;
    int PWMOutM2_n1 = 0;

    if(M1_Rolling_Avg_Vel > 0){
      PWMOutM1_n1 = -PWMbrakeM1_n1;
      PWMOutM2_n1 = -PWMbrakeM1_n1;
    }else{
      PWMOutM1_n1 = PWMbrakeM1_n1;
      PWMOutM2_n1 = PWMbrakeM2_n1;
    }

    double avgVel = (abs(M1_Vel_Save)+abs(M2_Vel_Save))/2;
    bool stoppedFlag = false;
    double velTol= 2; //was 20
    int loopC = 0;
    int offDuty = 0;
    if(PWMOutM1_n1 < 0){
      offDuty = -1;
    }else{
      offDuty = 1;
    }
    while(!stoppedFlag){
      //if(duty cycle on)
      //M1
      if(loopC == 0 || loopC % static_cast<int>(brakingProfileApply.at(4)) == 0){
        //on duty
        motorPinInterface(1, PWMOutM1_n1);
        // Serial.println("M1 on duty");
      }else{
        //off duty
        motorPinInterface(1, -offDuty);
        // Serial.println("M1 off duty");
      }

      //M2
      if(loopC == 0 || loopC % static_cast<int>(brakingProfileApply.at(5)) == 0){
        //on duty
        motorPinInterface(2, PWMOutM2_n1);
        // Serial.println("M2 on duty");
      }else{
        //off duty
        motorPinInterface(2, -offDuty);
        // Serial.println("M2 off duty");
      }

      avgVel = (abs(M1_Vel_Save)+abs(M2_Vel_Save))/2;
      if(avgVel < velTol){
        stoppedFlag = true;
      }else{
        delay(5); //loop speed - MAY BE weird response because of this
        loopC++;
      }
    }
  
  //to stop it wining
  analogWrite(M1_EN, 0);
  analogWrite(M2_EN, 0);

  }else{ //driving
    //M1
    motorPinInterface(1, M1PWM);
    // Serial.println("M1 PWM delivered: ");
    // Serial.println(M1PWM);

    //M2
    motorPinInterface(2, M2PWM);
    // Serial.println("M2 PWM delivered: ");
    // Serial.println(M2PWM);
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
  }

  //the logic doesn't hold if you keep pushing numbers higher
  if(abs(velocity) > 420){
    if(velocity > 0){
      velocity = 420;
    }else{
      velocity = -420;
    }
  }
  
  double PWMReturn;
  double relationCutoffPOS = 52+5;
  double relationCutoffNEG = -63-5;

  //quadratic formula co-efficients
  float a;
  float b;
  float c;

  if(posVelocityRequest){ //positive case
    a = -0.0166;
    b = 6.7173;
    c = -247.48 - velocity;
  }else{ //negative case
    a = 0.0129;
    b = 5.3007;
    c = 118.54 - velocity;
  }

  // Calculate discriminant
  float discriminant = b*b - 4*a*c;

  if (discriminant < 0) {
    Serial.println("No real solutions to PWM conversion returning 0 PWM");
    return 0;
  }else{
    // Two possible solutions for x
    float x1 = (-b + sqrt(discriminant)) / (2*a);
    float x2 = (-b - sqrt(discriminant)) / (2*a);

    if(abs(x1) < abs(x2)){
      PWMReturn = x1;
    }else{
      PWMReturn = x2;
    }
  }

  //if the PWM value is less than the cutoff point set it to the cutoff point
  // Serial.print("PWMReturn: ");
  // Serial.println(PWMReturn);

  //
  if(PWMReturn > 0 && PWMReturn < relationCutoffPOS){
    PWMReturn = relationCutoffPOS;
  }

  if(PWMReturn < 0 && PWMReturn > relationCutoffNEG){
    PWMReturn = relationCutoffNEG;
  }
  
  return int(PWMReturn);
}

void armMove(int pos){
  //check that the value is in range
  int sendPos = pos - currentArmPosition;
  stepper->move(sendPos);
  currentArmPosition = pos;
  delay(3500);
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
          Serial.print(",Enc_PID:");
          Serial.print(Enc_output);
          Serial.print(",M1_Rolling_Avg_Vel:");
          Serial.print(M1_Rolling_Avg_Vel);
          Serial.print(",z_Vel:");
          Serial.print(z_Vel);
          Serial.print("\r\n");
        }else{
          Serial.print(">M1_Vel:");
          Serial.print(M1_Vel_Save);
          Serial.print(",M2_Vel:");
          Serial.print(M2_Vel_Save);
          Serial.print(",POSPID_Target_M1:");
          double printTarget = Pos_setpoint;
          Serial.print(printTarget);
          Serial.print(",POSPID_Out:");
          Serial.print(Pos_output);
          Serial.print(",M1encPos:");
          Serial.print(M1_encoderPos);
          Serial.print(",M2encPos:");
          Serial.print(getM2EncPos());
          Serial.print(",M1PWM:");
          Serial.print(M1_Current_PWM);
          Serial.print(",M2PWM:");
          Serial.print(M2_Current_PWM);
          Serial.print(",Enc_PID:");
          Serial.print(Enc_output);
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

  stepper->moveTo(1480);
  delay(5000);
  stepper->moveTo(-1455);
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

void PosPIDCalculation(){

  //set Pos PID input to encoder values
  double avgEnc = (M1_encoderPos+getM2EncPos())/2;
  Pos_input = avgEnc; //average of the 2 enc

  //compute PID output
  pos_PID.Compute();

  // Serial.println("raw POSPID");
  // Serial.println(Pos_output);
 
  //logic for modifing position increases to limit acceleration
  if(Pos_setpoint > Pos_input){
    //forward case
    if(abs(Pos_output-prev_Pos_output) > max_Pos_Output_change){
      if(Pos_output > prev_Pos_output){
        Serial.println("Limiting Forward accelleration");
        if(prev_Pos_output*Pos_output > 0){
          Pos_output = prev_Pos_output + max_Pos_Output_change;
        }else{ //if we have just changed directions
          Pos_output = 0 + max_Pos_Output_change;
          // Serial.println("using new code");
        }
      }
    }
  }else{
    //backwards case
    if(abs(Pos_output-prev_Pos_output) > max_Pos_Output_change){
      if(Pos_output < prev_Pos_output){
        Serial.println("Limiting reverse accelleration");
        if(prev_Pos_output*Pos_output > 0){
          Pos_output = prev_Pos_output - max_Pos_Output_change;
        }else{ //if we have just changed directions
          // Serial.println("using new code");
          Pos_output = 0 - max_Pos_Output_change;
        }
      }
    }
  }

  //if the velocity value is lower than braketargetvel then set to braketargetval
  if(abs(Pos_output) < targetBrakeVel){
    if(Pos_output > 0){
      // Serial.println("manually changing POSPID to");
      // Serial.println(targetBrakeVel);
      Pos_output = targetBrakeVel;
    }else{
      // Serial.println("manually changing POSPID to");
      // Serial.println(-targetBrakeVel);
      Pos_output = -targetBrakeVel;
    }
  }

  //this code needs checking
  double pos_Tolerance = metersToEncTicks(0.1, true); //was 0.06

  //when close enforce targetBrakeVel if going too fast
  if(abs(avgEnc - Pos_setpoint) < pos_Tolerance){
    //if we are within range of the goal modify the posPid output
    if(Pos_output < 0){ //negative case
      // Serial.println("manually changing POSPID to");
      // Serial.println(-targetBrakeVel);
      Pos_output = -targetBrakeVel;
    }else{ //positive case
      // Serial.println("manually changing POSPID to");
      // Serial.println(targetBrakeVel);
      Pos_output = targetBrakeVel;
    }
  }

  //make sure that the output is positive if moving forward and negative if moving backwards
  if(Pos_setpoint > Pos_input){ //forward direction
    if(Pos_output < 0){
      Serial.print("Ignoring new POSPID: requested opposite velocity");
      Pos_output = prev_Pos_output;
    }
  }else{ //backwards
    if(Pos_output > 0){
      Serial.print("Ignoring new POSPID: requested opposite velocity");
      Pos_output = prev_Pos_output;
    }
  }

  //save previous output
  prev_Pos_output = Pos_output;
}

void VelPIDCalculation(){
  //velocuty PID
  //calculate time passed
  long int current_Time = micros();
  double time_Passed = (current_Time - prev_Enc_PID_Time)/1000; //convert from micros to millis

  //avoid to small increments and increase repeatability
  if (time_Passed < 1) {
    return; 
  }else{
    PosPIDCalculation(); //bypass = true - skipping for vel pid calc
    prev_Enc_PID_Time = current_Time;
    
    Enc_input = encTicksToMeters(M1_encoderPos, true) - encTicksToMeters(getM2EncPos(), false);  
    encDiffPrint = Enc_input*10000;

    //calculate pid output values
    enc_PID.Compute();

    double M1_Modified_Pos_output = Pos_output; //+ Enc_output;
    double M2_Modified_Pos_output = Pos_output; //- Enc_output;

    //convert velocity request to corresponding PWM reponse
    int M1_PWM_Out = velocityToPWM(M1_Modified_Pos_output);
    int M2_PWM_Out = velocityToPWM(M2_Modified_Pos_output);

    //calculat the effect of the ENC_PID on PWM
    // int PWM_test = velocityToPWM(Pos_output);
    // int M1_Diff = M1_PWM_Out - PWM_test;
    // int M2_Diff = M2_PWM_Out - PWM_test;
    // Serial.println("M1 and M2 Diff due to encPID: ");
    // Serial.println(M1_Diff);
    // Serial.println(M2_Diff);

    // check for PWM going the wrong direction - if it is set to previuous PWM value
    if(Pos_setpoint > Pos_input){ //forward direction - positive PWM
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

    //modify PWM form ENC_PID right at the end so the system has max authority
    if(Enc_output > 0){
      M1_PWM_Out += Enc_output;
    }else{
      M2_PWM_Out -= Enc_output;
    }

    //send PWM to drivetrain
    if(!robotStopped){
      actuateDriveTrain(M1_PWM_Out, M2_PWM_Out);
    }else{
      actuateDriveTrain(0,0,true);
      // actuateDriveTrain(0,0);

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

  //check if the robot is within set tolerance of the goal
  double pos_Tolerance = metersToEncTicks(0.035, true); //default value if data lookup doesn't work

  //calculate tolerance based on speed - DATA NOT READY FUTURE CODE
  // int inAvgVel = static_cast<int>((M1_Vel_Save+M2_Vel_Save)/2);
  int inAvgVel = static_cast<int>((M1_Rolling_Avg_Vel+M2_Rolling_Avg_Vel)/2);
  // Serial.print("Deciding Brake Based on Vel: ");
  // Serial.println(inAvgVel);
  if(inAvgVel > 0){
    //positive case
    pos_Tolerance = metersToEncTicks(interpolate(inAvgVel, 3)); //iterpolate using the correct dataset
  }else{
    //negative case
    pos_Tolerance = abs(metersToEncTicks(interpolate(inAvgVel, 4))); //iterpolate using the correct dataset
  }

  // Serial.print("Tolerance for braking: ");
  // Serial.println(pos_Tolerance);

  bool ret = false;
  //if both motors are within tolerance then brake and consider the goal reached
  double avgEnc = (M1_encoderPos+getM2EncPos())/2;

  //calculate if the robot should stop at a goal
  // if(((abs(Pos_setpoint) - abs(pos_Tolerance)) - abs(avgEnc)) < 0){ //old condition for stopping
  // Serial.print("Pos_Setpoint: ");
  // Serial.println(abs(Pos_setpoint));
  // Serial.print("avgEnc: ");
  // Serial.println(abs(avgEnc));
  // Serial.print("pos_Tolerance: ");
  // Serial.println(abs(pos_Tolerance));

  if(abs(Pos_setpoint-avgEnc) < abs(pos_Tolerance)){ // condition for stopping
    if(!robotStopped){
      Serial.println("Braking!");
      actuateDriveTrain(0,0,true);
      delay(500);
      M2_encoderPos = int(M1_encoderPos/0.99); //TEMP SOLUTION
      //for safety
    }

    // Serial.println("STOPPED AT TARGET");
    robotStopped = true;
    ret = true;
  }else{
    robotStopped = false;
  }
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

  M1_encoderPos = 0;
  M2_encoderPos = 0;

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
  int testVel = -125;
  int testVelInc = -5;

  int accelTime = 500; //ms of accel before braking
  int sampleTime = 45; //ms of sample to match rolling avg
  int accelEncTicks = metersToEncTicks(0.5);

  Serial.println("AvgVel,brakingDistance,brakeTime");

  while(true){
    //waiting for button press
    // Serial.println("waiting for press");
    actionButtonWait(false);
    // Serial.println("testing: ");
    // Serial.println(testVel);
    int PWM = velocityToPWM(testVel);
    // Serial.print("PWM: ");
    // Serial.println(PWM);
    actuateDriveTrain(PWM, PWM);
    
    //reset encoder pos
    M1_encoderPos = 0;
    M2_encoderPos = 0;

    bool reachedPosFlag = false;
    long int posReachStartTime = millis();
    while(!reachedPosFlag){
      double avgEncPos = (M1_encoderPos+getM2EncPos())/2;
      if(abs(avgEncPos) > accelEncTicks){
        reachedPosFlag = true;
      }
      // long int cTime = millis();
      // if(cTime - posReachStartTime > 5000){
      //   reachedPosFlag = true;

      // }
    }

    //record speed
    int M1_enc_save = M1_encoderPos;
    int M2_enc_save = getM2EncPos();
    //sampling time
    delay(sampleTime);

    int deltaEncM1 = M1_encoderPos - M1_enc_save;
    int deltaEncM2 = getM2EncPos() - M2_enc_save;

    double M1_MAXVel = ((double)deltaEncM1 / sampleTime) * 1000 * 60 / enc_Ticks_Per_Rot;
    double M2_MAXVel = ((double)deltaEncM2 / sampleTime) * 1000 * 60 / enc_Ticks_Per_Rot;

    long int startBrakeTime = micros();
    long int stopBrakeTime = micros();
    long int encTicksTempSave = M1_encoderPos;
    actuateDriveTrain(0, 0, true); //brake
    bool stoppedFlag = false;
    double stoppedTol = 20; // both motors must be below this velocity to be considered stopped (RPM)

    // Serial.println("Starting Stop Check");
    while(!stoppedFlag){
      // Serial.println(M1_Vel_Save);
      // Serial.println(M2_Vel_Save);

      double avgVelCheckStopped = (abs(M1_Vel_Save)+abs(M2_Vel_Save))/2;
      
      if(avgVelCheckStopped < stoppedTol){
        stopBrakeTime = micros();
        stoppedFlag = true;
      }
    }

    //calculate braking time
    double brakingTime = (stopBrakeTime - startBrakeTime)/1000;
    double avgVel = (M1_MAXVel+M2_MAXVel)/2;

    //calculate braking distance
    delay(100);
    double brakingDistance = encTicksToMeters(M1_encoderPos-encTicksTempSave);

    Serial.print(avgVel);
    Serial.print(",");
    Serial.print(brakingDistance, 6);
    Serial.print(",");
    Serial.println(brakingTime, 2);

    //increment testvel
    testVel += testVelInc;
  }
}

void velocityTest(){
  int PWMTestVal = -50;
  int PWMinc = -5;
  int PWMminInc = -1;

  int accelMillis = 750; //how long the robot is given to speed up to vel
  int velSampleMillis = 1500; //sampling period once at speed

  Serial.println("PWM value,velocityM1,velocityM2");

  //main while loop for testing PWM values
  while(true){ //turn off robot to stop test
    if(abs(PWMTestVal) > 150){
      accelMillis = 500;
      velSampleMillis = 1000;
    }else{
      int accelMillis = 750;
      int velSampleMillis = 1500;
    }

    //running velocity test
    actuateDriveTrain(PWMTestVal, PWMTestVal);
    delay(accelMillis);

    //save encPos and time
    long int saveTime = millis();
    int M1_enc_save = M1_encoderPos;
    int M2_enc_save = getM2EncPos();
    //sampling time
    delay(velSampleMillis);

    int deltaEncM1 = M1_encoderPos - M1_enc_save;
    int deltaEncM2 = getM2EncPos() - M2_enc_save;

    actuateDriveTrain(0, 0); //brake

    double M1_Vel = ((double)deltaEncM1 / velSampleMillis) * 1000 * 60 / enc_Ticks_Per_Rot;
    double M2_Vel = ((double)deltaEncM2 / velSampleMillis) * 1000 * 60 / enc_Ticks_Per_Rot;

    //printing data for csv format
    Serial.print(PWMTestVal);
    Serial.print(",");
    Serial.print(M1_Vel, 2);
    Serial.print(",");
    Serial.println(M2_Vel, 2);

    if(abs(PWMTestVal) < 70){
      PWMTestVal += PWMminInc;
    }else{
      PWMTestVal += PWMinc;
    }
    //wait for the button to be pressed
    actionButtonWait(false);
  }
}

void OffsetTest(){
  int PWMTestVal = -70;
  int PWMInc = -20;
  //testingPeriod
  int testMillis = 1500;

  std::vector<std::pair<int, float>> offsetStorage; //PWM then offset

  bool testingValues = true;
  while(testingValues){
    if(abs(PWMTestVal) < 89){
      testMillis = 3000;
    }else if(abs(PWMTestVal) > 125){
      testMillis = 2000;
    }else if(abs(PWMTestVal) > 165){
      testMillis = 1500;
    }

    bool offsetFound = false;
    bool accepted = false;
    offset = 1.0;
    while(!offsetFound){
      if(!accepted){
        Serial.println("Enter offset to test: ");

        while(true){
          float tempOffset = -1;
          if (Serial.available() > 0) {
            tempOffset = Serial.parseFloat(); //offset variable is used in actauteDriveTrain
          }

          if(tempOffset > 0 && tempOffset < 5){
            Serial.println("You enterred: ");
            Serial.println(tempOffset);
            offset = tempOffset;
            break; //exit blocking loop once input is received
          }
        }

        actuateDriveTrain(PWMTestVal,PWMTestVal);
        delay(testMillis);
        actuateDriveTrain(0,0); //stop
      }else{
        offsetFound = true;
        std::pair<int, float> tempPair = {PWMTestVal, offset};
        offsetStorage.push_back(tempPair);
      }

      if(!offsetFound){
        accepted = false;
        Serial.println("Is this value good? (y/n): ");
        while(true){
          if (Serial.available() > 0) {
            char accept = Serial.read();
            if (accept == 'y' || accept == 'Y') {
              accepted = true;
              Serial.println("You selected yes");
              break;

            } else if (accept == 'n' || accept == 'N') {
              accepted = false;
              Serial.println("You selected no");
              break;
            }
          }
        }
      }
    }
    PWMTestVal += PWMInc;
    Serial.println("Testing PWM: ");
    Serial.println(PWMTestVal);
    if(PWMTestVal > 255){
      testingValues = false;
    }
  }


  //printout for excell
  Serial.println("PWM value,offset");

  for(size_t i = 0; i < offsetStorage.size(); i++){
    Serial.print(offsetStorage.at(i).first);
    Serial.print(",");
    Serial.println(offsetStorage.at(i).second, 2);
  }
}

void IMUUpdate(bool calcZPos = true){
  sox.getEvent(&accel, &gyro, &temp);
  z_Vel = gyro.gyro.z * 180.0 / PI - z_Error;

  //new code for finding the z position of the robot - untested
  if(calcZPos){
    long int currentTime = micros();
    int IMUtimeDelta = currentTime-lastIMUTime;
    lastIMUTime = currentTime;
    z_POS_IMU += z_Vel*IMUtimeDelta*0.000001; //convert to seconds
  }
}

void IMUerrorCalc(){
  delay(200);
  double sum = 0;
  int tests = 10;
  z_Error = 0;

  //grab test data
  for(int i = 0; i < tests; i++){
    IMUUpdate(false); //don't calculate Z pos
    sum += z_Vel;
    delay(10);
  }

  //calculating error
  z_Error = sum/tests;

  //set the last IMU time to current time to avoid error
  lastIMUTime = micros();
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
          Pos_setpoint = metersToEncTicks(posTargetPosition[currentPosGoalIndex], true); //2 wheels different diameters not accounted for here
          pos_PID.Compute();
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
        armMove(armTargetPosition[currentArmGoalIndex]);
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
    Serial.println("MOVING TO NEXT INSTRUCTION");
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
      //IMUUpdate();

      //velocity calculation
      long int current_Time = micros();
      double time_Passed = (current_Time - prev_Time)/1000; //convert from micros to millis
      if (time_Passed > 15){ //enough time has passed to get a decent sample of velocity
        //setting prev_time to time now as calculating is occcuring now
        prev_Time = micros();

        //find change in position  
        int M1_deltaEncPos = M1_encoderPos - prev_M1_enc_Pos;
        int M2_deltaEncPos = getM2EncPos() - prev_M2_enc_Pos;

        //save new enc pos as old for next loop
        prev_M1_enc_Pos = M1_encoderPos;
        prev_M2_enc_Pos = getM2EncPos();

        //calculate velocity and save to global variable
        M1_Vel_Save = ((double)M1_deltaEncPos / time_Passed) * 1000 * 60 / enc_Ticks_Per_Rot;
        M2_Vel_Save = ((double)M2_deltaEncPos / time_Passed) * 1000 * 60 / enc_Ticks_Per_Rot;

        //vel save has changed now calculate the rolling average again with the change
        calcRollingAvg();

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
  pos_PID.SetMode(AUTOMATIC);
  pos_PID.SetOutputLimits(-TRANSLATE_MAX_SPEED, TRANSLATE_MAX_SPEED);
  pos_PID.SetSampleTime(1);

  //Enc Pid
  enc_PID.SetMode(AUTOMATIC);
  enc_PID.SetOutputLimits(-475, 475);
  enc_PID.SetSampleTime(1);

  //Stepper setup ###########################
  // max speed tested 3200, accell 4800
  engine.init(); //Initialize the engine and set up the GPIOs

  // Attach the stepper to a step_pin
  stepper = engine.stepperConnectToPin(STEP_PIN);

  if (stepper) {
    stepper->setDirectionPin(DIR_PIN);  // Set the direction pin
    stepper->setAutoEnable(true);

    // Set acceleration and speed
    stepper->setSpeedInHz(3200/2);   // Steps per second 3200
    stepper->setAcceleration(4800/2); // Steps per second squared 4800
  }

  //Servo setup
  //commented out for now to avoid braking the servos
  // servoLink1.attach(SERVO_LINK1_PIN);
  // servoLink2.attach(SERVO_LINK2_PIN);
  // servoLink1.write(servoLink1_Starting_Angle);
  // servoLink2.write(servoLink2_Starting_Angle);

  //Action button setup
  pinMode(ACTION_BUTTON_PIN, INPUT);

  Pos_setpoint = 0;
  Enc_setpoint = 0;

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
  }

  // StepperTest();
  instructionRegisterManager();
  // brakeTest();
  // velocityTest();
  // OffsetTest();


  loopNo++;
}
