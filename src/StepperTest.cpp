// #include <FastAccelStepper.h>

// #define STEP_PIN 7 //called PUL on the TB6600
// #define DIR_PIN 6

// // Create an instance of the FastAccelStepperEngine
// FastAccelStepperEngine engine;

// // Create a pointer to the stepper
// FastAccelStepper *stepper = NULL;


// void setup() {
//   // Initialize the engine and set up the GPIOs
//   engine.init();

//   // Attach the stepper to a specific pin (for example, GPIO 33 for step and GPIO 32 for direction)
//   stepper = engine.stepperConnectToPin(STEP_PIN);

//   if (stepper) {
//     stepper->setDirectionPin(DIR_PIN);  // Set the direction pin
//     // stepper->setEnablePin(14);     // Optional: Enable pin if you have one
//     stepper->setAutoEnable(true);

//     // Set acceleration and speed
//     stepper->setSpeedInHz(3200);   // Steps per second
//     stepper->setAcceleration(4800); // Steps per second squared
//   }
// }

// void loop() {
//     stepper->move(1000);
//     delay(2000);
//     stepper->move(-1000);

//     // Optional: Add a small delay between direction changes for smoother operation
//     delay(1000); // 1 second delay

// }
