#ifndef VEHICLECONFIGH
#define VEHICLECONFIGH

// Battery type
struct Config{
boolean liPo = false; // LiPo is protected by ESC
float cutoffVoltage = 4.0; // 5V supply

// Board type
float boardVersion = 1.5;
boolean HP = false;

// Vehicle address
int vehicleNumber = 1;

// Vehicle type
byte vehicleType = 0;

// Lights
boolean escBrakeLights = false;
boolean tailLights = false;
boolean headLights = false;
boolean indicators = false;
boolean beacons = false;

// Servo limits
byte lim1L = 155, lim1R = 65; // Steering R 155, L 65
byte lim2L = 71, lim2C = 106, lim2R = 146; // 3 speed gearbox shifting servo 71 = 3. gear, 106 2. gear, 146 = 1. gear.
byte lim3L = 135, lim3R = 45; // ESC
byte lim3Llow = 105, lim3Rlow = 75; // limited top speed ESC angles!
byte lim4L = 45, lim4R = 135; // Controlled by pot, for sound triggering!
#define THREE_SPEED_GEARBOX // Vehicle has a mechanical 3 speed shifting gearbox, switched by servo CH2.
// Not usable in combination with the "tailLights" option

// lier configuration
int maxPWMfull = 255;
int maxPWMlimited = 170;
int minPWM = 0;
byte maxAccelerationFull = 7;
byte maxAccelerationLimited = 12;

// Variables for self balancing (vehicleType = 4) only!
float tiltCalibration = 0.0;

// Steering configuration
byte steeringTorque = 255;

// Motor 2 PWM frequency
byte pwmPrescaler2 = 8; // 3936Hz

// Additional Channels
boolean TXO_momentary1 = true;
boolean TXO_toggle1 = false;
boolean potentiometer1 = true;

// Engine sound
boolean engineSound = false;

// Tone sound
boolean toneOut = false;
};


#endif
