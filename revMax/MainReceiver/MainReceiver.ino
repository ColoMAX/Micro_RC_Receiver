//Max Suurland Technologies
//2020
/*
...............................................................................
......,&æ##©±~............»@N#N§=..............................................
....'¶#########=........X#########Ñ´........´XN#############################I..
...,N###########*......£###########É.......%################################I..
...?############ë......¶############^.....?#################################I..
...?############ë......¶############^.....I#################################I..
...?############ë......¶############^.....´¶################################I..
...?############ë......¶############^.......%###############################I..
...?############ë......¶############^..........~»???????????????????????????~..
...?############ë......¶############^..........................................
...?############ë......¶############^..........................................
...?############ë......¶############^.....^////////////////////////////'.......
...?############ë......¶############^.....##############################*....
...?############ë......¶############^.....###############################É...
...?############ë......¶############^.....################################=..
...?############ë......¶############^.....################################»..
...?############ë......¶############^.....###############################%...
...?############ë......¶############^.....#############################£´....
..............................................................................


*/
#include <Wire.h> // I2C library (for the MPU-6050 gyro /accelerometer)
#include <RF24.h> // Installed via Tools > Board > Boards Manager > Type RF24
#include <printf.h>
#include <Servo.h>
#include <statusLED.h> // https://github.com/TheDIYGuy999/statusLED
#include <TB6612FNG.h> // https://github.com/TheDIYGuy999/TB6612FNG ***NOTE*** V1.2 required!! <<<-----
#include <PWMFrequency.h> // https://github.com/TheDIYGuy999/PWMFrequency
#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library/

// Tabs (header files in sketch directory)
#include "readVCC.h"
#include "vehicleConfig.h"
#include "steeringCurves.h"
//#include "tone.h"
//#include "balancing.h"
#include "helper.h"
#include "radioUtils.h"

#define BATTERY_DETECT_PIN A7 // The 20k (to battery) & 10k (to GND) battery detection voltage divider is connected to pin A7
#define DIGITAL_OUT_1 1 // 1 = TXO Pin

const float codeVersion = 3.32; // Software revision

// Radio channels (126 channels are supported)
byte chPointer = 1; // Channel 1 (the first entry of the array) is active by default
const byte NRFchannel[] {
  1, 2
};
const uint64_t pipeIn[] = {
  0xE9E8F0F0B1LL, 0xE9E8F0F0B2LL, 0xE9E8F0F0B3LL, 0xE9E8F0F0B4LL, 0xE9E8F0F0B5LL,
  0xE9E8F0F0B6LL, 0xE9E8F0F0B7LL, 0xE9E8F0F0B8LL, 0xE9E8F0F0B9LL, 0xE9E8F0F0B0LL
};
const int maxVehicleNumber = (sizeof(pipeIn) / (sizeof(uint64_t)));


struct RcData {
  byte axis1; // Aileron (Steering for car)
  byte axis2; // Elevator
  byte axis3; // Throttle
  byte axis4; // Rudder
  boolean mode1 = false; // Mode1 (toggle speed limitation)
  boolean mode2 = false; // Mode2 (toggle acc. / dec. limitation)
  boolean momentary1 = false; // Momentary push button
  boolean momentary2 = false; //reservd button
  byte pot1; // Potentiometer
};
RcData data;

// This struct defines data, which are embedded inside the ACK payload
struct ackPayload {
  float vcc; // vehicle vcc voltage
  float batteryVoltage; // vehicle battery voltage
  boolean batteryOk = true; // the vehicle battery voltage is OK!
  byte channel = 1; // the channel number
};
ackPayload payload;

struct SystemData {
  // Motors
boolean isDriving; // is the vehicle driving?
// Headlight off delay
unsigned long millisLightOff = 0; 
// Indicators
boolean left;
boolean right;
boolean hazard;
// Engine sound
boolean engineOn = false;
};
SystemData systemdata;

Config config;

RF24 radio(7, 8);

Servo servo1; // steering
Servo servo2;  //gearbox
Servo servo3; //throttle
Servo servo4; // difflock
//Servo servo5; //

// Motor objects
TB6612FNG Motor1;
TB6612FNG Motor2;

// Status LED objects
statusLED tailLight(false); // "false" = output not inverted
statusLED headLight(false);
statusLED indicatorL(false);
statusLED indicatorR(false);
statusLED beaconLights(false);


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void setup() {
  // put your setup code here, to run once:
 Serial.begin(115200);
 // printf_begin();
delay(2000);
setupRadio();

servo1.attach(A0);
servo2.attach(A1);
servo3.attach(A2);
servo4.attach(A3);
//servo5.attach(A4);

// All axes to neutral position
  data.axis1 = 50;
  data.axis2 = 50;
  data.axis3 = 50;
  data.axis4 = 50;
  data.pot1 = 50; 

setupMotors(); //lier en takel

//pinMode
  
}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void loop() {
  // put your main code here, to run repeatedly:

}


/////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\FUNCTIONS///////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\
//
// =======================================================================================================
// RADIO SETUP
// =======================================================================================================
//

void setupRadio() {
  radio.begin();
  radio.setChannel(NRFchannel[chPointer]);

  // Set Power Amplifier (PA) level to one of four levels: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
  radio.setPALevel(RF24_PA_HIGH); // HIGH

  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(pipeIn[config.vehicleNumber - 1], true); // Ensure autoACK is enabled
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(5, 5);                  // 5x250us delay (blocking!!), max. 5 retries
  //radio.setCRCLength(RF24_CRC_8);          // Use 8-bit CRC for performance

#ifdef DEBUG
  radio.printDetails();
  Serial.println(chPointer);
  delay(3000);
#endif

  radio.openReadingPipe(1, pipeIn[config.vehicleNumber - 1]);
  radio.startListening();
}

//
// =======================================================================================================
// READ RADIO DATA
// =======================================================================================================
//

void readRadio() {

  static unsigned long lastRecvTime = 0;
  byte pipeNo;

  if (radio.available(&pipeNo)) {
    radio.writeAckPayload(pipeNo, &payload, sizeof(struct ackPayload) );  // prepare the ACK payload
    radio.read(&data, sizeof(struct RcData)); // read the radia data and send out the ACK payload
    systemdata.hazard = false;
    lastRecvTime = millis();
//#ifdef DEBUG
//    Serial.print(data.axis1);
//    Serial.print("\t");
//    Serial.print(data.axis2);
//    Serial.print("\t");
//    Serial.print(data.axis3);
//    Serial.print("\t");
//    Serial.print(data.axis4);
//    Serial.println("\t");
//#endif
  }

  // Switch channel
  if (millis() - lastRecvTime > 500) {
    chPointer ++;
    if (chPointer >= sizeof((*NRFchannel) / sizeof(byte))) chPointer = 0;
    radio.setChannel(NRFchannel[chPointer]);
    payload.channel = NRFchannel[chPointer];
  }

  if (millis() - lastRecvTime > 1000) { // set all analog values to their middle position, if no RC signal is received during 1s!
    data.axis1 = 50; // Aileron (Steering for car)
    data.axis2 = 50; // Elevator
    data.axis3 = 50; // Throttle
    data.axis4 = 50; // Rudder
    systemdata.hazard = true; // Enable hazard lights
    payload.batteryOk = true; // Clear low battery alert (allows to re-enable the vehicle, if you switch off the transmitter)
//#ifdef DEBUG
//    Serial.println("No Radio Available - Check Transmitter!");
//#endif
  }

  if (millis() - lastRecvTime > 2000) {
    setupRadio(); // re-initialize radio
    lastRecvTime = millis();
  }
}

//
// =======================================================================================================
// MOTOR DRIVER SETUP
// =======================================================================================================
//

void setupMotors( void ) {

  // TB6612FNG H-Bridge pins
  // ---- IMPORTANT ---- The pin assignment depends on your board revision and is switched here to match!
  const byte  motor1_in1 = 4;
  const byte  motor1_in2 = 9;
  const byte  motor1_pwm = 6;

  const byte  motor2_in1 = 5;
  const byte  motor2_in2 = 2;
  const byte  motor2_pwm = 3;


  // SYNTAX: IN1, IN2, PWM, min. input value, max. input value, neutral position width
  // invert rotation direction true or false
  Motor1.begin(motor1_in1, motor1_in2, motor1_pwm, 0, 100, 4, false); // Lier
  Motor2.begin(motor2_in1, motor2_in2, motor2_pwm, 0, 100, 4, false); // Takel

  // Motor PWM frequency prescalers (Requires the PWMFrequency.h library)
  // Differential steering vehicles: locked to 984Hz, to make sure, that both motors use 984Hz.
  if (config.vehicleType == 1 || config.vehicleType == 2 || config.vehicleType == 6) config.pwmPrescaler2 = 32;
  
  // ----------- IMPORTANT!! --------------
  // Motor 1 always runs @ 984Hz PWM frequency and can't be changed, because timers 0 an 1 are in use for other things!
  // Motor 2 (pin 3) can be changed to the following PWM frequencies: 32 = 984Hz, 8 = 3936Hz, 1 = 31488Hz
  setPWMPrescaler(3, config.pwmPrescaler2); // pin 3 is hardcoded, because we can't change all others anyway
}

//
// =======================================================================================================  <<                                    EDIT FROM HERE
// DRIVE MOTORS Lier en takel
// =======================================================================================================
//

void driveMotorsForklift() {

  volatile int maxPWM;
  volatile byte maxAcceleration;

  // Speed limitation (max. is 255)
  if (data.mode1) {
    maxPWM = config.maxPWMlimited; // Limited
  } else {
    maxPWM = config.maxPWMfull; // Full
  }

  if (!payload.batteryOk && config.liPo) data.axis3 = 50; // Stop the vehicle, if the battery is empty!

  // Acceleration & deceleration limitation (ms per 1 step input signal change)
  if (data.mode2) {
    maxAcceleration = config.maxAccelerationLimited; // Limited
  } else {
    maxAcceleration = config.maxAccelerationFull; // Full
  }

  // ***************** Note! The ramptime is intended to protect the gearbox! *******************
  // SYNTAX: Input value, max PWM, ramptime in ms per 1 PWM increment
  // false = brake in neutral position inactive


  if (Motor1.drive(data.axis3, config.minPWM, maxPWM, maxAcceleration, true) ) { // The drive motor (function returns true, if not in neutral)
    systemdata.millisLightOff = millis(); // Reset the headlight delay timer, if the vehicle is driving!
  }
  Motor2.drive(data.axis2, 0, config.steeringTorque, 0, false); // The fork lifting motor (the steering is driven by servo 1)
}

//

//
// =======================================================================================================
// WRITE SERVO POSITIONS
// =======================================================================================================
//

void writeServos() {
  // Aileron or Steering
    servo1.write(map(data.axis1, 100, 0, config.lim1L, config.lim1R) ); // 45 - 135°

  // Elevator or shifting gearbox actuator
    if (data.axis2 < 10)servo2.write(config.lim2R);
    else if (data.axis2 > 90)servo2.write(config.lim2L);
    else servo2.write(config.lim2C);
  
  // Throttle (for ESC control, if you don't use the internal TB6612FNG motor driver)
  if (data.mode1) { // limited speed!                                                                                 add mass simulation
    servo3.write(map(data.axis3, 100, 0, config.lim3Llow, config.lim3Rlow ) ); // less than +/- 45°
  }
  else { // full speed!
    servo3.write(map(data.axis3, 100, 0, config.lim3L, config.lim3R) ); // 45 - 135°
  }

    servo4.write(map(data.axis4, 100, 0, config.lim4L, config.lim4R) ); // 45 - 135°

//        if (data.momentary1) servo4.write(lim4L);
//    else servo4.write(lim4R);

}


//
// =======================================================================================================
// CHECK RX BATTERY & VCC VOLTAGES
// =======================================================================================================
//


void checkBattery() {

  // switch between load and no load contition
  if (millis() - systemdata.millisLightOff >= 1000) { // one s after the vehicle did stop
    systemdata.isDriving = false; // no load
  }
  else {
    systemdata.isDriving = true; // under load
  }

  // Every 1000 ms, take measurements
  static unsigned long lastTrigger;
  if (millis() - lastTrigger >= 1000) {
    lastTrigger = millis();

    // Read both averaged voltages
    payload.batteryVoltage = batteryAverage();
    payload.vcc = vccAverage();

    if (battSense) { // Observe battery voltage
      if (payload.batteryVoltage <= cutoffVoltage) payload.batteryOk = false;
    }
    else { // Observe vcc voltage
      if (payload.vcc <= cutoffVoltage) payload.batteryOk = false;
    }
  }
}

// Voltage read & averaging subfunctions -----------------------------------------
// vcc ----
float vccAverage() {
  static int raw[6];

  if (raw[0] == 0) {
    for (int i = 0; i <= 5; i++) {
      raw[i] = readVcc(); // Init array
    }
  }

  raw[5] = raw[4];
  raw[4] = raw[3];
  raw[3] = raw[2];
  raw[2] = raw[1];
  raw[1] = raw[0];
  raw[0] = readVcc();
  float average = (raw[0] + raw[1] + raw[2] + raw[3] + raw[4] + raw[5]) / 6000.0;
  return average;
}

// battery ----
float batteryAverage() {
  static int raw[6];

  if (!battSense) return 0;

  if (raw[0] == 0) {
    for (int i = 0; i <= 5; i++) {
      raw[i] = analogRead(BATTERY_DETECT_PIN); // Init array
    }
  }

  raw[5] = raw[4];
  raw[4] = raw[3];
  raw[3] = raw[2];
  raw[2] = raw[1];
  raw[1] = raw[0];
  if (isDriving && HP) raw[0] = (analogRead(BATTERY_DETECT_PIN) + 31); // add 0.3V while driving (HP version only): 1023 steps * 0.3V / 9.9V = 31
  else raw[0] = analogRead(BATTERY_DETECT_PIN); // else take the real voltage (compensates voltage drop while driving)
  float average = (raw[0] + raw[1] + raw[2] + raw[3] + raw[4] + raw[5]) / 619.999; // 1023steps / 9.9V * 6 = 619.999
  return average;
}
