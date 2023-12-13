//#include <SoftwareWire.h>
#include <Wire.h>                               // Used to establish serial communication on the I2C bus
#include "SparkFun_TMAG5273_Arduino_Library.h"  // Used to send and recieve specific information from our sensor
#include <Joystick.h>
#include "Keyboard.h"

// Create the Joystick
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
                   JOYSTICK_TYPE_MULTI_AXIS, 2, 0,
                   false, false, false, false, false, false,
                   false, true, false, true, false);

// Constant that maps the physical pin to the joystick button.
const int pinToButtonMap = 9;

// Last state of the button
int lastButton1State = 0;
int lastButton2State = 0;
int lastThrottleState = 0;
int lastBrakeState = 0;

int throttleState = 0;
int throttle1Angle = 40;
int throttle2Angle = 80;
int throttle3Angle = 120;
int throttle4Angle = 160;
int throttle5Angle = 200;

int brakeState = 0;
int brake1Angle = 40;
int brake2Angle = 80;
int brake3Angle = 120;
int brake4Angle = 160;
int brake5Angle = 200;

int throttleAngleOffset = 45;
int throttleAngleCrossover = 315;
int throttleUnitsPerDegree = 5;

int brakeAngleOffset = 10;
int brakeAngleCrossover = 0;
int brakeUnitsPerDegree = 5;

// I2C default address
#define TMAG5273A1_I2C_ADDRESS_INITIAL 0X35  // I2C ADDRESS (7 MBS BITS - TMAG5273A1)

#define TMAG5273B1_I2C_ADDRESS_INITIAL 0X22  // I2C ADDRESS (7 MBS BITS - TMAG5273B1)

//#define JOYSTICK_MODE 1
#define SERIAL_MODE 1
//#define KEYBOARD_MODE 1

TMAG5273 sensor;  // Initialize hall-effect sensor
// I2C default address
volatile uint8_t i2cAddress = TMAG5273A1_I2C_ADDRESS_INITIAL;

TMAG5273 sensorThrottle;  // Initialize hall-effect sensor
// I2C default address
volatile uint8_t i2cAddressThrottle = TMAG5273B1_I2C_ADDRESS_INITIAL;

void setup() {

  Wire.begin();
// put your setup code here, to run once:
#ifdef SERIAL_MODE
  Serial.begin(115200);
#endif
  pinMode(A0, OUTPUT);
  pinMode(6, INPUT);
  pinMode(12, INPUT);

  delay(1000);
  // If begin is successful (0), then start example
  if (sensor.begin(i2cAddress, Wire) == 1) {
    sensor.setAngleEn(1);
    sensor.setMagnitudeGain(128);
#ifdef SERIAL_MODE
    Serial.println("TMAG 1 detected.");
#endif
  } else  // Otherwise, infinite loop
  {
#ifdef SERIAL_MODE
    Serial.println("Device failed to setup TMAG 1- Freezing code.");
#endif
    while (1)
      ;  // Runs forever
  }

  if (sensorThrottle.begin(i2cAddressThrottle, Wire) == 1) {
    sensorThrottle.setAngleEn(1);
    sensorThrottle.setMagnitudeGain(128);
#ifdef SERIAL_MODE
    Serial.println("TMAG 2 detected.");
#endif
  } else  // Otherwise, infinite loop
  {
#ifdef SERIAL_MODE
    Serial.println("Device failed to setup TMAG 2- Freezing code.");
#endif
    while (1)
      ;  // Runs forever
  }

  // Initialize Joystick Library
#ifdef JOYSTICK_MODE
  Joystick.begin();
#endif

#ifdef KEYBOARD_MODE
  Keyboard.begin();
#endif
}

void loop() {

  ReadMagnetometer();

  // put your main code here, to run repeatedly:
  int j3 = digitalRead(6);
  if (j3 == 1) {
#ifdef JOYSTICK_MODE
    Joystick.setButton(0, 0);
#endif
#ifdef KEYBOARD_MODE
    if (lastButton1State != 1) {
      Keyboard.print("F");
      lastButton1State = 1;
    }
#endif
  } else {
#ifdef JOYSTICK_MODE
    Joystick.setButton(0, 1);
#endif
#ifdef KEYBOARD_MODE
    if (lastButton1State != 0) {
      Keyboard.print("B");
      lastButton1State = 0;
    }
#endif
  }
  int j4 = digitalRead(12);
  if (j4 == 1) {
#ifdef JOYSTICK_MODE
    Joystick.setButton(1, 0);
#endif
#ifdef KEYBOARD_MODE
    if (lastButton2State != 1) {
      Keyboard.print("O");
      lastButton2State = 1;
    }
#endif
  } else {
#ifdef JOYSTICK_MODE
    Joystick.setButton(1, 1);
#endif
#ifdef KEYBOARD_MODE
    if (lastButton2State != 0) {
      Keyboard.print("P");
      lastButton2State = 0;
    }
#endif
  }

  delay(50);
  //digitalWrite(A0, HIGH);  // turn the LED on (HIGH is the voltage level)
  //delay(1000);                      // wait for a second
  //digitalWrite(A0, LOW);   // turn the LED off by making the voltage LOW
  //delay(1000);
}

void ReadMagnetometer() {
  // Checks if mag channels are on - turns on in setup
  if (sensor.getMagneticChannel() == 0x7)  //Check that XYZ magnetic channels are enabled.
  {
//     float x = sensor.getXData();
//     float y = sensor.getYData();
//     float z = sensor.getZData();

// #ifdef SERIAL_MODE
//   Serial.print("X: ");
//   Serial.println(x);
//   Serial.print("Y: ");
//   Serial.println(y);
//   Serial.print("Z: ");
//   Serial.println(z);
// #endif

    float throttleRaw = sensor.getAngleResult();
    int throttleInt = (int)throttleRaw;
    int throttle = throttleInt - throttleAngleOffset;
    if (throttle < 0) {
      throttle += 360;
    }
    int throttleOutput = throttle * throttleUnitsPerDegree;

#ifdef SERIAL_MODE
    Serial.print("Brake:");
    Serial.println(throttleRaw);
#endif

#ifdef JOYSTICK_MODE
    Joystick.setBrake(throttleOutput);
#endif

    if (throttle <= throttle1Angle - 5) {
      throttleState = 0;
    } else if (throttle > throttle1Angle - 5 && throttle < throttle1Angle + 5) {
      throttleState = 1;
    } else if (throttle > throttle2Angle - 5 && throttle < throttle2Angle + 5) {
      throttleState = 2;
    } else if (throttle > throttle3Angle - 5 && throttle < throttle3Angle + 5) {
      throttleState = 3;
    } else if (throttle > throttle4Angle - 5 && throttle < throttle4Angle + 5) {
      throttleState = 4;
    } else if (throttle >= throttle4Angle - 5) {
      throttleState = 5;
    }

#ifdef KEYBOARD_MODE
    if (throttleState != lastBrakeState) {
      if (throttleState > lastBrakeState) {
        Keyboard.print("]");
      } else {
        Keyboard.print("[");
      }
    }
#endif

    lastThrottleState = throttleState;
  }
  // else
  // {
  //   // If there is an issue, stop the magnetic readings and restart sensor/example
  //   //Serial.println("Mag Channels disabled, stopping..");
  // }

  // Checks if mag channels are on - turns on in setup
  if (sensorThrottle.getMagneticChannel() == 0x7)  //Check that XYZ magnetic channels are enabled.
  {
//       float x = sensorThrottle.getXData();
//       float y = sensorThrottle.getYData();
//       float z = sensorThrottle.getZData();

// #ifdef SERIAL_MODE
//       Serial.print("Xt: ");
//       Serial.println(x);
//       Serial.print("Yt: ");
//       Serial.println(y);
//       Serial.print("Zt: ");
//       Serial.println(z);
// #endif

    float throttleRaw = sensorThrottle.getAngleResult();
    int throttleInt = (int)throttleRaw;
    int throttle = 360 - throttleInt;
    //if (throttle > throttleAngleCrossover) {
    //  throttle = throttle - throttleAngleCrossover;
    //} else {
    //  throttle = throttle + throttleAngleOffset;
   // }

#ifdef SERIAL_MODE
    Serial.print("Throttle:");
    Serial.println(throttleRaw);
#endif
// if(throttle < 0)
// {
//   throttle += 360;
// }
#ifdef JOYSTICK_MODE
    int throttleOutput = throttle * throttleUnitsPerDegree;
    Joystick.setThrottle(throttle);
#endif

    if (throttle <= throttle1Angle - 5) {
      throttleState = 0;
    } else if (throttle > throttle1Angle - 5 && throttle < throttle1Angle + 5) {
      throttleState = 1;
    } else if (throttle > throttle2Angle - 5 && throttle < throttle2Angle + 5) {
      throttleState = 2;
    } else if (throttle > throttle3Angle - 5 && throttle < throttle3Angle + 5) {
      throttleState = 3;
    } else if (throttle > throttle4Angle - 5 && throttle < throttle4Angle + 5) {
      throttleState = 4;
    } else if (throttle >= throttle4Angle - 5) {
      throttleState = 5;
    }

#ifdef KEYBOARD_MODE
    if (throttleState != lastBrakeState) {
      if (throttleState > lastBrakeState) {
        Keyboard.print("+");
      } else {
        Keyboard.print("-");
      }
    }
#endif

    lastBrakeState = brakeState;
  }
  // else
  // {
  //   // If there is an issue, stop the magnetic readings and restart sensor/example
  //   //Serial.println("Mag Channels disabled, stopping..");
  // }
}
