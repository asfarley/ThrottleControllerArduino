//#include <SoftwareWire.h>
#include <Wire.h>            // Used to establish serial communication on the I2C bus
#include "SparkFun_TMAG5273_Arduino_Library.h" // Used to send and recieve specific information from our sensor
#include <Joystick.h>

// Create the Joystick
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_MULTI_AXIS, 2, 0,
  false, false, false, false, false, false,
  false, true, false, true, false);

// Constant that maps the physical pin to the joystick button.
const int pinToButtonMap = 9;

// Last state of the button
int lastButtonState = 0;

int throttleAngleOffset = 45;
int throttleAngleCrossover = 315;
int throttleUnitsPerDegree = 5;

int brakeAngleCalibrationOffset = 10;
int brakeAngleCrossover = 0;
int brakeUnitsPerDegree = 5;

// I2C default address
#define TMAG5273A1_I2C_ADDRESS_INITIAL 0X35 // I2C ADDRESS (7 MBS BITS - TMAG5273A1)

#define TMAG5273B1_I2C_ADDRESS_INITIAL 0X22 // I2C ADDRESS (7 MBS BITS - TMAG5273B1)

#define JOYSTICK_MODE 1
//#define SERIAL_MODE 1

TMAG5273 sensor; // Initialize hall-effect sensor
// I2C default address
volatile uint8_t i2cAddress = TMAG5273A1_I2C_ADDRESS_INITIAL;

TMAG5273 sensorThrottle; // Initialize hall-effect sensor
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
  if(sensor.begin(i2cAddress, Wire) == 1)
  {
    sensor.setAngleEn(1);
#ifdef SERIAL_MODE
    Serial.println("TMAG 1 detected.");
#endif
  }
  else // Otherwise, infinite loop
  {
#ifdef SERIAL_MODE
    Serial.println("Device failed to setup TMAG 1- Freezing code.");
#endif
    while(1); // Runs forever
  }

  if(sensorThrottle.begin(i2cAddressThrottle, Wire) == 1)
  {
    sensorThrottle.setAngleEn(1);
#ifdef SERIAL_MODE
    Serial.println("TMAG 2 detected.");
#endif
  }
  else // Otherwise, infinite loop
  {
#ifdef SERIAL_MODE
    Serial.println("Device failed to setup TMAG 2- Freezing code.");
#endif
    while(1); // Runs forever
  }

  // Initialize Joystick Library
#ifdef JOYSTICK_MODE
	Joystick.begin();
#endif
}

void loop() {

  ReadMagnetometer();

  // put your main code here, to run repeatedly:
   int j3 = digitalRead(6);
   if(j3 == 1)
   {
  #ifdef JOYSTICK_MODE
    Joystick.setButton(0, 0);
  #endif
   }
   else {
#ifdef JOYSTICK_MODE
    Joystick.setButton(0, 1);
#endif
   }
   int j4 = digitalRead(12);
  if(j4 == 1)
   {
#ifdef JOYSTICK_MODE
    Joystick.setButton(1, 0);
#endif
   }
   else {
#ifdef JOYSTICK_MODE
    Joystick.setButton(1, 1);
#endif
   }

  delay(50);
  //digitalWrite(A0, HIGH);  // turn the LED on (HIGH is the voltage level)
  //delay(1000);                      // wait for a second
  //digitalWrite(A0, LOW);   // turn the LED off by making the voltage LOW
  //delay(1000);  

}

void ReadMagnetometer()
{
   // Checks if mag channels are on - turns on in setup
    if(sensor.getMagneticChannel() == 0x7)  //Check that XYZ magnetic channels are enabled.
    {
      float brakeRaw = sensor.getAngleResult();
      int brakeInt = (int) brakeRaw;
      int brake = brakeInt - brakeAngleCalibrationOffset;
      if(brake < 0)
      {
        brake += 360;
      }
      int brakeOutput = brake * brakeUnitsPerDegree;

#ifdef SERIAL_MODE
      Serial.print("Brake:");
      Serial.println(brake);
#endif
      
      #ifdef JOYSTICK_MODE
      Joystick.setBrake(brakeOutput);
      #endif
    }
    // else
    // {
    //   // If there is an issue, stop the magnetic readings and restart sensor/example
    //   //Serial.println("Mag Channels disabled, stopping..");
    // }

       // Checks if mag channels are on - turns on in setup
    if(sensorThrottle.getMagneticChannel() == 0x7)  //Check that XYZ magnetic channels are enabled.
    {
      float throttleRaw = sensorThrottle.getAngleResult();
      int throttleInt = (int) throttleRaw;
      int throttle = 360 - throttleInt;
      if(throttle > throttleAngleCrossover)
      {
        throttle = throttle - throttleAngleCrossover;
      }
      else
      {
        throttle = throttle + throttleAngleOffset;
      }

#ifdef SERIAL_MODE
      Serial.print("Throttle:");
      Serial.println(throttle);
#endif
      // if(throttle < 0)
      // {
      //   throttle += 360;
      // }
      #ifdef JOYSTICK_MODE
      int throttleOutput = throttle * throttleUnitsPerDegree;
      Joystick.setThrottle(throttle);
      #endif
    }
    // else
    // {
    //   // If there is an issue, stop the magnetic readings and restart sensor/example
    //   //Serial.println("Mag Channels disabled, stopping..");
    // }
}



