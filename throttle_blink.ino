//#include <SoftwareWire.h>
#include <Wire.h>            // Used to establish serial communication on the I2C bus
#include "SparkFun_TMAG5273_Arduino_Library.h" // Used to send and recieve specific information from our sensor
#include <Joystick.h>

// Create the Joystick
Joystick_ Joystick;

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


TMAG5273 sensor; // Initialize hall-effect sensor
// I2C default address
volatile uint8_t i2cAddress = TMAG5273A1_I2C_ADDRESS_INITIAL;

TMAG5273 sensorThrottle; // Initialize hall-effect sensor
// I2C default address
volatile uint8_t i2cAddressThrottle = TMAG5273B1_I2C_ADDRESS_INITIAL;

void setup() {
  
  Wire.begin();
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(A0, OUTPUT);
  pinMode(6, INPUT);
  pinMode(12, INPUT);

  delay(1000);  
  // If begin is successful (0), then start example
  if(sensor.begin(i2cAddress, Wire) == 1)
  {
    sensor.setAngleEn(1);
    Serial.println("TMAG 1 detected.");
  }
  else // Otherwise, infinite loop
  {
    Serial.println("Device failed to setup TMAG 1- Freezing code.");
    while(1); // Runs forever
  }

  if(sensorThrottle.begin(i2cAddressThrottle, Wire) == 1)
  {
    sensorThrottle.setAngleEn(1);
    Serial.println("TMAG 2 detected.");
  }
  else // Otherwise, infinite loop
  {
    Serial.println("Device failed to setup TMAG 2- Freezing code.");
    while(1); // Runs forever
  }

  // Initialize Joystick Library
	//Joystick.begin();
}

void loop() {

  ReadMagnetometer();

  // put your main code here, to run repeatedly:
   int j3 = digitalRead(6);
   if(j3 == 1)
   {
    //Joystick.setButton(0, 0);
   }
   else {
    //Joystick.setButton(0, 1);
   }
   int j4 = digitalRead(12);
  if(j4 == 1)
   {
    //Joystick.setButton(1, 0);
   }
   else {
    //Joystick.setButton(1, 1);
   }

  digitalWrite(A0, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(A0, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);  

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
      Serial.print("Brake:");
      Serial.println(brake);
      //Joystick.setBrake(brakeOutput);
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
      Serial.print("Throttle:");
      Serial.println(throttle);
      // if(throttle < 0)
      // {
      //   throttle += 360;
      // }
      // int throttleOutput = throttle * throttleUnitsPerDegree;
      //Joystick.setThrottle(throttle);
    }
    // else
    // {
    //   // If there is an issue, stop the magnetic readings and restart sensor/example
    //   //Serial.println("Mag Channels disabled, stopping..");
    // }
}



