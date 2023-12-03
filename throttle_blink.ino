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

// I2C default address
#define TMAG5273_I2C_ADDRESS_INITIAL 0X35 // I2C ADDRESS (7 MBS BITS - TMAG5273A1)
#define TMAG5273_I2C_ADDRESS_WRITE 0X6A // I2C WRITE ADDRESS (8-bit)
#define TMAG5273_I2C_ADDRESS_READ 0X6B // I2C READ ADDRESS (8-bit)

TMAG5273 sensor; // Initialize hall-effect sensor
// I2C default address
volatile uint8_t i2cAddress = TMAG5273_I2C_ADDRESS_INITIAL;

void setup() {
  
  //myWire.begin();
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
    Serial.println("Begin");
  }
  else // Otherwise, infinite loop
  {
    Serial.println("Device failed to setup - Freezing code.");
    while(1); // Runs forever
  }

  // Initialize Joystick Library
	Joystick.begin();
}

void loop() {

  ReadMagnetometer();

  // put your main code here, to run repeatedly:
   int j3 = digitalRead(6);
   if(j3 == 1)
   {
    Joystick.setButton(0, 0);
   }
   else {
    Joystick.setButton(0, 1);
   }
   int j4 = digitalRead(12);
  if(j4 == 1)
   {
    Joystick.setButton(1, 0);
   }
   else {
    Joystick.setButton(1, 1);
   }
  // Serial.print("J3:");
  // Serial.println(j3);
  // Serial.print("J4:");
  // Serial.println(j4);
  //Serial.println("PF7:ON");
  digitalWrite(A0, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  //Serial.println("PF7:OFF");
  digitalWrite(A0, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);  
  //Serial.println("Hello from ThrottleController");

  // Read pin values
	//int currentButtonState = !digitalRead(pinToButtonMap);
	//if (currentButtonState != lastButtonState)
	{
	//Joystick.setThrottle(10);
  //Joystick.setBrake(10);
	//lastButtonState = currentButtonState;
	}
}

void ReadMagnetometer()
{
   // Checks if mag channels are on - turns on in setup
    if(sensor.getMagneticChannel() == 0x7)  //Check that XYZ magnetic channels are enabled.
    {
      sensor.setTemperatureEn(true);

      float magX = sensor.getXData();
      int xCast = (int) magX;
      Joystick.setBrake(xCast);
      //float magY = sensor.getYData();
      //float magZ = sensor.getZData();
      //float temp = sensor.getTemp();

      // Serial.print("(");
      // Serial.print(magX);
      // Serial.print(", ");
      // Serial.print(magY);
      // Serial.print(", ");
      // Serial.print(magZ);
      // Serial.println(") mT");
      // Serial.print(temp);
      // Serial.println(" C");
    }
    else
    {
      // If there is an issue, stop the magnetic readings and restart sensor/example
      Serial.println("Mag Channels disabled, stopping..");
    }
}



