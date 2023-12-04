#include <SoftwareWire.h>

// int sdaPin = PIN_WIRE_SDA;
// int sclPin = PIN_WIRE_SCL;

int sdaPin = 19; //PD1
int sclPin = 18; //PD0

SoftwareWire myWire(sdaPin, sclPin);

// These buffers must be at least as large as the largest read or write you perform.
char swTxBuffer[16];
char swRxBuffer[16];

// I2C default address
#define TMAG5273_I2C_ADDRESS_INITIAL 0X22 

void setup() {
  
  myWire.begin();
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(A0, OUTPUT);
  pinMode(12, INPUT);
  pinMode(6, INPUT);

  // If begin is successful (0), then start example
  // if(sensor.begin(i2cAddress, myWire) == 1)
  // {
  //   Serial.println("Begin");
  // }
  // else // Otherwise, infinite loop
  // {
  //   Serial.println("Device failed to setup - Freezing code.");
  //   while(1); // Runs forever
  // }
}

void loop() {

  ReadMagnetometer();

  // put your main code here, to run repeatedly:
  int j3 = digitalRead(12);
  int j4 = digitalRead(6);
  Serial.println("J3:");
  Serial.println(j3);
  Serial.println("J4:");
  Serial.println(j4);
  Serial.println("PF7:ON");
  digitalWrite(A0, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  Serial.println("PF7:OFF");
  digitalWrite(A0, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);  
  Serial.println("Hello from ThrottleController");
}

void ReadMagnetometer()
{
    float magX = 0;
    float magY = 0;
    float magZ = 0;
    //float magX = sensor.getXData();
    //float magY = sensor.getYData();
    //float magZ = sensor.getZData();

    Serial.print("(");
    Serial.print(magX);
    Serial.print(", ");
    Serial.print(magY);
    Serial.print(", ");
    Serial.print(magZ);
    Serial.println(") mT");
}

