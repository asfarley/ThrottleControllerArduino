//#include <SoftwareWire.h>
#include <Wire.h>            // Used to establish serial communication on the I2C bus
#include "SparkFun_TMAG5273_Arduino_Library.h" // Used to send and recieve specific information from our sensor

// int sdaPin = PIN_WIRE_SDA;
// int sclPin = PIN_WIRE_SCL;

//#define TMAG5273_DEVICE_ID_VALUE 0x5449   // Value found in the device ID register

//int sdaPin = 19; //PD1
//int sclPin = 18; //PD0
//int ledPin = 36; // PF7/A0

//SoftwareWire myWire(2, 3);

// These buffers must be at least as large as the largest read or write you perform.
// char swTxBuffer[16];
// char swRxBuffer[16];

// I2C default address
#define TMAG5273_I2C_ADDRESS_INITIAL 0X35 // I2C ADDRESS (7 MBS BITS - TMAG5273A1)
#define TMAG5273_I2C_ADDRESS_WRITE 0X6A // I2C WRITE ADDRESS (8-bit)
#define TMAG5273_I2C_ADDRESS_READ 0X6B // I2C READ ADDRESS (8-bit)

TMAG5273 sensor; // Initialize hall-effect sensor
// I2C default address
uint8_t i2cAddress = TMAG5273_I2C_ADDRESS_INITIAL;

void setup() {
  
  //myWire.begin();
  Wire.begin();
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(A0, OUTPUT);
  pinMode(6, INPUT);
  pinMode(12, INPUT);

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
}

void loop() {

  ReadMagnetometer();

  // put your main code here, to run repeatedly:
  int j3 = digitalRead(6);
  int j4 = digitalRead(12);
  Serial.print("J3:");
  Serial.println(j3);
  Serial.print("J4:");
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
   // Checks if mag channels are on - turns on in setup
    if(sensor.getMagneticChannel() != 0) 
    {
      sensor.setTemperatureEn(true);

      float magX = sensor.getXData();
      float magY = sensor.getYData();
      float magZ = sensor.getZData();
      float temp = sensor.getTemp();

      Serial.print("(");
      Serial.print(magX);
      Serial.print(", ");
      Serial.print(magY);
      Serial.print(", ");
      Serial.print(magZ);
      Serial.println(") mT");
      Serial.print(temp);
      Serial.println(" C");
    }
    else
    {
      // If there is an issue, stop the magnetic readings and restart sensor/example
      Serial.println("Mag Channels disabled, stopping..");
    }
}

// int8_t isConnected()
// {
//     // make sure the TMAG will acknowledge over I2C
//     myWire.beginTransmission(TMAG5273_I2C_ADDRESS_INITIAL);
//     if (myWire.endTransmission() != 0)
//     {
//         return -1;
//     }

//     if (getManufacturerID() != TMAG5273_DEVICE_ID_VALUE)
//     {
//         return -1;
//     }

//     return 0;
// }

// /// @brief Returns the 8-Bit Manufacturer ID. There are two
// ///  registers, LSB and MSB.
// ///     MANUFACTURER_ID_LSB
// ///     MANUFACTURER_ID_MSB
// /// @return 16-Bit Manufacturer ID
// uint16_t getManufacturerID()
// {
//     uint16_t deviceIDReg = 0;
//     uint8_t databuffer[2];

//     readRegisters(MANUFACTURER_ID_LSB, databuffer, 2);

//     deviceIDReg = (databuffer[1] << 8) | (databuffer[0]);

//     return deviceIDReg;
// }

// /// @brief Reads the register bytes from the sensor when called upon. This reads
// /// 2 bytes of information from the 16-bit register
// /// @param regAddress Register's address to read from
// /// @param dataBuffer Pointer to the data location being read to
// /// @param numBytes Number of bytes to read
// /// @return Error code (0 is success, negative is failure)
// int8_t readRegisters(uint8_t regAddress, uint8_t *dataBuffer, uint8_t numBytes)
// {
//     // uint8_t _deviceAddress = 0X22;
//     //  Jump to desired register address
//     myWire.beginTransmission(I2C_ADDRESS);
//     myWire.write(regAddress);
//     if (myWire.endTransmission())
//     {
//         return -1;
//     }

//     // Read bytes from these registers
//     myWire.requestFrom(I2C_ADDRESS, numBytes);

//     // Store all requested bytes
//     for (uint8_t i = 0; i < numBytes && myWire.available(); i++)
//     {
//         dataBuffer[i] = myWire.read();
//     }

//     return 0;
// }

// /// @brief Reads a register region from a device.
// /// @param regAddress I2C address of device
// /// @param dataBuffer Pointer to byte to store read data
// /// @param numBytes Number of bytes to write
// /// @return Error code (0 is success, negative is failure, positive is warning)
// int8_t writeRegisters(uint8_t regAddress, uint8_t *dataBuffer, uint8_t numBytes)
// {
//     // uint8_t _deviceAddress = 0X22;
//     //  Begin transmission
//     myWire.beginTransmission(TMAG5273_I2C_ADDRESS_INITIAL);

//     // Write the address
//     myWire.write(regAddress);

//     // Write all the data
//     for (uint8_t i = 0; i < numBytes; i++)
//     {
//         myWire.write(dataBuffer[i]);
//     }

//     // End transmission
//     if (myWire.endTransmission())
//     {
//         return -1;
//     }

//     return 0;
// }

// /// @brief Reads the register byte from the sensor when called upon.
// /// @param regAddress Register's address to read from
// /// @return Value of the register chosen to be read from
// uint8_t readRegister(uint8_t regAddress)
// {
//     uint8_t regVal = 0;
//     readRegisters(regAddress, &regVal, 2);
//     return regVal;
// }

// /// @brief Reads a register region from a device.
// /// @param regAddress I2C address of device
// /// @param data Value to fill register with
// /// @return Error code (0 is success, negative is failure, positive is warning)
// uint8_t writeRegister(uint8_t regAddress, uint8_t data)
// {
//     // Write 1 byte to writeRegisters()
//     writeRegisters(regAddress, &data, 1);
//     return data;
// }

// float getXData()
// {
//     int8_t xLSB = readRegister(X_LSB_RESULT);
//     int8_t xMSB = readRegister(X_MSB_RESULT);

//     // Variable to store full X data
//     int16_t xData = 0;
//     // Combines the two in one register where the MSB is shifted to the correct location
//     xData = xLSB + (xMSB << 8);

//     // Reads to see if the range is set to 40mT or 80mT
//     uint8_t rangeValXY = getXYAxisRange();
//     uint8_t range = 0;
//     if (rangeValXY == 0)
//     {
//         range = 40;
//     }
//     else if (rangeValXY == 1)
//     {
//         range = 80;
//     }

//     // 16-bit data format equation
//     float div = 32768;
//     float xOut = -(range * xData) / div;

//     return xOut;
// }

// /// @brief Reads back the Y-Channel data conversion results, the
// ///  MSB 8-Bits and LSB 8-Bits. This reads from the following registers:
// ///     Y_MSB_RESULT and Y_LSB_RESULT
// /// @return Y-Channel data conversion results
// float getYData()
// {
//     int8_t yLSB = 0;
//     int8_t yMSB = 0;

//     yLSB = readRegister(Y_LSB_RESULT);
//     yMSB = readRegister(Y_MSB_RESULT);

//     // Variable to store full Y data
//     int16_t yData = 0;
//     // Combines the two in one register where the MSB is shifted to the correct location
//     yData = yLSB + (yMSB << 8);

//     // Reads to see if the range is set to 40mT or 80mT
//     uint8_t rangeValXY = getXYAxisRange();
//     uint8_t range = 0;
//     if (rangeValXY == 0)
//     {
//         range = 40;
//     }
//     else if (rangeValXY == 1)
//     {
//         range = 80;
//     }

//     // 16-bit data format equation
//     float div = 32768;
//     float yOut = (range * yData) / div;

//     return yOut;
// }

// /// @brief Reads back the Z-Channel data conversion results, the
// ///  MSB 8-Bits and LSB 8-Bits. This reads from the following registers:
// ///     Z_MSB_RESULT and Z_LSB_RESULT
// /// @return Z-Channel data conversion results.
// float getZData()
// {
//     int8_t zLSB = 0;
//     int8_t zMSB = 0;

//     zLSB = readRegister(Z_LSB_RESULT);
//     zMSB = readRegister(Z_MSB_RESULT);

//     // Variable to store full X data
//     int16_t zData = 0;
//     // Combines the two in one register where the MSB is shifted to the correct location
//     zData = zLSB + (zMSB << 8); 

//     // Reads to see if the range is set to 40mT or 80mT
//     uint8_t rangeValZ = getZAxisRange();
//     uint8_t range = 0;
//     if (rangeValZ == 0)
//     {
//         range = 40;
//     }
//     else if (rangeValZ == 1)
//     {
//         range = 80;
//     }

//     // div = (2^16) / 2    (as per the datasheet equation 10)
//     // 16-bit data format equation
//     float div = 32768;
//     float zOut = (range * zData) / div;

//     return zOut;
// }

// /// @brief Returns the X and Y axes magnetic range from the
// ///  two following options:
// ///     0X0 = ±40mT, DEFAULT
// ///     0X1 = ±80mT
// ///     TMAG5273_REG_SENSOR_CONFIG_2 - bit 1
// /// @return X and Y axes magnetic range (0 or 1)
// uint8_t getXYAxisRange()
// {
//     uint8_t XYrangeReg = 0;
//     XYrangeReg = readRegister(SENSOR_CONFIG_2);

//     uint8_t axisRange = bitRead(XYrangeReg, 1);

//     if (axisRange == 0)
//     {
//         return 0;
//     }
//     else if (axisRange == 1)
//     {
//         return 1;
//     }

//     return 0;
// }

// /// @brief Returns the Z axis magnetic range from the
// ///  two following options:
// ///     0X0 = ±40mT, DEFAULT
// ///     0X1 = ±80mT
// ///     TMAG5273_REG_SENSOR_CONFIG_2 - bit 0
// /// @return Z axis magnetic range from ±40mT or ±80mT
// uint8_t getZAxisRange()
// {
//     uint8_t ZrangeReg = 0;
//     ZrangeReg = readRegister(SENSOR_CONFIG_2);

//     uint8_t ZaxisRange = bitRead(ZrangeReg, 0);

//     if (ZaxisRange == 0)
//     {
//         return 0;
//     }
//     else if (ZaxisRange == 1)
//     {
//         return 1;
//     }

//     return 0;
// }
