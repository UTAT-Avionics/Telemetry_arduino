#include <Arduino.h> // TODO: not sure if this is needed
#include <SD.h>

#include "MPU9250.h"

/* Notes:
 *  1. SD card communication occurs over SPI hardware pins. The SD cards needs to have a FAT32 / FAT16 filesystem loaded.
 *  2. EEPROM is size for micro is 1KB
 *  3. SD card library is slow, make it faster by following the follows https://forum.arduino.cc/index.php?topic=49649.0
 *  4. The SD library is huge at 13 kB, use a smaller more customized library for the SD card
 *  5. is the SD card a micro and mini, it depends on what it and the shield is to define
 *  6. files on SD library must use 8.3 convention 3 char extension, 8 char name
 *      https://www.arduino.cc/en/Reference/SDCardNotes
 *  7. Arudino uses AVR libc in user space, look its reference manual if needed.
 *  8. Arduino has no protection mechanisms as seen in normal OSes. You are running in kernel mode
 *     be careful and don't run out of SRAM
 *  9. Arduino sends data to computer at 38400 bit Hz 
 *
 * TODO: What is the maximum SPI speed for the SD card
 *
 *  Arduino Pro Micro
 *  8 MHz crystal 
 *  ATmega32U4
 *  20 digital I/O pins
 *  2.5K sram
 *  1K EEPROM
 *  
 *  RX - pin 0 for hardware serial (Serial1) UART
 *  TX - pin 1 for hardware serial
 *
 * Arduino Pro Micro
 * 
 * I2C 2, 3 (SDA, SCL)
 *
 * Connections
 * 2 I2C devices, IMU and baro (master read slave write)
 * 1 Serial device to communicate with SD card (master write slave read)
 */


#define SerialDebug true  // Set to true to get Serial output for debugging with computer

#define ArduinoUno
#define LOGFILE 'LOG.TXT' // TODO: check if this is valid 8.3 for SD card

int RX; // 0
int TX = 1;

#ifdef ArduinoUno
  // SPI pins (for SD)
  int SCLK = 13;
  int MISO = 12;
  int MOSI = 11;
  // TODO: I don't think we need an SS pin
  int SS = 10;

  // I2C pins (for coms with IMU and baro)
  int SDA = 27;
  int SCL = 28;

  // interrupt 
  int intPin = 2;
#else
  // arduino pro micro pin defs, same as before
  int SCLK = 15;
  int MISO = 14;
  int MOSI = 16;
  // TODO: I don't think we need an SS pin
  int SS = 10;

  int SDA = 2;
  int SCL = 3;

  // pin 3 maps to interrupt 0
  // pin 2 is interrupt 1
  // pin 0 is interrupt 2
  // pin 1 is interrupt 3
  // pin 7 is interrupt 4
  int intPin = 7; // interrupt 4
#endif

MPU9250 myIMU;

File log; // the log file

void setup()
{
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(38400);
  // Wait for Serial connection
  while(!Serial) {;}

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  pinMode(SS, OUTPUT);
  digitalWrite(intPin, LOW);

  // initialize SD card

  int sdopen = SD.begin(SS);

  if (sdopen) {
    log = SD.open(LOGFILE, FILE_WRITE);
  }

  if (!sdopen || !log) {
    // SD card initialization failed
    Serial.println("SD card initialization failed.");
    while(1) {;}
    // TODO: maybe flash LED to indicate failures
  }
  Serial.println("SD card initialized.");

  // Read the WHO_AM_I register, this is a good test of communication
  // TODO: What is in the WHO_AM_I register
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
    Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
    Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    // TODO: what is this calibration, how is done and what does it entail?
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM ");
    Serial.print(d, HEX);
    Serial.print(" I should be ");
    Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    Serial.println("AK8963 mag biases (mG)");
    Serial.println(myIMU.magBias[0]);
    Serial.println(myIMU.magBias[1]);
    Serial.println(myIMU.magBias[2]);

    Serial.println("AK8963 mag scale (mG)");
    Serial.println(myIMU.magScale[0]);
    Serial.println(myIMU.magScale[1]);
    Serial.println(myIMU.magScale[2]);
    delay(2000); // Add delay to see results before serial spew of data

    if(SerialDebug)
    {
      Serial.println("Magnetometer:");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}



void loop()
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  myIMU.delt_t = millis() - myIMU.count;
  if (myIMU.delt_t > 500)
  {
    // Write format
    // millis: x acceleration in milligs ,y , z, x gyro, y-gyro, z-gyro,
    // x mag, y mag, z mag
    char buf[15];
    log.print(millis()); log.print(":"); 

    log.print(1000 * dtostrf(myIMU.ax, 15, 5, buf));
    log.print(1000 * dtostrf(myIMU.ay, 15, 5, buf));
    log.print(1000 * dtostrf(myIMU.az, 15, 5, buf));

    log.print(dtostrf(myIMU.gx, 15, 5, buf));
    log.print(dtostrf(myIMU.gy, 15, 5, buf));
    log.print(dtostrf(myIMU.gz, 15, 5, buf));

    log.print(dtostrf(myIMU.mx, 15, 5, buf));
    log.print(dtostrf(myIMU.my, 15, 5, buf));
    log.print(dtostrf(myIMU.mz, 15, 5, buf));

    log.print('\n');

    log.flush();

    if(SerialDebug)
    {
      // Print acceleration values in milligs!
      Serial.print("X-acceleration: "); Serial.print(1000 * myIMU.ax);
      Serial.print(" mg ");
      Serial.print("Y-acceleration: "); Serial.print(1000 * myIMU.ay);
      Serial.print(" mg ");
      Serial.print("Z-acceleration: "); Serial.print(1000 * myIMU.az);
      Serial.println(" mg ");

      // Print gyro values in degree/sec
      Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
      Serial.print(" degrees/sec ");
      Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
      Serial.print(" degrees/sec ");
      Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
      Serial.println(" degrees/sec");

      // Print mag values in degree/sec
      Serial.print("X-mag field: "); Serial.print(myIMU.mx);
      Serial.print(" mG ");
      Serial.print("Y-mag field: "); Serial.print(myIMU.my);
      Serial.print(" mG ");
      Serial.print("Z-mag field: "); Serial.print(myIMU.mz);
      Serial.println(" mG");

      myIMU.tempCount = myIMU.readTempData();  // Read the adc values
      // Temperature in degrees Centigrade
      myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
      // Print temperature in degrees Centigrade
      Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
      Serial.println(" degrees C");
    }

    myIMU.count = millis();
  } // if (myIMU.delt_t > 500)

}
