
#include "I2Cdev.h"
#include "MPU6050.h"
#include <SoftwareSerial.h>
 
SoftwareSerial BTSerial(2,3);
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

// gyro
int16_t ax, ay, az;
int16_t gx, gy, gz;

int toothNum = 0;
char sep[10] = "\r\n";

// pressure
int fsrPin = 0;     // the FSR and 10K pulldown are connected to a0
int fsrReading;     // the analog reading from the FSR resistor divider

// switch and vibration
int ledPin = 5;
int buttonApin = 9;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    pinMode(ledPin, OUTPUT);
    pinMode(buttonApin, INPUT_PULLUP); 

    Serial.begin(9600);
    BTSerial.begin(9600);//블루투스와의 통신속도 설정
}

void loop() {
  // gyro
    toothNum = (toothNum++ % 7) + 1;
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if(az>0x0){
        char whole[10] = "1";
        char num[10] = {'0'};
        itoa(toothNum, &num[0], 10);
        strcat(whole, num);
        strcat(whole, sep);
        BTSerial.write(whole);
        Serial.println(whole);
    }
    else if(az<=0x0){
      char whole[10] = "4";
      char num[10] = {'0'};
      itoa(toothNum, &num[0], 10);
      strcat(whole, num);
      strcat(whole, sep);
      BTSerial.write(whole);
      Serial.println(whole);
   }

  // pressure
  fsrReading = analogRead(fsrPin);  
 
  Serial.print("Analog reading = ");
  Serial.print(fsrReading);     // the raw analog reading
 
 
  if (fsrReading == 0) {
    Serial.println(" - No pressure");
  } else if (fsrReading < 10) {
    Serial.println(" - Light touch");
  } else if (fsrReading < 50) {
    Serial.println(" - Light squeeze");
  } else if (fsrReading < 150) {
    Serial.println(" - Medium squeeze");
  } else {
    Serial.println(" - Big squeeze");
  }


  // switch & vibration
  if (digitalRead(buttonApin) == LOW)
  {
    Serial.println("vibration on");
    analogWrite(6, 100);
  } else {
    analogWrite(6, -1);
  }
  
  delay(500);
}