// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
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

int16_t ax, ay, az;
int16_t gx, gy, gz;
int Fx, Fz;
int Mx, Mz;
int Sensitivity = 100;
int Mapx, Mapz;
int chogix, chogiz;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO
int16_t velocityx[2] = {0,};
int16_t accelerationx[2] = {0,};
int16_t positionx[2] = {0,};

int16_t velocityy[2] = {0,};
int16_t accelerationy[2] = {0,};
int16_t positiony[2] = {0,};

int16_t velocityz[2] = {0,};
int16_t accelerationz[2] = {0,};
int16_t positionz[2] = {0,};

#define LED_PIN 13
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    accelgyro.initialize();
    Serial.begin(9600);
    BTSerial.begin(9600);//블루투스와의 통신속도 설정
    Mapx = 0; Mapz=0;
    chogix = 0;
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if(chogix == 0){
      chogix = az;
    }
//    Fx=(gx - chogix);   //센서 초기값따라 공식 수정.
//  Fz=(gz - chogiz);   //센서 초기값따라 공식 수정.
//  Mx = Fx / Sensitivity; 
//  Mz = Fz / Sensitivity;
//  Headmove(Mx * -1, Mz * -1);
//  Serial.print("x : ");
//  Serial.print(Mapx);
//  Serial.print(", z : ");
//  Serial.println(Mapz);

    accelerationx[1] = ax;
    velocityx[1] = velocityx[0] + accelerationx[1] - accelerationx[0];
   positionx[1] = positionx[1] + positionx[0] + velocityx[1];

   accelerationy[1] = ay;
    velocityy[1] = velocityy[1] + accelerationy[1] - accelerationy[0];
   positionx[1] = positiony[1]+ positiony[0] + velocityy[1];

    accelerationz[1] = az;
    velocityx[1] = velocityz[0] + accelerationz[1] - accelerationz[0];
  positionx[1] = positionz[1] + positionz[0] + velocityz[1];
  float a = (float)ax/16384*10;
  float b = (float)ay/16384*10;
  float c = (float)az/16384*10;
    
  //  Serial.print(F("position x,y,z : "));
    Serial.print(a);
    Serial.print(F(", "));
    Serial.print(b);
    Serial.print(F(", "));
    Serial.print(c);
    Serial.println(F(""));
    
    velocityx[0] = velocityx[1];
    accelerationx[0] = accelerationx[1];
    positionx[0] = positionx[1];

    velocityz[0] = velocityz[1];
    accelerationz[0] = accelerationz[1];
    positionz[0] = positionz[1];

    velocityy[0] = velocityy[1];
    accelerationy[0] = accelerationy[1];
    positiony[0] = positiony[1];


    delay(200);
/*
    Serial.print(az); Serial.print("\t");
    if(az>0){
        BTSerial.write("11\r\n");
        delay(2000);
    }
    else if(az<=0){
      BTSerial.write("41\r\n");
        delay(2000);
      }


          accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
          Serial.print(az); Serial.print("\t");
    if(az>0){
        BTSerial.write("12\r\n");
        delay(2000);
    }
    else if(az<=0){
      BTSerial.write("42\r\n");
        delay(2000);
      }


          accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
          Serial.print(az); Serial.print("\t");
    if(az>0){
        BTSerial.write("13\r\n");
        delay(2000);
    }
    else if(az<=0){
      BTSerial.write("43\r\n");
        delay(2000);
      }


          accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
          Serial.print(az); Serial.print("\t");
    if(az>0){
        BTSerial.write("14\r\n");
        delay(2000);
    }
    else if(az<=0){
      BTSerial.write("44\r\n");
        delay(2000);
      }


          accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
          Serial.print(az); Serial.print("\t");
    if(az>0){
        BTSerial.write("15\r\n");
        delay(2000);
    }
    else if(az<=0){
      BTSerial.write("45\r\n");
        delay(2000);
      }


          accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
          Serial.print(az); Serial.print("\t");
    if(az>0){
        BTSerial.write("16\r\n");
        delay(2000);
    }
    else if(az<=0){
      BTSerial.write("46\r\n");
        delay(2000);
      }


          accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
          Serial.print(az); Serial.print("\t");
    if(az>0){
        BTSerial.write("17\r\n");
        delay(2000);
    }
    else if(az<=0){
      BTSerial.write("47\r\n");
        delay(2000);
      }
*/
}

void Headmove(int x, int z){
  Mapx = Mapx + x;
  Mapz = Mapz + z;
  }
