#include <Wire.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(2, 3);
void get6050();
void writeBT();

const int MPU = 0x68;               //MPU6050 I2C주소
int AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

// number of tooth
int toothNum = 0;

// pressure
int pressurePin = 0;     // the pressure and 10K pulldown are connected to a0
int pressureReading;     // the analog reading from the pressure resistor divider

// switch and vibration
int ledPin = 5;
int buttonApin = 9;

// brush order 
int brush[100]    = { 11, 21, 31, 41, 11, 12, 13, 14, 15, 16, 17, 16, 15, 14, 13, 12, 11, 21, 22, 23, 24, 25, 26, 27, 26, 25, 24, 23, 22, 21, 31, 32, 33, 34, 35, 36, 37, 36, 35, 34, 33, 32, 31, 41, 42, 43, 44, 45, 46, 47, 46, 45, 44, 43, 42, 41 };
int duration[100] = {  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1 }; 
int brushCounter = 0;

// Bluetooth write interval
int interval = 1000;

void setup() {
    pinMode(ledPin, OUTPUT);
    pinMode(buttonApin, INPUT_PULLUP); 

    Serial.begin(9600);
    BTSerial.begin(9600);//블루투스와의 통신속도 설정
}

void loop() {
  // switch & vibration
  if (digitalRead(buttonApin) == LOW)
  {
    Serial.println("vibration on");
    analogWrite(6, 100);
  } else {
    analogWrite(6, -1);
  }
  // pressure
  pressureReading = analogRead(pressurePin);  
 
  Serial.print("Analog reading = ");
  Serial.print(pressureReading);     // the raw analog reading

  if (pressureReading < 10) {
    Serial.println(" - No pressure");
    writeBT(0);
  }
  else {
    Serial.println(" - pressure");
    toothNum = (toothNum++ % 7) + 1;

    if (duration[brushCounter] > 0) {
      writeBT(brush[brushCounter]);
      duration[brushCounter]--;
    } else {
      brushCounter++;
      return;
    }
  }
  delay(interval);
}

void writeBT(int tooth) {
  char whole[10] = "<";
  char num[10] = {'0'};
  char sep[10] = ">";
  itoa(tooth, &num[0], 10);
  strcat(whole, num);
  strcat(whole, sep);
  
  BTSerial.write(whole);
  Serial.println(whole);
}

// read raw accel/gyro measurements from device
void get6050(){
  Wire.beginTransmission(MPU);        //MPU6050 호출
  Wire.write(0x3B);//AcX 레지스터 위치 요청
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);    //14byte의 데이터를 요청
  AcX = Wire.read() << 8|Wire.read(); //두개의 나뉘어진 바이트를 하나로 이어붙입니다.
  AcY = Wire.read() << 8|Wire.read();
  AcZ = Wire.read() << 8|Wire.read();
  Tmp = Wire.read() << 8|Wire.read();
  GyX = Wire.read() << 8|Wire.read();
  GyY = Wire.read() << 8|Wire.read();
  GyZ = Wire.read() << 8|Wire.read();
}
