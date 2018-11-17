#include <Wire.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(2, 3);
void get6050();
void writeBT(int tooth, int checksum, int pressure);

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
int brush[100]    = { 11, 21, -1, 41, 11, 12, 13, -1, 15, 16, 17, 16, 15, 14, 13, 12, 11, 21, 22, 23, 24, 25, 26, 27, 26, 25, 24, 23, 22, 21, 31, 32, 33, 34, 35, 36, 37, 36, 35, 34, 33, 32, 31, 41, 42, 43, 44, 45, 46, 47, 46, 45, 44, 43, 42, 41 };
int duration[100] = {  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7 }; 
//int duration[100] = {  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1 }; 
//int duration[100] = {  1,  1,  1,  1, 0 };
int duration_copy[100] = { 0 };
int brushCounter = 0;
int total = 0;

// Bluetooth write interval
int interval = 1000;

void setup() {
    pinMode(ledPin, OUTPUT);
    pinMode(buttonApin, INPUT_PULLUP); 

    Serial.begin(9600);
    BTSerial.begin(9600);//블루투스와의 통신속도 설정

    for (int i = 0; i < 100; i++) {
      if ( duration[i] >  0 ) {
        total++;
        duration_copy[i] = duration[i];
      }
    }
}

void loop() {
  int currentTooth = -1;
  int pressure = -1;
  int checksum = -1;
  
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

  // do nothing if end of array 
  if (brushCounter > total) {
    for (int i = 0; i < total; i++) {
      duration[i] = duration_copy[i];
    }
    brushCounter = 0;
    return;
  }
    
  Serial.print("Analog reading = ");
  Serial.print(pressureReading);     // the raw analog reading

  if (pressureReading < 300)
    pressure = 1;
  else if (pressureReading < 700)
    pressure = 2;
  else 
    pressure = 3;

// uncomment to read pressure to send
//  if (pressureReading < 10) {
//    Serial.println(" - No pressure");
//    writeBT(-1);
//  }
//  else {
    Serial.println(" - pressure");
    toothNum = (toothNum++ % 7) + 1;

    if (duration[brushCounter] > 0) {
      currentTooth = brush[brushCounter];
      checksum = currentTooth % 7;
      writeBT(currentTooth, checksum, 2);     // TODO: send 2(good_pressure) for now
      duration[brushCounter]--;
    } else {
      brushCounter++;
      return;
    }
//  }
  delay(interval);
}

void writeBT(int tooth, int checksum, int pressure) {
  char whole[20] = "";
  char str_tooth[20] = {'0'};
  char str_checksum[20] = {'0'};
  char str_pressure[20] = {'0'};
  char sep[20] = "/";
  char tokenizer[20] = "\r\n";
  itoa(tooth, &str_tooth[0], 10);
  strcat(whole, str_tooth);
  strcat(whole, sep);
  itoa(checksum, &str_tooth[0], 10);
  strcat(whole, str_tooth); 
  strcat(whole, sep);
  itoa(pressure, &str_pressure[0], 10);
  strcat(whole, str_pressure);
  strcat(whole, tokenizer);
  
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
