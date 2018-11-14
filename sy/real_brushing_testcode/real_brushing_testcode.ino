#include <Wire.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(2, 3);
const int MPU = 0x68;               //MPU6050 I2C주소
int AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
void get6050();
void writeBT();

int toothNum = 0;

// pressure
int pressurePin = 0;     // the pressure and 10K pulldown are connected to a0
int pressureReading;     // the analog reading from the pressure resistor divider

// switch and vibration
int ledPin = 5;
int buttonApin = 9;


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
    // gyro
    toothNum = (toothNum++ % 7) + 1;
    // read raw accel/gyro measurements from device
    get6050();
    if (AcZ > 0x0) {
      writeBT(toothNum + 10);
    }
    else if(AcZ <= 0x0) {
      writeBT(toothNum + 40);
    }
  }
  delay(1000);
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
