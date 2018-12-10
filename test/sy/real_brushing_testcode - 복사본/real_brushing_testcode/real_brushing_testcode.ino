#include <Wire.h>
#include <SoftwareSerial.h>
#include <math.h> // (no semicolon)

SoftwareSerial BTSerial(2, 3);
void get6050();
void writeBT(int tooth, int checksum, int pressure);

const int MPU = 0x68;               //MPU6050 I2C주소
int AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int index = 999;
int pry[28][3]= {0, };
int LUT[28] = {0, };
// number of tooth
int toothNum = 0;
#define MPU6050_ACCEL_XOUT_H 0x3B // R
#define MPU6050_PWR_MGMT_1 0x6B // R/W
#define MPU6050_PWR_MGMT_2 0x6C // R/W
#define MPU6050_WHO_AM_I 0x75 // R
#define MPU6050_I2C_ADDRESS 0x68
// pressure
int pressurePin = 0;     // the pressure and 10K pulldown are connected to a0
int pressureReading;     // the analog reading from the pressure resistor divider

// switch and vibration
int ledPin = 5;
int buttonApin = 9;
int count = 0;
// brush order 
int test[18] = {37, 36, 35, 34, 33, 32, 31, 41, 42, 43, 44, 45, 46, 47, 36, 35, 44, 45};
int brush[100]    = { 11, 21, -1, 41, 11, 12, 13, -1, 15, 16, 17, 16, 15, 14, 13, 12, 11, 21, 22, 23, 24, 25, 26, 27, 26, 25, 24, 23, 22, 21, 31, 32, 33, 34, 35, 36, 37, 36, 35, 34, 33, 32, 31, 41, 42, 43, 44, 45, 46, 47, 46, 45, 44, 43, 42, 41 };
int duration[100] = {  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7 }; 
//int duration[100] = {  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1 }; 
//int duration[100] = {  1,  1,  1,  1, 0 };
int duration_copy[100] = { 0 };
int brushCounter = 0;
int total = 0;
struct GyroKalman{
  float x_angle, x_bias;
  float P_00, P_01, P_10, P_11;
  float Q_angle, Q_gyro;
  float R_angle;
};

struct GyroKalman angX;
struct GyroKalman angY;
struct GyroKalman angZ;

static const float R_angle = 0.3;     //.3 default
static const float Q_angle = 0.01;  //0.01 (Kalman)
static const float Q_gyro = 0.04; //0.04 (Kalman)
const int lowX = -2150;
const int highX = 2210;
const int lowY = -2150;
const int highY = 2210;
const int lowZ = -2150;
const int highZ = 2550;
/* time */
unsigned long prevSensoredTime = 0;
unsigned long curSensoredTime = 0;

typedef union accel_t_gyro_union
{
  struct
  {
  uint8_t x_accel_h;
  uint8_t x_accel_l;
  uint8_t y_accel_h;
  uint8_t y_accel_l;
  uint8_t z_accel_h;
  uint8_t z_accel_l;
  uint8_t t_h;
  uint8_t t_l;
  uint8_t x_gyro_h;
  uint8_t x_gyro_l;
  uint8_t y_gyro_h;
  uint8_t y_gyro_l;
  uint8_t z_gyro_h;
  uint8_t z_gyro_l;
  } reg;

  struct
  {
    int x_accel;
    int y_accel;
    int z_accel;
    int temperature;
    int x_gyro;
    int y_gyro;
    int z_gyro;
  } value;
};
int xInit[5] = {0,0,0,0,0};
int yInit[5] = {0,0,0,0,0};
int zInit[5] = {0,0,0,0,0};
int initIndex = 0;
int initSize = 5;
int xCal = 0;
int yCal = 0;
int zCal = 1800;

int gx1_raw = 0;
int gy1_raw = 0;
int gz1_raw = 0;
int currentTooth = 999;
int checksum = 999;
// Bluetooth write interval
int interval = 100;

void setup() {
    pinMode(ledPin, OUTPUT);
    pinMode(buttonApin, INPUT_PULLUP); 

    Serial.begin(9600);
    BTSerial.begin(9600);//블루투스와의 통신속도 설정
  initGyroKalman(&angX, Q_angle, Q_gyro, R_angle);
  initGyroKalman(&angY, Q_angle, Q_gyro, R_angle);
  initGyroKalman(&angZ, Q_angle, Q_gyro, R_angle);
  Wire.begin();
   Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);//MPU6050 을 동작 대기 모드로 변경
  Wire.endTransmission(true);

    LUT[0] = 37;
  LUT[1] = 36;
  LUT[2] = 35;
  LUT[3] = 34;
  LUT[4] = 33;
  LUT[5] = 32;
  LUT[6] = 31;
  LUT[7] = 41;
  LUT[8] = 42;
  LUT[9] = 43;
  LUT[10] = 44;
  LUT[11] = 45;
  LUT[12] = 46;
  LUT[13] = 47;
  LUT[14] = 27;
  LUT[15] = 26;
  LUT[16] = 25;
  LUT[17] = 24;
  LUT[18] = 23;
  LUT[19] = 22;
  LUT[20] = 21;
  LUT[21] = 11;
  LUT[22] = 12;
  LUT[23] = 13;
  LUT[24] = 14;
  LUT[25] = 15;
  LUT[26] = 16;
  LUT[27] = 17;
    count = 0;

   pry[0][2] = -1;
   pry[1][2] = -3;
   pry[2][2] = -6;
   pry[3][2] = -9;
   pry[4][2] = -12;
   pry[5][2] = -15;
   pry[6][2] = -18;

   pry[7][2] = 18;
   pry[8][2] = 15;
   pry[9][2] = 12;
   pry[10][2] = 9;
   pry[11][2] = 6;
   pry[12][2] = 3;
   pry[13][2] = 1;

   pry[14][2] = -1;
   pry[15][2] = -3;
   pry[16][2] = -6;
   pry[17][2] = -9;
   pry[18][2] = -12;
   pry[19][2] = -15;
   pry[20][2] = -18;

   pry[21][2] = 18;
   pry[22][2] = 15;
   pry[23][2] = 12;
   pry[24][2] = 9;
   pry[25][2] = 6;
   pry[26][2] = 3;
   pry[27][2] = 1;
}

void loop() {
currentTooth = -1;
int  pressure = -1;
checksum = -1;
  

  // pressure
  pressureReading = analogRead(pressurePin);  
  if(pressureReading >= 40){
    pressure = 1;
  }
  if(count <18){
 if(pressure == 1){
  currentTooth = test[count];
    checksum = currentTooth % 7;
    writeBT(currentTooth, checksum, 2);

    delay(interval); 
    pressureReading = analogRead(pressurePin); 
    if(pressureReading <40){
      pressure = -1;
      count ++;
      Serial.println(count);
      
    delay(500); 
      }
  }
  }
  else if(count ==18){
      int error;
  double dT;
  accel_t_gyro_union accel_t_gyro;
  
  curSensoredTime = millis();

  error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
  if(error != 0) {
    Serial.print(F("Read accel, temp and gyro, error = "));
    Serial.println(error,DEC);
  }

  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap
  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);
  

  if(prevSensoredTime > 0) {
    int gx1=0, gy1=0, gz1 = 0;
    float gx2=0, gy2=0, gz2 = 0;

    int loopTime = curSensoredTime - prevSensoredTime;

    gx2 = angleInDegrees(lowX, highX, accel_t_gyro.value.x_gyro);
    gy2 = angleInDegrees(lowY, highY, accel_t_gyro.value.y_gyro);
    gz2 = angleInDegrees(lowZ, highZ, accel_t_gyro.value.z_gyro);

    predict(&angX, gx2, loopTime);
    predict(&angY, gy2, loopTime);
    predict(&angZ, gz2, loopTime);

    gx1 = update(&angX, accel_t_gyro.value.x_accel) / 10;
    gy1 = update(&angY, accel_t_gyro.value.y_accel) / 10;
    gz1 = update(&angZ, accel_t_gyro.value.z_accel) / 10;

    /////////////////////////////////////////////////////////////////////////////
    //  ���� ����� �� �����Ǵ� n���� ���� ��� => ���� �����Ǵ� ���� ����
    /////////////////////////////////////////////////////////////////////////////
    if(initIndex < initSize) {
      xInit[initIndex] = gx1;
      yInit[initIndex] = gy1;
      zInit[initIndex] = gz1;
      if(initIndex == initSize - 1) {
        int sumX = 0; int sumY = 0; int sumZ = 0;
        for(int k=1; k <= initSize; k++) {
          sumX += xInit[k];
          sumY += yInit[k];
          sumZ += zInit[k];
        }

        xCal -= sumX/(initSize -1);
        yCal -= sumY/(initSize -1);
        zCal = (sumZ/(initSize -1) - zCal);
      }
      initIndex++;
    }
    
    /////////////////////////////////////////////////////////////////////////////
    //  �������� ���� �ʿ��� �۾��� ó���ϴ� ��ƾ
    /////////////////////////////////////////////////////////////////////////////
    else {
      // ������ ����
        gx1 += xCal;
        gy1 += yCal;
        //gz1 += zCal;
  
      // �������� ���� �ʿ��� ó���� ����
      // if(gz1 < 1400 && -250 < gy1 && gy1 < 250 && gx1 < 500) {
      //  Serial.print(F("Turn right"));
      //  Serial.println(F(""));
      //}

    }

    gx1_raw = map(gx1, -16383, 16383, -200, 200);
    gy1_raw = map(gy1, -16383, 16383, -200, 200);
    gz1_raw = map(gz1, -16383, 16383, -200, 200);
 /*   if(gx1_raw>= 10 && gy1_raw<=10){
      Serial.print(F("47\n"));
      }
      else if(gx1_raw>= -10 && gy1_raw<=10){
      Serial.print(F("46\n"));
      }
      else if(gy1_raw<=10){
      Serial.print(F("45\n"));
      }
      else{
        Serial.print(F("X\n"));
      }*/
      pressureReading = analogRead(pressurePin);
if(pressureReading>= 10){
          for(int i = 0; i < 14; i++){
          if(((pry[i][2] - gz1_raw)>=-1 &&(pry[i][2] - gz1_raw)<=1 ) && accel_t_gyro.value.x_accel>=0 ){
            index = i;
            break;
          }  
          for(int i = 14; i < 28; i++){
          if(((pry[i][2] - gz1_raw)>=-1 &&(pry[i][2] - gz1_raw)<=1 ) && accel_t_gyro.value.x_accel<0 ){
            index = i;
            break;
          }  
            }

        }
        

  }
 pressureReading = analogRead(pressurePin);
 if(pressureReading<9){
  index =999;
 }
 if(index!=999){
   currentTooth = LUT[index];
   checksum = currentTooth %7;
   writeBT(currentTooth, checksum, 2);
  }
      Serial.print(index);
      Serial.println(F(""));
      Serial.print(F("Angle x,y,z : "));
      Serial.print(gx1_raw, DEC);
      Serial.print(F(", "));
      Serial.print(gy1_raw, DEC);
      Serial.print(F(", "));
      Serial.print(gz1_raw, DEC);
      Serial.println(F(""));

  
  delay(200);
//  analogWrite( vibrator_pin , strength );
} // End of loop()
prevSensoredTime = curSensoredTime;
    }
else{
    Serial.println(" end");
}

//  }
  
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

/**************************************************
 * Sensor read/write
 **************************************************/
int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;
  
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  
  n = Wire.write(start);
  if (n != 1)
    return (-10);
  
  n = Wire.endTransmission(false); // hold the I2C-bus
  if (n != 0)
    return (n);
  
  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);
  return (0); // return : no error
}

int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;
  
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  
  n = Wire.write(start); // write the start address
  if (n != 1)
    return (-20);
    
  n = Wire.write(pData, size); // write data bytes
  if (n != size)
    return (-21);
    
  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);
  return (0); // return : no error
}

int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;
  error = MPU6050_write(reg, &data, 1);
  return (error);
}

/**************************************************
 * Raw data processing
 **************************************************/
float angleInDegrees(int lo, int hi, int measured) {
  float x = (hi - lo)/180.0;
  return (float)measured/x;
}

void initGyroKalman(struct GyroKalman *kalman, const float Q_angle, const float Q_gyro, const float R_angle) {
  kalman->Q_angle = Q_angle;
  kalman->Q_gyro = Q_gyro;
  kalman->R_angle = R_angle;
  
  kalman->P_00 = 0;
  kalman->P_01 = 0;
  kalman->P_10 = 0;
  kalman->P_11 = 0;
}

/*
* The kalman predict method.
* kalman    the kalman data structure
* dotAngle    Derivitive Of The (D O T) Angle. This is the change in the angle from the gyro.
*           This is the value from the Wii MotionPlus, scaled to fast/slow.
* dt        the change in time, in seconds; in other words the amount of time it took to sweep dotAngle
*/
void predict(struct GyroKalman *kalman, float dotAngle, float dt) {
  kalman->x_angle += dt * (dotAngle - kalman->x_bias);
  kalman->P_00 += -1 * dt * (kalman->P_10 + kalman->P_01) + dt*dt * kalman->P_11 + kalman->Q_angle;
  kalman->P_01 += -1 * dt * kalman->P_11;
  kalman->P_10 += -1 * dt * kalman->P_11;
  kalman->P_11 += kalman->Q_gyro;
}

/*
* The kalman update method
* kalman  the kalman data structure
* angle_m   the angle observed from the Wii Nunchuk accelerometer, in radians
*/
float update(struct GyroKalman *kalman, float angle_m) {
  const float y = angle_m - kalman->x_angle;
  const float S = kalman->P_00 + kalman->R_angle;
  const float K_0 = kalman->P_00 / S;
  const float K_1 = kalman->P_10 / S;
  kalman->x_angle += K_0 * y;
  kalman->x_bias += K_1 * y;
  kalman->P_00 -= K_0 * kalman->P_00;
  kalman->P_01 -= K_0 * kalman->P_01;
  kalman->P_10 -= K_1 * kalman->P_00;
  kalman->P_11 -= K_1 * kalman->P_01;
  return kalman->x_angle;
}
