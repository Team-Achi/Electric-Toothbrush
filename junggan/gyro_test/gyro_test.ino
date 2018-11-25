#include <math.h> // (no semicolon)
#include <Wire.h>

/* MPU-6050 sensor */
#define MPU6050_ACCEL_XOUT_H 0x3B // R
#define MPU6050_PWR_MGMT_1 0x6B // R/W
#define MPU6050_PWR_MGMT_2 0x6C // R/W
#define MPU6050_WHO_AM_I 0x75 // R
#define MPU6050_I2C_ADDRESS 0x68

int vibrator_pin = 6;
int strength = 255;
int ledPin = 5;
int buttonApin = 9;
int index = 999;
int pry[28][3]= {0, };
int fsrPin = 0;
int fsrReading;
/* Kalman filter */
struct GyroKalman{
	float x_angle, x_bias;
	float P_00, P_01, P_10, P_11;
	float Q_angle, Q_gyro;
	float R_angle;
};

struct GyroKalman angX;
struct GyroKalman angY;
struct GyroKalman angZ;

static const float R_angle = 0.3; 		//.3 default
static const float Q_angle = 0.01;	//0.01 (Kalman)
static const float Q_gyro = 0.04;	//0.04 (Kalman)
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
int count = 0;
void setup()
{
	int error;
	uint8_t c;
  pinMode( vibrator_pin , OUTPUT);
///  pinMode(ledPin, OUTPUT);
//  pinMode(buttonApin, INPUT_PULLUP); 
  analogWrite( vibrator_pin , strength );
  delay(100);
	initGyroKalman(&angX, Q_angle, Q_gyro, R_angle);
	initGyroKalman(&angY, Q_angle, Q_gyro, R_angle);
	initGyroKalman(&angZ, Q_angle, Q_gyro, R_angle);

	Serial.begin(9600);
	Wire.begin();

	error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
	Serial.print(F("WHO_AM_I : "));
	Serial.print(c,HEX);
	Serial.print(F(", error = "));
	Serial.println(error,DEC);

	error = MPU6050_read (MPU6050_PWR_MGMT_2, &c, 1);
	Serial.print(F("PWR_MGMT_2 : "));
	Serial.print(c,HEX);
	Serial.print(F(", error = "));
	Serial.println(error,DEC);
  pinMode(buttonApin, INPUT_PULLUP);
}


void loop()
{
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
			//	Serial.print(F("Turn right"));
			//	Serial.println(F(""));
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
      fsrReading = analogRead(fsrPin);
if(fsrReading!= 0){
      if(count<28){
        if(digitalRead(buttonApin) == LOW){
          if(count ==0){
            pry[0][0] = gx1_raw;
            pry[0][1] = gy1_raw;
            pry[0][2] = gz1_raw;
            delay(500);
          }
          else if(count == 1){
            pry[1][0] = gx1_raw;
            pry[1][1] = gy1_raw;
            pry[1][2] = gz1_raw;
            delay(500);
          }
          else if(count == 2){
            pry[2][0] = gx1_raw;
            pry[2][1] = gy1_raw;
            pry[2][2] = gz1_raw;
            delay(500);
          }
          else if(count == 3){
            pry[3][0] = gx1_raw;
            pry[3][1] = gy1_raw;
            pry[3][2] = gz1_raw;
            delay(500);
          }
          else if(count == 4){
            pry[4][0] = gx1_raw;
            pry[4][1] = gy1_raw;
            pry[4][2] = gz1_raw;
            delay(500);
          }
          else if(count == 5){
            pry[5][0] = gx1_raw;
            pry[5][1] = gy1_raw;
            pry[5][2] = gz1_raw;
            delay(500);
          }
          else if(count == 6){
            pry[6][0] = gx1_raw;
            pry[6][1] = gy1_raw;
            pry[6][2] = gz1_raw;
            delay(500);
          }
          else if(count == 7){
            pry[7][0] = gx1_raw;
            pry[7][1] = gy1_raw;
            pry[7][2] = gz1_raw;
            delay(500);
          }
          else if(count == 8){
            pry[8][0] = gx1_raw;
            pry[8][1] = gy1_raw;
            pry[8][2] = gz1_raw;
            delay(500);
          }
          else if(count == 9){
            pry[9][0] = gx1_raw;
            pry[9][1] = gy1_raw;
            pry[9][2] = gz1_raw;
            delay(500);
          }
          else if(count == 10){
            pry[10][0] = gx1_raw;
            pry[10][1] = gy1_raw;
            pry[10][2] = gz1_raw;
            delay(500);
          }
          else if(count == 11){
            pry[11][0] = gx1_raw;
            pry[11][1] = gy1_raw;
            pry[11][2] = gz1_raw;
            delay(500);
          }
          else if(count == 12){
            pry[12][0] = gx1_raw;
            pry[12][1] = gy1_raw;
            pry[12][2] = gz1_raw;
            delay(500);
          }
          else if(count == 13){
            pry[13][0] = gx1_raw;
            pry[13][1] = gy1_raw;
            pry[13][2] = gz1_raw;
            delay(500);
          }
          else if(count == 14){
            pry[14][0] = gx1_raw;
            pry[14][1] = gy1_raw;
            pry[14][2] = gz1_raw;
            delay(500);
          }
          else if(count == 15){
            pry[15][0] = gx1_raw;
            pry[15][1] = gy1_raw;
            pry[15][2] = gz1_raw;
            delay(500);
          }
          else if(count == 16){
            pry[16][0] = gx1_raw;
            pry[16][1] = gy1_raw;
            pry[16][2] = gz1_raw;
            delay(500);
          }
          else if(count == 17){
            pry[17][0] = gx1_raw;
            pry[17][1] = gy1_raw;
            pry[17][2] = gz1_raw;
            delay(500);
          }
          else if(count == 18){
            pry[18][0] = gx1_raw;
            pry[18][1] = gy1_raw;
            pry[18][2] = gz1_raw;
            delay(500);
          }
          else if(count == 19){
            pry[19][0] = gx1_raw;
            pry[19][1] = gy1_raw;
            pry[19][2] = gz1_raw;
            delay(500);
          }
          else if(count == 20){
            pry[20][0] = gx1_raw;
            pry[20][1] = gy1_raw;
            pry[20][2] = gz1_raw;
            delay(500);
          }
          else if(count == 21){
            pry[21][0] = gx1_raw;
            pry[21][1] = gy1_raw;
            pry[21][2] = gz1_raw;
            delay(500);
          }
          else if(count == 22){
            pry[22][0] = gx1_raw;
            pry[22][1] = gy1_raw;
            pry[22][2] = gz1_raw;
            delay(500);
          }
          else if(count == 23){
            pry[23][0] = gx1_raw;
            pry[23][1] = gy1_raw;
            pry[23][2] = gz1_raw;
            delay(500);
          }
          else if(count == 24){
            pry[24][0] = gx1_raw;
            pry[24][1] = gy1_raw;
            pry[24][2] = gz1_raw;
            delay(500);
          }
          else if(count == 25){
            pry[25][0] = gx1_raw;
            pry[25][1] = gy1_raw;
            pry[25][2] = gz1_raw;
            delay(500);
          }
          else if(count == 26){
            pry[26][0] = gx1_raw;
            pry[26][1] = gy1_raw;
            pry[26][2] = gz1_raw;
            delay(500);
          }
          else if(count == 27){
            pry[27][0] = gx1_raw;
            pry[27][1] = gy1_raw;
            pry[27][2] = gz1_raw;
            delay(500);
          }
         Serial.println("OK");
               Serial.print(F("Angle x,y,z : "));
      Serial.print(gx1_raw, DEC);
      Serial.print(F(", "));
      Serial.print(gy1_raw, DEC);
      Serial.print(F(", "));
      Serial.print(gz1_raw, DEC);
      Serial.print(F(", "));
      Serial.print(count);
      Serial.println(F(""));
          count ++;
          }
        }
        else{
          for(int i = 0; i < 14; i++){
          if(((pry[i][2] - gz1_raw)>=-1 &&(pry[i][2] - gz1_raw)<=1 ) && accel_t_gyro.value.x_accel>=0 ){
            index = i;
          //  Serial.print(i);
          //  Serial.println(" ");
            break;
          }  
          for(int i = 14; i < 28; i++){
          if(((pry[i][2] - gz1_raw)>=-1 &&(pry[i][2] - gz1_raw)<=1 ) && accel_t_gyro.value.x_accel<0 ){
            index = i;
          //  Serial.print(i);
          //  Serial.println(" ");
            break;
          }  
            }
      //&& ((pry[i][1] - gy1_raw)>=-1 &&(pry[i][1] - gy1_raw)<=1 ) && ((pry[i][2] - gz1_raw)>=-1 &&(pry[i][2] - gz1_raw)<=1 )

        }
        }

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
//	analogWrite( vibrator_pin , strength );
}	// End of loop()
prevSensoredTime = curSensoredTime;}

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
* kalman 		the kalman data structure
* dotAngle 		Derivitive Of The (D O T) Angle. This is the change in the angle from the gyro.
* 					This is the value from the Wii MotionPlus, scaled to fast/slow.
* dt 				the change in time, in seconds; in other words the amount of time it took to sweep dotAngle
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
* kalman 	the kalman data structure
* angle_m 	the angle observed from the Wii Nunchuk accelerometer, in radians
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
