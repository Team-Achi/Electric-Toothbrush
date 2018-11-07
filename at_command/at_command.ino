#include <SoftwareSerial.h>
 
SoftwareSerial BTSerial(2,3);
void setup()
{
  Serial.begin(9600); //컴퓨터와의 통신속도 설정
  Serial.println("start");
  BTSerial.begin(9600);//블루투스와의 통신속도 설정
}
 
void loop()
{
  
  //delay(1); 
  while (BTSerial.available()) {      
    Serial.write(BTSerial.read());  //블루투스 데이터 -> 시리얼 모니터 
  } 
  //delay(1); 
  while (Serial.available()) {          
    BTSerial.write(Serial.read());  //시리얼 모니터 -> 블루투스  
  } 
}
