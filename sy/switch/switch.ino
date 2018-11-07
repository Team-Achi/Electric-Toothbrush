int ledPin = 5;
int buttonApin = 9;

byte leds = 0;
 
void setup() 
{
  pinMode(ledPin, OUTPUT);
  pinMode(buttonApin, INPUT_PULLUP); 
}
 
void loop() 
{
  if (digitalRead(buttonApin) == LOW)
  {
    Serial.println("switch down");
    analogWrite(6, 100);
  } else {
    analogWrite(6, -1);
  }
}
