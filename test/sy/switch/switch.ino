
int buttonApin = 9;

void setup() 
{
  Serial.begin(9600);
  pinMode(buttonApin, INPUT_PULLUP); 
}
 
void loop() 
{
  if (digitalRead(buttonApin) == LOW)
  {
    Serial.println("switch down");
  //  analogWrite(6, 100);
  } else {
    Serial.println("switch on");
  //  analogWrite(6, -1);
  }
}
