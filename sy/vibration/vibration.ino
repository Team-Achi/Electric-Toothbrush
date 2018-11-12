int vibrator_pin = 6;
int strength = 20;
int ledPin = 5;
int buttonApin = 9;


void setup() {
  // put your setup code here, to run once:
  pinMode( vibrator_pin , OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buttonApin, INPUT_PULLUP); 
}

void loop() {
  // put your main code here, to run repeatedly:
analogWrite( vibrator_pin , strength );
//delay(1000);



}
