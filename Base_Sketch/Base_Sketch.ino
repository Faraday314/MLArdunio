#define AMP_PIN A3
#define RELAY_PIN 8

void setup() {
  pinMode(AMP_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);
  Serial.begin(9600);
}

void loop() {
  
  float vOut = (analogRead(AMP_PIN) / 1023.0) * 5000;
  float current = (vOut);
  float shift = 2500-current;
  float amps = abs(shift);
  
  Serial.println(amps);
}
