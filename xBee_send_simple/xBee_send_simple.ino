#define pinBtn 8

void setup() {
  Serial.begin(9600);
  pinMode(pinBtn, INPUT);
}


void loop() {
  Serial.write('B');
  delay(1000);
}
