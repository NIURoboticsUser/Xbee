#define pinBtn 8

unsigned long lastTime = 0;
unsigned long currTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(pinBtn, INPUT);
}


void loop() {
  delay(5000);
   Serial.write('A');
   while(true) {
  if(Serial.read() == 'B') {
    currTime = micros();
    Serial.print((currTime-lastTime));
    Serial.write('A');
    Serial.println();
    lastTime=currTime;
  }  
   }
}
