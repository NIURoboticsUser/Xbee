#define pinBtn 8

unsigned long lastTime = 0;
unsigned long currTime = 0;
unsigned long totalTime = 0;
int count = 0;

void setup() {
  Serial.begin(9600);
  pinMode(pinBtn, INPUT);
}


void loop() {
//  delay(5000);
   Serial.write('A');
   while(true) {
  if(Serial.read() == 'B') {
    currTime = micros();
    unsigned long timeElapsed = currTime-lastTime;
    Serial.print((int)(totalTime/count));
    Serial.write('A');
    Serial.println();
    lastTime=currTime;
    totalTime += timeElapsed;
    count++;
  }  
   }
}
