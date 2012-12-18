//For the Arduino
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DofData.h"
#include "DofHandler.h"
#include <cctype>

#define xbee_TX 4
#define xbee_RX 5

//Setup Motor Information Format:  {pinNum, Default PWM}
int frontMotor[2] = {11, 255};
int rightMotor[2] = {10, 255};
int rearMotor[2] = {6, 255};
int leftMotor[2] = {5, 255};

//Define references to all the structs
DofData DOF_allData;
GyroData DOF_gyroData;
EulerData DOF_eulerData;

//Set different boundaries for safety procedures
int minSpeed = (255*0.35);
int maxSpeed = (255*0.0);
int hoverSpeed = (255*0.3);
const int minStep = 2;

//Define the structure that will be received by the xbee from the controller, and will tell it to go up, down, left, right, etc.
struct CONTROL {
  char command;
  int amount;
} command_struct;

//Setup Software Serial for xBee
SoftwareSerial xbee(xbee_RX,xbee_TX);
//Setup 9DOF handler
DofHandler<HardwareSerial> _9DOF(&Serial);

void setup()
{
  xbee.begin(9600); // Xbee uses Software Serial
//  Serial.begin(9600);
  _9DOF.begin(9600, 28800); //The 9DOF board uses Hardware Serial.  The first argument is old baud rate, the second argument is new one.
  
  //Set Motor pinMode
  pinMode(frontMotor[0], OUTPUT);
  pinMode(rightMotor[0], OUTPUT);
  pinMode(rearMotor[0], OUTPUT);
  pinMode(leftMotor[0], OUTPUT);
  
  //Set the Type of data to request from the 9DOF
  _9DOF.setDataMode(DOF_DATA_MODE_GYRO);
  //_9DOF.setUpdateInterval(20); //Stay above 20 for stabile results
}

//DEBUG VARIABLES
bool debug = false; //Enable to get average time spitout
unsigned long lastTime = 0;
unsigned long totalTime = 0;
int count = 0;
//END OF DEBUG VARIABLES


//Different Variables
boolean justStarted = true;
boolean inAir = false;
boolean sendData = false; //Send Data through the xbee


/**
  * Main loop
  * 
  * This is the main loop for the robot
  */
void loop() {
  if(justStarted) { //The Board just started
    initialize(); //Run some functions on a delay to allow the board to be fully working
  }
  doCommand(); //check for data being sent from the xbee and do things according to data received
  getData(); //Request and store data from the 9DOF
  stabilizingAlgorithm(); //control the motors based on data from the 9DOF to keep stable
}

/**
  * doCommand
  * 
  * This command checks for a command incoming to the xbee and then does the necessary things
  */
void doCommand() {
  if(xbee.available() >= sizeof(command_struct)) {
    //Read the Data from the xbee and save it to the contro
    saveSerialToStruct(xbee, &command_struct, sizeof(command_struct));
    switch(toupper(command_struct.command)) {
      case 'T': //Take-off
        if(!inAir) {
          takeOff();
        }
        break;
      case 'Z': //Land
        land();
        break;
      case 'H': //Hover
        hover();
        break;
      case 'R': //Go Right
        goRight(command_struct.amount);
        break;
      case 'L': //Go Left
        goLeft(command_struct.amount);
        break;
      case 'F': //Go Forward
        goForward(command_struct.amount);
        break;
      case 'B': //Go Backwards
        goBackwards(command_struct.amount);
        break;
      case 'I': //Increase Altitude
        increaseAltitude(command_struct.amount);
        break;
      case 'D': //Decrease Altitue
        decreaseAltitude(command_struct.amount);
        break;
      case 'G': //Set request data to GYRO and send data to Master
        _9DOF.setDataMode(DOF_DATA_MODE_GYRO);
        sendData = true;
        break;
      case 'A': //Set request data to ALL and send data to Master
        _9DOF.setDataMode(DOF_DATA_MODE_ALL);
        sendData = true;
        break;
      case 'E': //Set request data to EULER and send data to Master
        _9DOF.setDataMode(DOF_DATA_MODE_EULER);
        sendData = true;
        break;
      case 'S': //Spin Right
        spinRight(command_struct.amount);
        break;
      case 'Q': //Spin Left
        spinLeft(command_struct.amount);
        break;
    }
  }
}


/**
  * getData
  * 
  * This gets the data from the 9DOF and calls the proper function to store
  */
void getData() {
  if(_9DOF.checkStreamValid()) { //checks to see if the data stream from the 9DOF is correct
    switch(_9DOF.getLastDataMode()) {
      case DOF_DATA_MODE_ALL: //All data was sent
        handleData_ALL();
        break;
      case DOF_DATA_MODE_GYRO: //Only gyro data was sent
        handleData_GYRO();
        break;
      case DOF_DATA_MODE_EULER: //Only Euler angles were sent
        handleData_EULER();
        break;
    }
  }
}


/**
  * handleData_ALL
  * 
  * Gets the data stream from the 9DOF and stores it into the proper structure.
  *
  * @notes this function is used for ALL data only
  */
void handleData_ALL() {
  DOF_allData = _9DOF.getData();
}

/**
  * handleData_GYRO
  * 
  * Gets the data stream from the 9DOF and stores it into the proper structure.
  *
  * @notes this function is used for Gyro data only
  */
void handleData_GYRO() {
  DOF_gyroData = _9DOF.getGyroData();
}

/**
  * handleData_EULER
  * 
  * Gets the data stream from the 9DOF and stores it into the proper structure.
  *
  * @notes this function is used for EULER data only
  */
void handleData_EULER() {
  DOF_eulerData = _9DOF.getEulerData();
}


/**
  * stabilizingAlgorithm
  * 
  * Handles all the math to make sure the flight is stable
  */
void stabilizingAlgorithm() {
}


/**
  * initialize
  * 
  * Stats the device up
  *
  * @notes 2500ms delay is to make sure everything is functioning before any commands can be given
  */
void initialize() {
  Serial.println("Device is Starting Up");
  justStarted = false;
  delay(2500); //Offer a small delay to make sure everything is on properly startedup
  _9DOF.requestData(); //Create the inital request for data
  lastTime = micros();
}


/**
  * preTakeOff
  * 
  * slowly increased the speed of the propellers so take off can be smooth
  */
void preTakeOff() {
  increaseAltitude(minSpeed*2);
  delay(250);
  unsigned int numSteps = 25; //Number of steps to take til the robot is fully taken off
  for(unsigned int i = 0; i < numSteps; i++) {
    increaseAltitude((minSpeed*2)/numSteps);
    delay(30);
  }
  delay(250);
}


/**
  * takeOff
  * 
  * increased the propeller speed to the required minimum speed for flight
  */
void takeOff() {
  preTakeOff();
  inAir = true;
  increaseAltitude(minSpeed);
}

/**
  * land
  * 
  * Slowly decreased propeller speed to cause the landing to be soft
  */
void land() {
  int speedSteps = (hoverSpeed/25);
  decreaseAltitude(minSpeed);
  delay(250);
  unsigned int numSteps = 25; //Number of steps to take til the motors are fully turned off
  for(unsigned int i = 0; i < numSteps; i++) {
    decreaseAltitude((minSpeed*2)/numSteps);
    delay(30);
  }
  delay(250);
  inAir = false;
}


/**
  * goRight
  * 
  * change the motor speeds to cause the Robot to go Right
  *
  * @param How much to move
  */
void goRight(const int changeStep) {
}


/**
  * goLeft
  * 
  * change the motor speeds to cause the Robot to go Left
  *
  * @param How much to move
  */
void goLeft(const int changeStep) {
}


/**
  * goForward
  * 
  * change the motor speeds to cause the Robot to go forward
  *
  * @param How much to move
  */
void goForward(const int changeStep) {
}


/**
  * goBackwards
  * 
  * change the motor speeds to cause the Robot to go backward
  *
  * @param How much to move
  */
void goBackwards(const int changeStep) {
}


/**
  * spinRight
  * 
  * change the motor speeds to cause the Robot to rotate right
  *
  * @param How much to move
  */
void spinRight(const int changeStep) {
}

/**
  * spinLeft
  * 
  * change the motor speeds to cause the Robot to rotate left
  *
  * @param How much to move
  */
void spinLeft(const int changeStep) {
}


/**
  * changeHoverSpeed
  * 
  * change the hovor speed
  *
  * @param speed to change motor by
  * @param true increased the motor speed
  */
void changeHoverSpeed(const int *changeStep, boolean increase) {
  if(increase) { //To Increase motor speed you need to subtract to have a lower PWM
    hoverSpeed -= *changeStep;
  } else { //To Decrease motor speed you need to add to have a higher PWM
    hoverSpeed += *changeStep;
  }
}


/**
  * increaseAltitude
  * 
  * increase all motor speeds by specified amount
  *
  * @param speed to increase motor by
  */
void increaseAltitude(const int changeStep) {
  changeHoverSpeed(&changeStep, true); //Increase the hover speed.   THIS IS ONLY TEMPORARY TO FIND A GOOD HOVER SPEED
  increaseMotor(frontMotor, &changeStep);
  increaseMotor(rightMotor, &changeStep);
  increaseMotor(rearMotor, &changeStep);
  increaseMotor(leftMotor, &changeStep);
}

/**
  * decreaseAltitude
  * 
  * decreased all motor speeds by specified amount
  *
  * @param speed to decrease motor by
  */
void decreaseAltitude(const int changeStep) {
  changeHoverSpeed(&changeStep, false); //Decrease the hover speed.   THIS IS ONLY TEMPORARY TO FIND A GOOD HOVER SPEED
  decreaseMotor(frontMotor, &changeStep);
  decreaseMotor(rightMotor, &changeStep);
  decreaseMotor(rearMotor, &changeStep);
  decreaseMotor(leftMotor, &changeStep);
}

/**
  * hovor
  * 
  * sets the motor to a constant hovering speed
  */
void hover() {
  setMotorSpeed(frontMotor, hoverSpeed);
  setMotorSpeed(rightMotor, hoverSpeed);
  setMotorSpeed(rearMotor, hoverSpeed);
  setMotorSpeed(leftMotor, hoverSpeed);
}


/**
  * increaseMotor
  * 
  * decreased motor speed by specified amount
  *
  * @param Motor to change
  * @param speed to change motor by
  */
void increaseMotor(int *pinInfo, const int *changeStep) {
  pinInfo[1] -= *changeStep;
  setMotorSpeed(pinInfo);
}


/**
  * decreaseMotor
  * 
  * decreased motor speed by specified amount
  *
  * @param Motor to change
  * @param speed to change motor by
  */
void decreaseMotor(int *pinInfo, const int *changeStep) {
  pinInfo[1] += *changeStep;
  setMotorSpeed(pinInfo);
}

/**
  * setMotorSpeed
  * 
  * Sets the speed of the motor
  *
  * @param Motor to change
  */
void setMotorSpeed(int *motorInfo) {
  analogWrite(motorInfo[0], motorInfo[1]);
}


/**
  * setMotorSpeed
  * 
  * Sets the speed of the motor
  *
  * @param Motor to change
  * @param speed to set motor to
  */
void setMotorSpeed(int *motorInfo, const int newSpeed) {
  motorInfo[1] = newSpeed;
  setMotorSpeed(motorInfo);
}



/**
  * saveSerialToStruct
  * 
  * Gets the data from the stream and stores to a struct
  *
  * @param Stream to read data from
  * @param ptr to struct to fill
  * @param size of struct
  */
void saveSerialToStruct(Stream &ostream, void* ptr, unsigned int objSize) {
  char data[objSize]; //Create a tmp char array of the data from Streal
  ostream.readBytes(data, objSize); //Read the # of bytes
  memcpy(ptr, data, objSize); //Copy the bytes into the struct
}
    

/**
  * sertialStructPrint
  * 
  * Gets the data from the struct and sends to the stream
  *
  * @param Stream to send data to
  * @param ptr to struct to fill
  * @param size of struct
  */
void serialStructPrint(Stream &ostream, void* ptr, unsigned int objSize) {
  byte * b = (byte *) ptr; //Create a ptr array of the bytes to send
  for(unsigned int i = 0; i<objSize; i++) {
    ostream.write(b[i]); //Write each byte to the stream
  }
}



/*
void getDataOld() {
    if(Serial.available() >= sizeof(AMG_ANGLES)) {
    //Serial.println("Getting Struct Data");
      Serial.readBytes(inData, sizeof(AMG_ANGLES));
      
      memcpy(&gData, inData, sizeof(f));
      if(debug && gData.checkSum != (gData.x + gData.y + gData.z)%10) {
         Serial.println("Error #1"); 
      }
      if(debug && (count%100) == 0) {
        Serial.print((totalTime/count));
        Serial.print("-");
        Serial.println(count);
      }

      if(debug) {
        totalTime += ((micros())-lastTime);
        lastTime = micros();
        count++;
      }
      doStuffBasedOnData();
      delay(2); //For Stability
      Serial.print('A');
  }
}*/
