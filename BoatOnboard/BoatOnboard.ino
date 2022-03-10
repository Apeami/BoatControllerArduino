#include "networkControl.h"
#include <ArduinoMotorCarrier.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

#define DEBUG

const char ssid[] = "AndroidAP8BE4";        // your network SSID (name)
const char pass[] = "fgou8655";    // your network password (use for WPA, or use as key for WEP)

unsigned int localPort = 2390;      // local port to listen on
const char* IPaddr = "192.168.122.171"; //Ip address to listen to

int connectedController = 0;

void setUpMotor(){
  controller.begin();
  controller.reboot();
  delay(500);
  #ifdef DEBUG
  float batteryVoltage = (float)battery.getRaw() / 77;
  Serial.print("Battery voltage: ");
  Serial.println(batteryVoltage);
  #endif
  M1.setDuty(0); M2.setDuty(0); M3.setDuty(0); M4.setDuty(0);
}

#define Cfoward  1.00;
#define Cturn  5;
void motorChange(int forward, int turn){
  //forward 0 to 100, turn 0 to 100 , actually 93 but for simplicity it will be 100 in model
  //Foward and turn value comes in from the controller and is within the range.
  // need to use the setduty functions from -100 to 100 to control motor
  Serial.println("DATA");
  Serial.println(forward);
  Serial.println(turn);
  if (connectedController){
    //CONTROL SYSTEM NEEDS TO BE IMPLEMENTED 
    //START OF CONTROL SYSTEM
    //Input values forward 0 - 93, turn 0 - 93
    //Output values motor -100 - 100

    //Define general constants
    const int inputRange = 93;
    const int outputRange = 200;

    //Define specific constants
    const double turnRatio = 0.3;

    //Control calculation
    forward -= (inputRange/2);
    turn -= (inputRange/2);

    //Linear function for forward motion
    forward = (forward * outputRange) / inputRange;
    //Linear function for forward motion
    Serial.println(forward);
    Serial.println(turn);
    int forward3 = forward;
    int forward4 = forward;
    if (forward>0){
      forward3 -= (turn * turnRatio);
      forward4 += (turn * turnRatio);
    }else if(forward<=0){
      forward3 += (turn * turnRatio);
      forward4 -= (turn * turnRatio);
    }

    Serial.println(forward3);
    Serial.println(forward4);
    //Set the motors
    M3.setDuty(forward3);
    M4.setDuty(forward4);
    //M3.setDuty(100);
    //M4.setDuty(100);
   
    //END OF CONTROL SYSTEM
  }else{
    M3.setDuty(0);
    M4.setDuty(0);
  }
}


void setup() {
  Serial.begin(9600);
  //while (!Serial);
  setUpNetwork(localPort, ssid, pass);
  setUpMotor();
}

char dataSend[] = {1,0};

void loop() {
  
  //DO stuff with recieved data
  char* recievedData = readInformation(4,5);
  connectedController = recievedData[4];
  
  motorChange(recievedData[0], recievedData[1]);

  //SEND gathered data out
  sendInformation(dataSend,2, localPort, IPaddr);

  //DELAY for timing
  delay(100);
}
