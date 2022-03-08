#include <networkControl.h>
#include <ArduinoMotorCarrier.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

#define DEBUG

const char ssid[] = "AndroidAP8BE4";        // your network SSID (name)
const char pass[] = "fgou8655";    // your network password (use for WPA, or use as key for WEP)

unsigned int localPort = 2390;      // local port to listen on
const char* IPaddr = "192.168.169.171"; //Ip address to listen to

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
    //CONTROL SYSTEM NEEDS TO BE IMPLEMENTED START
    int forwardV=(forward-50)/Cfoward;
    int turnV=(turn-50)/Cturn;
    //M3.setDuty(forwardV+turnV);
    //M4.setDuty(forwardV-turnV);
    M3.setDuty(100); //This is 100 full motor speed
    M4.setDuty(100);
   
    //CONTROL SYSTEM NEEDS TO BE IMPLEMENTENTED END
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
  char* recievedData = readInformation(4,4);
  connectedController = recievedData[4];
  
  motorChange(recievedData[0], recievedData[1]);

  //SEND gathered data out
  sendInformation(dataSend,2, localPort, IPaddr);

  //DELAY for timing
  delay(100);
}
