#include <ArduinoMotorCarrier.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

char ssid[] = "AndroidAP8BE4";        // your network SSID (name)
char pass[] = "fgou8655";    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the Wifi radio's status

unsigned int localPort = 2390;      // local port to listen on
const char* IPaddr = "192.168.174.171"; //Ip address to listen to

WiFiUDP Udp;

void setUpMotor(){
  controller.begin();
  Serial.println("reboot");
  controller.reboot();
  delay(500);
  float batteryVoltage = (float)battery.getRaw() / 77;
  Serial.print("Battery voltage: ");
  Serial.println(batteryVoltage);
  M1.setDuty(0);
  M2.setDuty(0);
  M3.setDuty(0);
  M4.setDuty(0);
}

#define Cfoward  0.7;
#define Cturn  0.2;
void motorChange(int forward, int turn){
  //forward -100 to 100, turn -100 to 100
  //Foward and turn value comes in from the controller and is within the range.
  // need to use the setduty functions from -100 to 100 to control motor
  
  //CONTROL SYSTEM NEEDS TO BE IMPLEMENTED START
  int forwardV=0.7*forward;
  int turnV=0.2*turn;
  M3.setDuty(forwardV+turnV);
  M3.setDuty(100); //This is 100 full motor speed
  M4.setDuty(forwardV-turnV);
  //CONTROL SYSTEM NEEDS TO BE IMPLEMENTENTED END
}

void setUpNetwork(){
    while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to network: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }
  Serial.println("You're connected to the network");
  Udp.begin(localPort);
}

void sendInformation(char* data){
   Udp.beginPacket(IPaddr, localPort);
   Udp.write(data);
   Udp.endPacket();
}

#define packetBufferLEN 20
char packetBuffer[packetBufferLEN]; //buffer to hold incoming packet

void resetBuffer(){
  for(int i=0;i<packetBufferLEN;i++){
    packetBuffer[i]=0;
  }
}

char* readInformation(){
  resetBuffer();
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());
    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, packetBufferLEN-1);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    Serial.println("Contents:");
    for (int i=0;i<4;i++){
      Serial.println(packetBuffer[i],HEX);
    }
  }
  return packetBuffer;
}


void setup() {
  Serial.begin(9600);
  //while (!Serial);
  setUpNetwork();
  setUpMotor();
}

char dataSend[] = {1,0};

void loop() {
  
  //DO stuff with recieved data
  char* recievedData = readInformation();
  Serial.println("Reading:");
  Serial.println(recievedData[0],DEC);
  Serial.println(recievedData[1],DEC);
  
  motorChange(recievedData[0], recievedData[1]);

  //SEND gathered data out
  sendInformation(dataSend);

  //DELAY for timing
  delay(100);
}
