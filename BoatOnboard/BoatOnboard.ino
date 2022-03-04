#include <ArduinoMotorCarrier.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

#define DEBUG

char ssid[] = "AndroidAP8BE4";        // your network SSID (name)
char pass[] = "fgou8655";    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the Wifi radio's status

unsigned int localPort = 2390;      // local port to listen on
const char* IPaddr = "192.168.169.171"; //Ip address to listen to

int connectedController = 0;

WiFiUDP Udp;

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
   data[2]=0;
   for (int i=0;i<2;i++){data[i]+=1;}
   Udp.beginPacket(IPaddr, localPort);
   Udp.write(data);
   Udp.endPacket();
   for (int i=0;i<2;i++){data[i]-=1;}
}

#define packetBufferLEN 20
char packetBuffer[packetBufferLEN]; //buffer to hold incoming packet

void resetBuffer(){
  for(int i=0;i<packetBufferLEN;i++){
    packetBuffer[i]=0;
  }
}

char* readInformation(){
  packetBuffer[4]=0;
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    resetBuffer();
    IPAddress remoteIp = Udp.remoteIP();
    #ifdef DEBUG
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());
    #endif
    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, packetBufferLEN-1);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    for (int i=0;i<5;i++){packetBuffer[i]-=1;}

  }
   #ifdef DEBUG
   Serial.println("Contents:");
   for (int i=0;i<5;i++){
     Serial.println(packetBuffer[i],HEX);
   }
   #endif
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
  
  motorChange(recievedData[0], recievedData[1]);

  connectedController = recievedData[4];

  //SEND gathered data out
  sendInformation(dataSend);

  //DELAY for timing
  delay(100);
}
