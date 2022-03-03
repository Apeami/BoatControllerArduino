#include <WiFiNINA.h>
#include <WiFiUdp.h>

char ssid[] = "AndroidAP8BE4";        // your network SSID (name)
char pass[] = "fgou8655";    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the Wifi radio's status

unsigned int localPort = 2390;      // local port to listen on
const char* IPaddr = "192.168.174.11"; //Ip address to listen to

WiFiUDP Udp;

const int LED_RED = 2;
const int LED_GREEN = 3;
const int BUT_1 = 4;
const int BUT_2 = 5;
const int POT_1 = A0;
const int POT_2 = A1;

void setUpController(){
  pinMode(LED_RED,OUTPUT);
  pinMode(LED_GREEN,OUTPUT);
  pinMode(BUT_1,INPUT);
  pinMode(BUT_2,INPUT);
}

void prepareData(char* buffer){
  buffer[0] = analogRead(POT_1)/11;
  buffer[1] = analogRead(POT_2)/11;

  buffer[2] = digitalRead(BUT_1);
  buffer[3] = digitalRead(BUT_2);
}

void setTestLED(int value){
  if (value==0){digitalWrite(LED_RED, LOW);}
  if (value==1){digitalWrite(LED_RED, HIGH);}
}

void setUpNetwork(){
    while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to network: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }
  // you're connected now, so print out the data:
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
    for (int i=0;i<2;i++){
      Serial.println(packetBuffer[i],HEX);
    }
  }
  return packetBuffer;
}


void setup() {
  Serial.begin(9600);
  //while (!Serial);
  setUpNetwork();
  setUpController();
}

char dataSend[] = {0,0,0,0};

void loop() {
  
  //DO stuff with recieved data
  char* recievedData = readInformation();
  setTestLED(recievedData[0]);

  //SEND gathered data out
  prepareData(dataSend);
  Serial.println("Reading:");
  for (int i=0;i<4;i++){
      Serial.println(dataSend[i],HEX);
    }
  sendInformation(dataSend);

  //DELAY for timing
  delay(100);
}
