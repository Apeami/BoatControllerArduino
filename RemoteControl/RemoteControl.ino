#include <WiFiNINA.h>
#include <WiFiUdp.h>

#define DEBUG

char ssid[] = "AndroidAP8BE4";        // your network SSID (name)
char pass[] = "fgou8655";    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the Wifi radio's status

unsigned int localPort = 2390;      // local port to listen on
const char* IPaddr = "192.168.169.11"; //Ip address to listen to

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

int cu1=0;
int cu2=0;
void prepareData(char* buffer){
  buffer[0] = analogRead(POT_1)/11;
  buffer[1] = analogRead(POT_2)/11;


   buffer[2] = digitalRead(BUT_1);
   buffer[3] = digitalRead(BUT_2);

   if (buffer[2]==1){cu1++;}else{cu1=0;}
   if (buffer[3]==1){cu2++;}else{cu2=0;}
   if (cu1>10){buffer[2]=1;}else{buffer[2]=0;}
   if (cu2>10){buffer[3]=1;}else{buffer[3]=0;}
    
  Serial.println("Input Values");
  Serial.println(buffer[0],HEX);
  Serial.println(buffer[1],HEX);
  Serial.println(buffer[2],HEX);
  Serial.println(buffer[3],HEX);

  buffer[4] = 1;
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
   data[5]=0;
   for (int i=0;i<5;i++){data[i]+=1;}
   Udp.beginPacket(IPaddr, localPort);
   Udp.write(data);
   Udp.endPacket();
   for (int i=0;i<5;i++){data[i]-=1;}
}



#define packetBufferLEN 20
char packetBuffer[packetBufferLEN]; //buffer to hold incoming packet

void resetBuffer(){
  for(int i=0;i<packetBufferLEN;i++){
    packetBuffer[i]=0;
  }
}

char* readInformation(){
  packetBuffer[0]=0;
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
    for (int i=0;i<2;i++){packetBuffer[i]-=1;}
  }
   #ifdef DEBUG
   Serial.println("Contents:");
   for (int i=0;i<2;i++){
     Serial.println(packetBuffer[i],HEX);
   }
   #endif
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
  sendInformation(dataSend);

  //DELAY for timing
  delay(100);
}
