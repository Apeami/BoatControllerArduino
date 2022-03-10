#include <WiFiNINA.h>
#include <WiFiUdp.h>

WiFiUDP Udp;

int status = WL_IDLE_STATUS;     // the Wifi radio's status
void setUpNetwork(unsigned int localPort, const char* ssid, const char* pass){
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

void sendInformation(char* data, int len, const unsigned int localPort, const char* IPaddr){
   data[len]=0;
   for (int i=0;i<len;i++){data[i]+=1;} //Increase number
   Udp.beginPacket(IPaddr, localPort);
   Udp.write(data);
   Udp.endPacket();
   for (int i=0;i<len;i++){data[i]-=1;} //Decrease Number
}

#define packetBufferLEN 20
char packetBuffer[packetBufferLEN]; //buffer to hold incoming packet

void resetBuffer(){
  for(int i=0;i<packetBufferLEN;i++){
    packetBuffer[i]=0;
  }
}

char* readInformation(int successBit,int estLen){
  packetBuffer[successBit]=0;
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
    for (int i=0;i<len;i++){packetBuffer[i]-=1;}
  }
   #ifdef DEBUG
   Serial.println("Contents:");
   for (int i=0;i<estLen;i++){
     Serial.println(packetBuffer[i],HEX);
   }
   #endif
  return packetBuffer;
}
