#include "networkControl.h"
#include <WiFiNINA.h>
#include <WiFiUdp.h>

#define DEBUG

char ssid[] = "AndroidAP8BE4";        // your network SSID (name)
char pass[] = "fgou8655";    // your network password (use for WPA, or use as key for WEP)

unsigned int localPort = 2390;      // local port to listen on
const char* IPaddr = "192.168.122.38"; //Ip address to listen to

int systemON = 0;

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

  //Debounce stuff
  if (buffer[2]==1){cu1++;}else{cu1=0;}
  if (buffer[3]==1){cu2++;}else{cu2=0;}
  if (cu1>10){buffer[2]=1;}else{buffer[2]=0;}
  if (cu2>10){buffer[3]=1;}else{buffer[3]=0;}
  
  #ifdef DEBUG
  Serial.println("Input Values");
  Serial.println(buffer[0],HEX);
  Serial.println(buffer[1],HEX);
  Serial.println(buffer[2],HEX);
  Serial.println(buffer[3],HEX);
  #endif
  
  buffer[4] = 1;
}

void setTestLED(int value){
  if (value==0){digitalWrite(LED_RED, LOW);}
  if (value==1){digitalWrite(LED_RED, HIGH);}
}

void checkForON(char* dataSend){
  static int prevSend=0;
  if (dataSend[2]==1 && prevSend==0){
    if (systemON==1){
      systemON=0;
      dataSend[4] = 0;
      digitalWrite(LED_GREEN, LOW);
    }else if (systemON==0){
      systemON=1;
      dataSend[4] = 1;
      digitalWrite(LED_GREEN, HIGH);
    }
  }
  prevSend=dataSend[2];
}

void enableSystem(char* dataSend){
  if (systemON==0){
    dataSend[4] = 0;
  }else if (systemON==1){
    dataSend[4] = 1;
  }
}

void setup() {
  Serial.begin(9600);
  //while (!Serial);
  setUpNetwork(localPort, ssid, pass);
  setUpController();
}

char dataSend[] = {0,0,0,0,0};

void loop() {

  
  //DO stuff with recieved data
  char* recievedData = readInformation(0,2);
  setTestLED(recievedData[0]);

  //SEND gathered data out
  prepareData(dataSend);
  checkForON(dataSend);
  enableSystem(dataSend);
  sendInformation(dataSend,5, localPort, IPaddr);

  //DELAY for timing
  delay(100);
}
