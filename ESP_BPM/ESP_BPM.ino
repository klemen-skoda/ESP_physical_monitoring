#if defined(ESP8266)
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#else
#include <WiFi.h>          //https://github.com/esp8266/Arduino
#endif

//needed for library
#if defined(ESP8266)
#include <ESP8266WebServer.h>
#else
#include <WebServer.h>
#endif
#include <DNSServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager


#include <PubSubClient.h>

// select which pin will trigger the configuration portal when set to LOW
// ESP-01 users please note: the only pins available (0 and 2), are shared 
// with the bootloader, so always set them HIGH at power-up
#define TRIGGER_PIN 0
#define HR_PIN 34
#define GSR_PIN 32


//#include "BME280.h"
//#include "DS18B20.h"

const char* ssid     = "AP_Monitoring";
const char* password = "HPMonitoring";

#define mqtt_server "10.0.0.1"
#define mqtt_user "hpm"
#define mqtt_password "hpm5281"

int IBI = 0;
int BPM = 0;
int peak = 0;
bool crossed = false;
int treshold_down = 2000;
int treshold_up = 4000;
int first_beat = true;
int timer = 0;
int last_data_time = 0;
int lastPeak = 0;
int startIndex = 0;
int signalBPM = 0;

int initBuffer = 0;
int writeIndex = 0;
int IBIbuffer[10];

long previousMillis = 0;

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("\n Starting");

  pinMode(TRIGGER_PIN, INPUT);

  WiFiManager wifiManager;
  wifiManager.setTimeout(600);
  
  //fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  //wifiManager.autoConnect("AutoConnectAP");
  //or use this for auto generated name ESP + ChipID
  //wifiManager.autoConnect();

  
  while(!wifiManager.autoConnect("HR_monitor", "hrmp2018")) {
    //Serial.println("Failed to connect and hit timeout!");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
  }

  client.setServer(mqtt_server, 8193);
  //client.setServer(mqtt_server, 1883);
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  int repeated = 0;
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // If you do not want to use a username and password, change next line to
    //if (client.connect("ESP32Client")) {
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      repeated++;
      if(repeated > 4){
        delay(5000);
        ESP.restart();
        delay(2000);
      }
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void sendState(char data[150]) {
  char topicStr[] = "data/HRM0000";
  Serial.println(data);
  if (client.publish(topicStr, data)) {
    Serial.print(F("successfully sent\r"));
    //writeToSD(file_name, data);
  } else {
    Serial.print(F("unsuccessfully sent\r"));
    //writeToSD(file_name, data);
  }
}

void HR_detection(){
  //Serial.printf("before settings heap size: %u\n", ESP.getFreeHeap());
  char data[150] = "\0";
  signalBPM = analogRead(HR_PIN);
  //sprintf(data, "{\"ID\": \"HRM0000\",\"HRsignal\": %d}", signalBPM);
  //sendState(data);
  //data[0] = '\0';
  //Serial.println(signalBPM);
  int timerTmp = millis();
  
  //here we seak the peak of pulse signal
  if (signalBPM > treshold_down) {
    if (peak < signalBPM) {
      peak = signalBPM;
    }
  //once the signal goes below the lower treshold and we have a peak thats above that treshold we check what that peak would be
  } else if (signalBPM < treshold_down and peak > treshold_down and startIndex > 5) {
    int timer1 = 0;
    timer1 = millis();
    
    //we check if peak is not above the upper treshold, if it would be that would most likely be noise
    if (peak < treshold_up) {
      //we cheak if the peak is in +-15% of the last known peak 
      if (first_beat == false and lastPeak * 0.85 < peak and lastPeak * 1.15 > peak) {
        //we calculate IBI(time in between beats)
        lastPeak = peak;
        IBI = timer1 - timer; //if time waaaaay too big ignore IBI
        if(IBI < 3000){
          timer = timer1;
          IBIbuffer[writeIndex] = IBI;
          writeIndex++;
  
          //at startup we fill out buffer so when we calculate BPM any errors are muffled
          //when buffer is filled it afts as a round trip buffer FIFO
          if(initBuffer == 10){
            int sumIBI = 0;
            for(int i = 0; i<10; i++){
              Serial.println(IBIbuffer[i]);
              sumIBI += IBIbuffer[i];
            }
            //here we calculate beat from the sum of last 10 IBI intervals
            BPM = int(60000/int(sumIBI/10));
            //Serial.println(BPM);
          }else{
            initBuffer++;
          }
  
          if (writeIndex == 10){
            writeIndex = 0;
          }
          Serial.println("-------------------------------------------------------");
          Serial.println(IBI);
          Serial.println(60000/IBI);
          Serial.println(peak);
          //int resultTmp = millis()-timerTmp;
          //Serial.println(resultTmp);
        } else if(first_beat == true) {
          timer = timer1;
          first_beat = false;
        }
      }
    //if we get peak that is out of bounds we restart the code so the IBI time because only legal IBI time is the one between 2 consecutive legal peaks
    } else {
      first_beat = true;
    }
    //when every legal peak is sensed new BPM is sent along with GSR sensor value
    if(BPM != 0){
      sprintf(data, "{\"ID\": \"HRM0000\",\"BPM\": %d, \"GSR\": %d}", BPM, analogRead(GSR_PIN));
      sendState(data);
    }
    peak = 0;
  //at startup we seak for max peak, just so we are sure that start peak we get is legal
  } else if (peak > 0 and signalBPM < treshold_down) {
    Serial.println(peak);
    //Serial.println(startIndex);
    if (lastPeak < peak) {
      lastPeak = peak;
    }
    startIndex++;
    peak = 0;
  }
  //we do the measurement every 5 millisecondss
  delay(5-(millis()-timerTmp));
  
  if(millis()-previousMillis >= 1000){
    previousMillis = millis();
    data[0] = '\0';
    if(BPM != 0){
      sprintf(data, "{\"ID\": \"HRM0000\",\"BPM\": %d, \"GSR\": %d}", BPM, analogRead(GSR_PIN));
      sendState(data);
    }else{
      sprintf(data, "{\"ID\": \"HRM0000\", \"GSR\": %d}", analogRead(GSR_PIN));
      sendState(data);
    }
  }
  //Serial.printf("settings heap size: %u\n", ESP.getFreeHeap());
}


void loop() {
  // is configuration portal requested?
  if ( digitalRead(TRIGGER_PIN) == LOW ) {
    //WiFiManager
    //Local intialization. Once its business is done, there is no need to keep it around
    WiFi.disconnect(true);
    WiFiManager wifiManager;
    wifiManager.setTimeout(600);
    
    if (!wifiManager.startConfigPortal("HR_monitor")) {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }

    //if you get here you have connected to the WiFi
    Serial.println("connected!");
  }

  if (!client.loop()) {
    reconnect();
  }
  
  HR_detection();


  // put your main code here, to run repeatedly:

}
