// RT300 Watchdog
// IP watchdog and web based power switch with MQTT support for the radio telescope of the radebeul observatory

#include <SPI.h>
#include <Ethernet.h>     //https://github.com/masterx1981/Ethernet.git
#include <EthernetICMP.h> //https://github.com/masterx1981/Ethernet.git
#include <PubSubClient.h> //https://github.com/knolleary/pubsubclient.git

static byte mac[] = {0x00,0x22,0xF9,0x01,0x70,0xF4};
IPAddress pingAddr(172,16,2,12);

SOCKET pingSocket = 1;
EthernetClient ethClient;
EthernetClient pingClient;
PubSubClient mqttclient(ethClient);
EthernetICMPPing ping(pingSocket, (uint16_t)random(0, 255));

boolean             watchdog_state = false;
boolean             relay_state    = false;
const int           RELAY1         =     3;
const int           RELAY2         =     4;
const int           RELAYLED1      =     5;
const int           RELAYLED2      =     6;
const int           STATUSLED      =     7;
const byte          max_cycles     =     3;
const unsigned long interval       =  5000;
unsigned long       timer          =     0;
int                 ping_cycles    =     0;

void setup() {
  digitalWrite(STATUSLED, HIGH);
  digitalWrite(RELAYLED1, LOW);
  digitalWrite(RELAYLED2, LOW);
  digitalWrite(RELAY1, HIGH);
  digitalWrite(RELAY2, HIGH);
  pinMode(STATUSLED,OUTPUT);
  pinMode(RELAYLED1,OUTPUT);
  pinMode(RELAYLED2,OUTPUT);
  pinMode(RELAY1,OUTPUT);
  pinMode(RELAY2,OUTPUT);
   
  Ethernet.begin(mac);
  mqttclient.setServer("antrares.tk", 43580);
  mqttclient.setCallback(callback);
}

void callback(char* topic, byte* payload, unsigned int length) {
    payload[length] = '\0';
    String MqttMessage = String((char*)payload);

 if (MqttMessage == "WDT_OFF"){
    watchdog_state = false;
    digitalWrite(STATUSLED, HIGH);
  }
  if (MqttMessage == "WDT_ON"){
    ping_cycles = 0;
    watchdog_state = true;
    digitalWrite(STATUSLED, LOW);
  }
  if (MqttMessage == "PWR_OFF"){
    digitalWrite(RELAY1, HIGH);
    digitalWrite(RELAY2, HIGH);
    relay_state = false;
    digitalWrite(RELAYLED1, LOW);
    digitalWrite(RELAYLED2, LOW);      
    }
    if (MqttMessage == "PWR_ON"){
      digitalWrite(RELAY1, LOW);
      digitalWrite(RELAY2, LOW);
      relay_state = true;
      digitalWrite(RELAYLED1, HIGH);
      digitalWrite(RELAYLED2, HIGH);      
  }
  MqttReport();
}

void reconnect() {
  if (mqttclient.connect("STW-RT300_Watchdog")) {
    mqttclient.subscribe("STW/RT300/WATCHDOG/command");
  } else {
      delay(5000);
    }
}

void PingRequest() {
  digitalWrite(STATUSLED, HIGH);
  EthernetICMPEchoReply echoReply = ping(pingAddr, 4);
  if (echoReply.status == SUCCESS) {
    ping_cycles = 0;
    digitalWrite(STATUSLED, LOW);
  } else {
      ping_cycles++;
      if (ping_cycles >= max_cycles) {
        digitalWrite(RELAY1, HIGH);
        digitalWrite(RELAY2, HIGH);
        watchdog_state = false;
        relay_state    = false;
        digitalWrite(RELAYLED1, LOW);
        digitalWrite(RELAYLED2, LOW);    
        digitalWrite(STATUSLED, HIGH);    
      }
    }
  timer = millis();
  MqttReport();
}

void MqttReport() {
  Ethernet.maintain();
  if (!mqttclient.connected()) {
    reconnect();
  }
  if (!watchdog_state && !relay_state) {
    mqttclient.publish("STW/RT300/WATCHDOG/status", "0|0" );
  } 
  if (!watchdog_state && relay_state) {
    mqttclient.publish("STW/RT300/WATCHDOG/status", "0|1" );
    }
  if (watchdog_state && !relay_state) {
    mqttclient.publish("STW/RT300/WATCHDOG/status", "1|0" );
  } 
  if (watchdog_state && relay_state) {
    mqttclient.publish("STW/RT300/WATCHDOG/status", "1|1" );
    }
  if (!watchdog_state) {
    timer = millis();
  }
}

void loop() {
  if (millis() - timer >= interval && watchdog_state) {
    PingRequest();    
  }
  if (millis() - timer >= interval && !watchdog_state) {
    MqttReport();    
  } 
  mqttclient.loop();
}
