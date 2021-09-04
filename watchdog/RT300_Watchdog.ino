// RT300 Watchdog
// IP watchdog and web based power switch for the radio telescope of the radebeul observatory

#include <EtherCard.h>

static byte mymac[] = { 0x00,0x22,0xF9,0x01,0x70,0xF4 };
static byte hisip[] = { 172,16,2,12 }; // ping target IP
byte Ethernet::buffer[1400];

char*         pwd            =  "fill-password-here";
boolean       password_valid =   false;
boolean       watchdog_state =   false;
boolean       relay_state    =   false;
const int     STATUSLED      =       1;
const int     RELAYLED1      =      13;
const int     RELAYLED2      =      12;
const int     RELAY1         =      11;
const int     RELAY2         =      10;
const byte    MAXCYCLES      =       4;
const long    PING_DELAY     = 5000000;
unsigned long ping_timer     =       0;
int           ping_cycles    =       0;

void setup () {

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

  ether.begin(sizeof Ethernet::buffer, mymac);
  ether.dhcpSetup();
  ether.copyIp(ether.hisip, hisip);

  while (ether.clientWaitingGw())
    ether.packetLoop(ether.packetReceive());
}
 
void loop () {

  word len = ether.packetReceive(); 
  word pos = ether.packetLoop(len);

  //IP Watchdog
  if (watchdog_state == true) {
   if ((micros() - ping_timer) >= PING_DELAY) {
    digitalWrite(STATUSLED, HIGH);
    ping_cycles++;
    ping_timer = micros();
    ether.clientIcmpRequest(ether.hisip);
   }
   if (len > 0 && ether.packetLoopIcmpCheckReply(ether.hisip)) {
    ping_cycles = 0;
    digitalWrite(STATUSLED, LOW);
   }
   if (ping_cycles >= MAXCYCLES) {
    digitalWrite(RELAY1, HIGH);
    digitalWrite(RELAY2, HIGH);
    watchdog_state = false;
    relay_state    = false;
    digitalWrite(RELAYLED1, LOW);
    digitalWrite(RELAYLED2, LOW);    
    digitalWrite(STATUSLED, HIGH);    
   }
  }

  if(pos) {

    //POST incoming?
    if(strstr((char *)Ethernet::buffer + pos, "POST /") != 0) {

      // search and verify password
      char password[20];      
      char* password_position = strstr((char *)Ethernet::buffer + pos, "&pwd=");
      if(password_position != 0) {
        strcpy(password, password_position + 5);
        if(strcmp(password, pwd) == 0) {
          password_valid = true;                  
        } else {
           password_valid = false;
        }
      }
      
      if(password_valid) {

        if(strstr((char *)Ethernet::buffer + pos, "PWR_OFF=") != 0) {
          digitalWrite(RELAY1, HIGH);
          digitalWrite(RELAY2, HIGH);
          relay_state = false;
          digitalWrite(RELAYLED1, LOW);
          digitalWrite(RELAYLED2, LOW);          

        } else if(strstr((char *)Ethernet::buffer + pos, "PWR_ON=") != 0) {
          digitalWrite(RELAY1, LOW);
          digitalWrite(RELAY2, LOW);
          relay_state = true; 
          digitalWrite(RELAYLED1, HIGH);
          digitalWrite(RELAYLED2, HIGH);
        }
        
        if(strstr((char *)Ethernet::buffer + pos, "WDT_OFF=") != 0) {
          watchdog_state = false;
          digitalWrite(STATUSLED, HIGH);
 
        } else if(strstr((char *)Ethernet::buffer + pos, "WDT_ON=") != 0) {
          ping_timer  = 0;
          ping_cycles = 0;
          watchdog_state = true;
          digitalWrite(STATUSLED, LOW);
        }
      }
    }      
    
    //HTML Page        
    BufferFiller bfill = ether.tcpOffset();
    bfill.emit_p(PSTR(
     "HTTP/1.1 200 OK\r\n"
     "Content-Type: text/html;charset=utf-8\r\n"
     "Pragma: no-cache\r\n"
     "cache-control: no-cache\r\n"
     "\r\n"
     "<!DOCTYPE html><html><head><title>RT300 Watchdog</title>"
     "<style>body{font-family:Helvetica,Arial,sans-serif;font-size:120%;color:#f7692c;background-color:#242424;overflow:hidden}button{width:85px}button:enabled,button[enabled]{background-color:grey}input{background-color:darkgrey;color:#242424;width:78px;height:9px}</style>"
     "<script src=\"https://code.jquery.com/jquery-3.6.0.min.js\"></script><script type=\"text/javascript\">jQuery(document).ready(function(){setInterval(function(){jQuery(\"#controls\").load(window.location.href + \" #controls\" );}, 10000);});</script>"
     "</head><html><body><iframe src=\"http://172.16.2.11/rtcam/\" width=\"800\" height=\"600\" scrolling=\"no\" style=\"border:0\"></iframe><br>"
     "<div id=\"controls\"><form method=\"POST\"><table><tr><td>Main Power&nbsp;</td>"));
    
    if(relay_state == true) bfill.emit_p(PSTR("<td><button name=\"PWR_ON\" style=\"background-color:lime;color:black\" disabled>PWR_ON</button></td><td><button name=\"PWR_OFF\">PWR_OFF</button></td></tr>"));
    else bfill.emit_p(PSTR("<td><button name=\"PWR_ON\">PWR_ON</button></td><td><button name=\"PWR_OFF\" style=\"background-color:red;color:black\" disabled>PWR_OFF</button></td></tr>"));

    if(watchdog_state == true) bfill.emit_p(PSTR("<tr><td>IP Watchdog&nbsp;</td><td><button name=\"WDT_ON\" style=\"background-color:lime;color:black\" disabled>WDT_ON</button></td><td><button name=\"WDT_OFF\">WDT_OFF</button></td></tr>"));
    else bfill.emit_p(PSTR("<tr><td>IP Watchdog&nbsp;</td><td><button name=\"WDT_ON\">WDT_ON</button></td><td><button name=\"WDT_OFF\" style=\"background-color:red;color:black\" disabled>WDT_OFF</button></td></tr>"));

    bfill.emit_p(PSTR("<tr><td></td><td></td><td><input type=\"password\" name=\"pwd\" placeholder=\"Password\"></td></tr></table></form></div></body></html>"));

    ether.httpServerReply(bfill.position());
  }  
}
