#include <ESP8266WiFi.h>
#include <MicroGear.h>
#include <time.h>

const char* ssid     = "Hooheinz";
const char* password = "codeunlock";

#define APPID   "SmartPlant"
#define KEY     "kr5mzn2NBDJ9EOU"
#define SECRET  "uFBdXgz8k2XkquT2UM11A0s1d"
#define ALIAS   "nodemcu"

WiFiClient client;

int timer = 0;
int i = 1;
char in[10];
int minn = 0;
int th = 0, tm = 0;
MicroGear microgear(client);

//int LED = D4;

/* If a new message arrives, do this */
void onMsghandler(char *topic, uint8_t* msg, unsigned int msglen) {
  for (int i = 0; i < msglen; i++)
  {
    Serial.print((char)msg[i]);
  }
}

void onFoundgear(char *attribute, uint8_t* msg, unsigned int msglen) {
  /*
    Serial.print("Found new member --> ");
    for (int i = 0; i < msglen; i++)
    Serial.print((char)msg[i]);

    Serial.println();
  */
}

void onLostgear(char *attribute, uint8_t* msg, unsigned int msglen) {
  /*
    Serial.print("Lost member --> ");
    for (int i = 0; i < msglen; i++)
    Serial.print((char)msg[i]);
    Serial.println();
  */
}

/* When a microgear is connected, do this */
void onConnected(char *attribute, uint8_t* msg, unsigned int msglen) {
  //Serial.println("Connected to NETPIE...");
  /* Set the alias of this microgear ALIAS */
  microgear.setAlias(ALIAS);
}


void setup() {
  /* Add Event listeners */

  /* Call onMsghandler() when new message arraives */
  microgear.on(MESSAGE, onMsghandler);

  /* Call onFoundgear() when new gear appear */
  microgear.on(PRESENT, onFoundgear);

  /* Call onLostgear() when some gear goes offline */
  microgear.on(ABSENT, onLostgear);

  /* Call onConnected() when NETPIE connection is established */
  microgear.on(CONNECTED, onConnected);


  Serial.begin(115200);
  //Serial.println("Starting...");

  /* Initial WIFI, this is just a basic method to configure WIFI on ESP8266.                       */
  /* You may want to use other method that is more complicated, but provide better user experience */
  if (WiFi.begin(ssid, password)) {
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print("-");
    }
  }
  /*
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  */
  /* Initial with KEY, SECRET and also set the ALIAS here */
  microgear.init(KEY, SECRET, ALIAS);

  /* connect to NETPIE to a specific APPID */
  microgear.connect(APPID);
  //pinMode(LED, OUTPUT);
  in[1] = '\0';
}

int space = 0;
char cHumid[40];
int humid = 0;
void loop() {

  /* To check if the microgear is still connected */
  if (microgear.connected()) {
    //Serial.println("connected");

    /* Call this method regularly otherwise the connection may be lost */
    microgear.loop();

    while (Serial.available() > 0) {
      in[0] = Serial.read();
      if ((in[0] >= '0' && in[0] <= '9') || in[0] == '+') {
        cHumid[space++] = in[0];
      } else if (in[0] == ' ') {
        cHumid[space] = '\0';
        space = 0;
        microgear.publish("/dirt", cHumid);
        //Serial.println(cHumid);
      } else if (in[0] == 'F') {
        microgear.chat("nodemcu", "F");
      } else if (in[0] == 'W') {
        microgear.chat("nodemcu", "W");
      }
    }
  }
  else {
    Serial.print("x");
    //Serial.println("connection lost, reconnect...");
    if (timer >= 5000) {
      microgear.connect(APPID);
      timer = 0;
    }
    else timer += 100;
  }
  delay(100);
}
