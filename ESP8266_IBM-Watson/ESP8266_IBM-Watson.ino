#include <ESP8266WiFi.h>
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient/releases/tag/v2.3

//-------- Customise these values -----------
//const char* ssid = "OpenFCT";
//const char* password = "";

//const char* ssid = "JAFNetwork";
//const char* password = "1234567890";

//const char* ssid = "VFD 600";
//const char* password = "1234567890";

const char* ssid = "RICS_LAB";
const char* password = "rics2014lab";

//const char* ssid = "Vodafone-D4F7CF";
//const char* password = "54117E6C3E";


#define ORG "rfh2dd"
#define DEVICE_TYPE "ESP8266_01"
#define DEVICE_ID "ESP8266_01"
//#define TOKEN 

#define authMethod "use-token-auth"
#define clientId "d:rfh2dd:ESP8266-Type:ESP8266-1"
#define token "tese12345"
#define server "rfh2dd.messaging.internetofthings.ibmcloud.com"

int temperature = 27;
float pH = 7.2;
int orp = 700;
int pressure = 1;
int water_level = 2;
int chlorine_level = 1;
int acid_level = 2;
int security = 0;



//-------- Customise the above values --------

//char server[] = ORG ".messaging.internetofthings.ibmcloud.com";
char topic[] = "iot-2/evt/status/fmt/json";
//char authMethod[] = "use-token-auth";
//char token[] = TOKEN;
//char clientId[] = "d:" ORG ":" DEVICE_TYPE ":" DEVICE_ID;

WiFiClient wifiClient;
PubSubClient client(server, 1883, NULL, wifiClient);

void setup() {
 Serial.begin(115200);
 Serial.println();

 Serial.print("Connecting to "); Serial.print(ssid);
 WiFi.begin(ssid, password);
 while (WiFi.status() != WL_CONNECTED) {
 delay(500);
 Serial.print(".");
 } 
 Serial.println("");

 Serial.print("WiFi connected, IP address: "); Serial.println(WiFi.localIP());
}

int counter = 0;

void loop() {

 if (!client.connected())
 {
   Serial.print("Reconnecting client to ");
   Serial.println(server);
   while (!client.connect(clientId, authMethod, token))
   {
     Serial.print(".");
     delay(500);
   }
   Serial.println();
 }

 String payload = "{\"d\":{\"pH\":";
 payload += String(pH);
 payload += ",\"ORP\":";
 payload += String(orp);
 payload += ",\"temp\":";
 payload += String(temperature);
 payload += ",\"pressure\":";
 payload += String(pressure);
 payload += ",\"w_l\":";
 payload += String(water_level);
 payload += ",\"c_l\":";
 payload += String(chlorine_level);
 payload += ",\"a_l\":";
 payload += String(acid_level);
 payload += ",\"sec\":";
 payload += String(security);
 payload += "}}";
 
 
 Serial.print("Sending payload: ");
 Serial.println(payload);
 
 if (client.publish(topic, (char*) payload.c_str())) {
 Serial.println("Publish ok");
 } else {
 Serial.println("Publish failed");
 }


 delay(5000);
}
