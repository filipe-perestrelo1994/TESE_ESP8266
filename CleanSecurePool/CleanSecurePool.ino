/*ANTES DE COMPILAR OU FAZER ALTERAÇÕES, LER O FINAL DO FICHEIRO*/

//INTEGRAÇÂO DE TUDO!
#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient/releases/tag/v2.3
#include <Wire.h>
#include <PID_v1.h>


//VARIÁVEIS DE INÍCIO
int w_l2 = -1, c_l2 = -1, a_l2 = -1; //VARIÀVEIS DE NOTIFICAÇÕES
int servidorON = 0; //CONTROLO DO SERVIDOR

int security = 0;

int temp_threshold = 25; //Controlo da temperatura
boolean SecurityOn = false; //Controlo de afogamentos
boolean MaintenanceOn = true; //Controlo da manutenção

boolean waterLevelOK = true;
boolean acidLevelOK = true;
boolean chlorineLevelOk = true;

boolean pressureOk = true;

boolean PumpOff = false;

int i = 0;

/********************************************************************************TEMPORIZADOR*******************************************************************************************/
extern "C" {
#include "user_interface.h"}


  //os_timer_t mTimer;
  os_timer_t timerCSP;
  
  volatile bool _timeout = false;

  //timer Trigger
  void tCallback(void *tCall) {
    _timeout = true;
  }

  void usrInit(void) {
    os_timer_setfn(&timerCSP, tCallback, NULL);
    os_timer_arm(&timerCSP, 2400000, true);
  }
/******************************************************************************FIM DO TEMPORIZADOR*******************************************************************************************/


/********************************************************************************VARIÁVEIS*******************************************************************************************/

//-------------------------------------------------------------------------VARIÁVEIS DO SERVIDOR--------------------------------------------------------------------------------------------------------
WiFiServer wifiServer(80); //SERVIDOR
//ESP8266WebServer wifiServer(80);
//-------------------------------------------------------------------------FIM DAS VARIÁVEIS DO SERVIDOR--------------------------------------------------------------------------------------------------------



  

  //-------------------------------------------------------------------------VARIÁVEIS DA CLOUD--------------------------------------------------------------------------------------------------------
  //-------- Customise these values -----------

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




  //-------- Customise the above values --------

  //char server[] = ORG ".messaging.internetofthings.ibmcloud.com";
  char topic[] = "iot-2/evt/status/fmt/json";
  //char authMethod[] = "use-token-auth";
  //char token[] = TOKEN;
  //char clientId[] = "d:" ORG ":" DEVICE_TYPE ":" DEVICE_ID;

  WiFiClient wifiClient;
  PubSubClient client(server, 1883, NULL, wifiClient);
  //---------------------------------------------------------FIM DAS VARIÁVEIS DA CLOUD--------------------------------------------------------------------------------------------------------



  //----------------------------------------------------------------------VARIÁVEIS DO ACELERÓMETRO--------------------------------------------------------------------------------------------------------
//#define DEBUG true
//#define Serial if(DEBUG)Serial

#define DEVICE (0x53) // Device address as specified in data sheet
  float last_value = 0;
  float prelast_value = 0;

  int show_count = 0;
  int trigger_count = 0;
  float trigger_value = -5; //DEFAULT VALUE ???
  float current_value_x = 0;
  float current_value_y = 0;
  float current_value_z = 0;


#define ADXL345_MG2G_MULTIPLIER (0.004)
#define SENSORS_GRAVITY_STANDARD          (SENSORS_GRAVITY_EARTH)
#define SENSORS_GRAVITY_EARTH             (9.80665F)              /**< Earth's gravity in m/s^2 */

  byte _buff[6];
  char POWER_CTL = 0x2D;    //Power Control Register
  char DATA_FORMAT = 0x31;
  char DATAX0 = 0x32;    //X-Axis Data 0
  char DATAX1 = 0x33;    //X-Axis Data 1
  char DATAY0 = 0x34;    //Y-Axis Data 0
  char DATAY1 = 0x35;    //Y-Axis Data 1
  char DATAZ0 = 0x36;    //Z-Axis Data 0
  char DATAZ1 = 0x37;    //Z-Axis Data 1

  float max_x = 0;
  float min_x = 0;
  float cal_x = 0;
  float x = 0;

  float max_y = 0;
  float min_y = 0;
  float cal_y = 0;
  float y = 0;

  float max_z = 0;
  float min_z = 0;
  float cal_z = 0;
  float z = 0;
  //------------------------------------------------------------FIM DAS VARIÁVEIS DO ACELERÓMETRO--------------------------------------------------------------------------------------------------------

  //------------------------------------------------------------VARIÁVEIS DA TEMPERATURA--------------------------------------------------------------------------------------------------------
#define ONE_WIRE_BUS 5

  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
  OneWire oneWire(ONE_WIRE_BUS);

  // Pass our oneWire reference to Dallas Temperature.
  DallasTemperature DS18B20(&oneWire);
  char temperatureCString[6];
  //------------------------------------------------------------FIM DAS VARIÁVEIS DA TEMPERATURA--------------------------------------------------------------------------------------------------------

  //------------------------------------------------------------VARIÁVEIS DO PID pH-------------------------------------------------------------------------------------------------------------
  double pH;
  double pH_ref = 7.5; 
  double pH_measured;
  double pH_delay; //reference
  double Kp=0.0103, Ki=8.62*pow(10,-6), Kd=0; //gains (Since this is a PI controller it Kd=0)
  
  //------------------------------------------------------------FIM DAS VARIÁVEIS DO PID pH--------------------------------------------------------------------------------------------------------
  
  /********************************************************************************FIM DAS VARIÁVEIS*******************************************************************************************/


  /*---------------------------------------------FUNÇÕES DO SISTEMA-------------------------------------*/

  /****************************************************************************************MULTIPLEXER***********************************************************************************************/
  boolean readMultiplexer(boolean S0, boolean S1, boolean S2)
  {

    digitalWrite(14, S0);
    digitalWrite(12, S1);
    digitalWrite(13, S2);

    delay(1000);

    boolean y = digitalRead(4);

    delay(1000);

    return y;
  }
  /****************************************************************************************FIM DO MULTIPLEXER***********************************************************************************************/

  /****************************************************************************************NÍVEL DE ÁGUA***********************************************************************************************/
  boolean readWater0()
  {
    int water0 = readMultiplexer(LOW, LOW, LOW);
    return water0;
  }
  boolean readWater1()
  {
    int water1 = readMultiplexer(LOW, LOW, HIGH);
    return water1;
  }

  int defineWaterLevel(boolean water1, boolean water0)
  {
    int w_l;
    if (water1 == LOW && water0 == HIGH)
    {
      Serial.println("Level 0 - Empty");
      w_l = 0;
      delay(1000);
    }

    if (water1 == LOW && water0 == LOW)
    {
      Serial.println("Level 1 - Almost Empty");
      w_l = 1;
      delay(1000);
    }

    if (water1 == HIGH && water0 == HIGH)
    {
      Serial.println("Level 2 - Normal");
      w_l = 2;
      delay(1000);
    }

    if (water1 == HIGH && water0 == LOW)
    {
      Serial.println("Level 3 - Full");
      w_l = 3;
      delay(1000);
    }

    return w_l;
  }
  /****************************************************************************************FIM DO NÍVEL DE ÁGUA***********************************************************************************************/

  /****************************************************************************************NÍVEL DE CLORO***********************************************************************************************/
  boolean readChlorine0()
  {
    int chlorine0 = readMultiplexer(LOW, HIGH, LOW);
    return chlorine0;
  }
  boolean readChlorine1()
  {
    int chlorine1 = readMultiplexer(LOW, HIGH, HIGH);
    return chlorine1;
  }
  int defineChlorineLevel(boolean chlorine1, boolean chlorine0)
  {
    int c_l;
    if (chlorine1 == LOW && chlorine0 == HIGH)
    {
      Serial.println("Level 1 - Almost Empty");
      c_l = 1;
      delay(1000);
    }

    if (chlorine1 == LOW && chlorine0 == LOW)
    {
      Serial.println("Level 2 - Normal");
      c_l = 2;
      delay(1000);
    }

    if (chlorine1 == HIGH && chlorine0 == HIGH)
    {
      Serial.println("Level 0 - Empty");
      c_l = 0;
      delay(1000);
    }
    return c_l;
  }
  /****************************************************************************************FIM DO NíVEL DE CLORO***********************************************************************************************/

  /****************************************************************************************NÍVEL DE ÁCIDO***********************************************************************************************/
  boolean readAcid0()
  {
    int acid0 = readMultiplexer(HIGH, LOW, LOW);
    return acid0;
  }
  boolean readAcid1()
  {
    int acid1 = readMultiplexer(HIGH, LOW, HIGH);
    return acid1;
  }
  int defineAcidLevel(boolean acid1, boolean acid0)
  {
    int a_l;
    if (acid1 == LOW && acid0 == HIGH)
    {
      Serial.println("Level 1 - Almost Empty");
      a_l = 1;
      delay(500);
    }

    if (acid1 == LOW && acid0 == LOW)
    {
      Serial.println("Level 2 - Normal");
      a_l = 2;
      delay(500);
    }

    if (acid1 == HIGH && acid0 == HIGH)
    {
      Serial.println("Level 0 - Empty");
      a_l = 0;
      delay(500);
    }
    return a_l;
  }
  /****************************************************************************************FIM DO NÍVEL DE ÁCIDO***********************************************************************************************/

  /****************************************************************************************DESCODIFICADOR***********************************************************************************************/
  void writeDecoder(boolean S0, boolean S1, int d)
  {


    
    delay(500);
    digitalWrite(15, LOW);
    digitalWrite(3, S0);
    digitalWrite(1, S1);
    
    //0|0 -> Aquecimento
    //0|1 -> Cloro
    //1|0 -> Ácido
    //1|1 -> Bomba

    delay(d);

  }
  /****************************************************************************************FIM DO DESCODIFICADOR***********************************************************************************************/

 /****************************************************************************************PRESSÃO***********************************************************************************************/
float checkFilterPressure()
{
  float pressure = 1;
  return pressure;
}
/****************************************************************************************FIM DA PRESSÃO***********************************************************************************************/
 
 /****************************************************************************************ACELERÓMETRO***********************************************************************************************/

  /*Variáveis*/

/*Ler uma quantidade de bytes do dispositivo*/
  void readFrom(byte address, int num, byte _buff[])
  {
    Wire.beginTransmission(DEVICE); // start transmission to device
    Wire.write(address); // sends address to read from
    Wire.endTransmission(); // end transmission
    Wire.beginTransmission(DEVICE); // start transmission to device
    Wire.requestFrom(DEVICE, num); // request 6 bytes from device

    int i = 0;
    while (Wire.available()) // device may send less than requested (abnormal)
    {
      _buff[i] = Wire.read(); // receive a byte
      i++;
    }
    Wire.endTransmission(); // end transmission
  }
  /*--------------------------------*/

  /*Ler acelerómetro*/
  float readAccel()
  {
    Serial.println("readAccel");
    uint8_t howManyBytesToRead = 6; //6 for all axes
    readFrom( DATAX0, howManyBytesToRead, _buff); //read the acceleration data from the ADXL345
    short x = (((short)_buff[1]) << 8) | _buff[0];
    //readFrom( DATAY0, howManyBytesToRead, _buff);
    short y = (((short)_buff[3]) << 8) | _buff[2];
    //readFrom( DATAZ0, howManyBytesToRead, _buff);
    short z = (((short)_buff[5]) << 8) | _buff[4];
    Serial.println(x * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD);
    Serial.println(y * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD);
    Serial.println(z * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD);
    return z * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;

  }
  /*--------------------------------*/

  /*Escrever no registo*/
  void writeTo(byte address, byte val)
  {
    Wire.beginTransmission(DEVICE); // start transmission to device
    Wire.write(address); // send register address
    Wire.write(val); // send value to write
    Wire.endTransmission(); // end transmission
  }
  /*--------------------------------*/

  
  /*Setup Acelerómetro*/
  void setupAccelerometer()
  {
    Wire.begin(0,2);

    // Put the ADXL345 into +/- 2G range by writing the value 0x01 to the DATA_FORMAT register.
    // FYI: 0x00 = 2G, 0x01 = 4G, 0x02 = 8G, 0x03 = 16G
    writeTo(DATA_FORMAT, 0x00);

    // Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
    writeTo(POWER_CTL, 0x08);

    int i = 0;
    for (i = 0; i < 11; i++)
    {
      //uint8_t howManyBytesToRead = 6;
      //readFrom( DATAX0, howManyBytesToRead, _buff);
      float calib_z ;//= (((short)_buff[1]) << 8) | _buff[0];
      calib_z = readAccel();
      if (i == 0)
        cal_z = z;
      if (i > 0)
        cal_z = cal_z + calib_z;
      Serial.println(calib_z);
      delay(100);
    }

    //cal_z = cal_z / 10;
    Serial.print("cal_z: "); Serial.println(cal_z);

    Serial.println("Accelerometer calibrated!");
    
    return;
  }
  /*--------------------------------*/
  

  /****************************************************************************************FIM DO ACELERÓMETRO***********************************************************************************************/


  /****************************************************************************************TEMPERATURA***********************************************************************************************/
  float getTemperature()
  {
    float tempC;
    DS18B20.requestTemperatures();
    tempC = DS18B20.getTempCByIndex(0);
    delay(100);
    return tempC;
  }
  /*Setup Temperatura*/
  void setupTemperature()
  {
    DS18B20.begin();

    Serial.println("Temperature calibrated!");
  }

  int setTemperature(int temp_threshold, int tempC)
  {
    int  temperature = temp_threshold - tempC;
    if (temperature > 0)
    {
      Serial.println("Writing decoder Temperature");
      writeDecoder(LOW, LOW, 2000);//temperature * 1000
    }
    return temp_threshold;
  }

  /*--------------------------------*/

  /****************************************************************************************FIM DA TEMPERATURA***********************************************************************************************/

  /****************************************************************************************MULTIPLEXER ANALÓGICO***********************************************************************************************/
  int AnalogSensorValue(boolean S0, boolean S1, boolean S2)
  {
  digitalWrite(14,S0);
  digitalWrite(12,S1);
  digitalWrite(13,S2);

  delay(500);
  
  int sensorValue = analogRead(A0);
  sensorValue = map(sensorValue, 0, 1023, 0, 1000);
  Serial.print(sensorValue);
  Serial.println(" mV");
  
  return sensorValue;
  }
  /************************************************************************************FIM DO MULTIPLEXER ANALÓGICO***********************************************************************************************/

  /****************************************************************************************pH***********************************************************************************************/
  double read_pH() //Ler pH
  {
    double pH_mV = (double)AnalogSensorValue(LOW, LOW, LOW);
    return pH_mV;
  }

  double calculate_pH(int pH_mV)
  {
    double tempK = (double)getTemperature() + 273.15; //Passar a temperatura de graus Celsius para Kelvin
    double F = 9.6485309*pow(10,4); //Constante de Faraday
    double R = 8.314510; //Constante dos gases

    double pH_S = 7; //pH neutro
    
    double Es = 0.385;//Potencial elétrico do pH de referência, isto é do pH neutro
    //double Es = 0.440; 

    double Ex = (double)read_pH()/1000;
    
    double pH_X = pH_S + ((Es-Ex)*F)/(R*tempK*log(10));

    return pH_X;
  }

  void activatePID()
  {
    pH_measured = calculate_pH(read_pH());

    if(pH_measured > 7.8) //pH elevado -> Bomba peristáltica 2 (Ácido)
    {
      boolean S0 = LOW;
      boolean S1 = HIGH;

      PID pH_PID(&pH_measured, &pH_delay, &pH_ref, Kp, Ki, Kd, DIRECT); //PID do pH
      pH_PID.SetMode(AUTOMATIC);
      pH_PID.Compute();

      writeDecoder(S0,S1,(int)pH_delay);
    }
    if(pH < 7.2) //pH baixo -> Bomba peristáltica 1 (Cloro)
    {
      boolean S0 = HIGH;
      boolean S1 = LOW;

      PID pH_PID(&pH_measured, &pH_delay, &pH_ref, Kp, Ki, Kd, DIRECT); //PID do pH
      pH_PID.SetMode(AUTOMATIC);
      pH_PID.Compute();

      writeDecoder(S0,S1,(int)pH_delay);
    }
    return;
  }

  
  /***************************************************************************************FIM DO pH***********************************************************************************************/
  
  
  /****************************************************************************************ORP***********************************************************************************************/

  int readORP() //Ler orp
  {
    int orp = AnalogSensorValue(LOW, LOW, HIGH);
    orp = orp + 118; //Desfazagem do multiplexer (alteração)
    Serial.print("ORP = ");
    Serial.print(orp);
    Serial.println(" mV");
    return orp;
  }

  int notifyORP(int orp) //notificar
  {
    if (orp < 650)
    {
      Serial.println("Low ORP value! Add chlorine!");
      boolean S0 = HIGH;
      boolean S1 = LOW;
      int d = (650-orp)*100;
      writeDecoder(S0,S1,d);
    }
    else
    {
      if (orp > 750)
      {
        Serial.println("High ORP value! Get out of the water");
      }
      else
      {
        Serial.println("ORP stable!");
      }
    }
    return orp;
  }
  /****************************************************************************************FIM DO ORP***********************************************************************************************/

  
  /****************************************************************************************CONXEXÃO À INTERNET***********************************************************************************************/
  void setupWiFi() {
    Serial.println();

    Serial.print("Connecting to "); Serial.print(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");

    Serial.print("WiFi connected, IP address: "); Serial.println(WiFi.localIP());

    wifiServer.begin();
  }
/***********************************************************************************FIM DA CONXEXÃO À INTERNET***********************************************************************************************/
  
/****************************************************************************************CLOUD***********************************************************************************************/
  /*Enviar dados para a cloud*/
  void sendDataToCloud(int security) {

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
    //int security = 0;   
    //security = checkDrowning(SecurityOn);
    int tempC = (int)getTemperature();
    int orp = readORP();
    boolean water0 = readWater0();
    boolean water1 = readWater1();
    int w_l = defineWaterLevel(water1, water0);
    boolean chlorine0 = readChlorine0();
    boolean chlorine1 = readChlorine1();
    int c_l = defineChlorineLevel(chlorine1, chlorine0);
    boolean acid0 = readAcid0();
    boolean acid1 = readAcid1();
    int a_l = defineAcidLevel(acid1, acid0);
    float pressure = checkFilterPressure();
    

    String payload = "{\"d\":{\"pH\":";
    payload += String(pH);
    payload += ",\"ORP\":";
    payload += String(orp);
    payload += ",\"temp\":";
    payload += String(tempC);
    payload += ",\"pressure\":";
    payload += String(pressure);
    payload += ",\"w_l\":";
    payload += String(w_l);
    payload += ",\"c_l\":";
    payload += String(c_l);
    payload += ",\"a_l\":";
    payload += String(a_l);
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

    client.disconnect();
    delay(10000);
  }
  /****************************************************************************************FIM DA CLOUD***********************************************************************************************/
int checkDrowning(boolean SecurityON)
  {
    int security = 0;
    if (SecurityON == true)
    { 
      float a1 = readAccel();
      delay(1000);
      float a2 = readAccel();
      delay(1000);
      float a3 = readAccel();
      delay(1000);
      float a4 = readAccel();
      delay(1000);
      float a5 = readAccel();
      delay(1000);
      float a6 = readAccel();
      delay(1000);
      float a7 = readAccel();
      delay(1000);
      float a8 = readAccel();
      delay(1000);
      float a9 = readAccel();
      delay(1000);
      float a10 = readAccel();
      delay(1000);
      
      if ((abs(a1 - a2) > 10) || (abs(a2 - a3) > 10) || (abs(a3 - a4) > 10) || (abs(a4 - a5) > 10) || (abs(a5 - a6) > 10) || (abs(a6 - a7) > 10) || (abs(a7 - a8) > 10) || (abs(a8 - a9) > 10) || (abs(a9 - a10) > 10))
      {
        security = 1;
        Serial.println("POSSIBLE DROWNING!!!!");
        sendDataToCloud(security);
      }
    }  
    return security;
    
  }
  /**************************************************************************SERVIDOR*********************************************************************************************************/
  void ReceiveAppData()
  {
    // Wait until the client sends some data
    //Serial.println(wifiClient);
    //Serial.println(wifiServer);
    
    /*Serial.println("new client");
    while (!wifiClient.available()) {
      delay(1);
    }*/
    
    //wifiServer.handleClient();
    
    //String req1 = wifiClient.readString();
    String req = wifiClient.readStringUntil('/');
    int id = 5;
    String String_msg = "";
    if(req == "GET ")
    {
      Serial.println("Receiceived Get Request! Sending data to application!");
      id =1;
    }
    if(req == "POST ")
    {
      Serial.println("Receiceived Post Request! Read ID");
      String String_id = wifiClient.readStringUntil(' ');      
      String trash = wifiClient.readStringUntil('&');
      String_msg = wifiClient.readStringUntil('=');

      Serial.print("Data = ");
      Serial.print(String_msg);
      Serial.println("!");      
      id = String_id.toInt();

    
    }
    Serial.print("ID = ");
    Serial.println(id);

    String s = "";
    switch (id)
    {
      case 0:
        {
          int tempC = (int)getTemperature();
          Serial.print("Temperature set will be changed to ");
          Serial.println(String_msg);
          int temperatureSet = String_msg.toInt();
          Serial.print("Temperature set to");
          Serial.println(temperatureSet);
          temp_threshold = setTemperature(temperatureSet, tempC);
          s = "Temperature set to ";
          s += String(temp_threshold);
          
          Serial.println(s);
          break;
        }
      case 1:
        {
          Serial.println("Get Request Received");
          
          float tempC = getTemperature();
          int orp = readORP();
          boolean water0 = readWater0();
          boolean water1 = readWater1();
          int w_l = defineWaterLevel(water1, water0);
          boolean chlorine0 = readChlorine0();
          boolean chlorine1 = readChlorine1();
          int c_l = defineChlorineLevel(chlorine1, chlorine0);
          boolean acid0 = readAcid0();
          boolean acid1 = readAcid1();
          int a_l = defineAcidLevel(acid1, acid0);
          Serial.print("a_l = ");
          Serial.println(a_l);
          float pressure = checkFilterPressure();
          s = "Temperature: " + String(tempC) + ", ";
          s += "pH: " + String(pH) + ", ";
          s += "ORP: " + String(orp) + ", ";
          s += "Pressure: " + String(pressure) + ", ";
          s += "Water_level: " + String(w_l) + ", ";
          s += "Chlorine_level: " + String(c_l) + ", ";
          s += "Acid_level: " + String(a_l);

          Serial.println("Get response -> " + s);
          break;
        }

      case 2:
        {
          if (String_msg == "SecurityOn")
          {
            SecurityOn = true;
            s = "Security mode activated";
            Serial.println(s);
          }
          else
          {
            if (String_msg == "SecurityOff")
            {
              SecurityOn = false;
              s = "Security mode deactivated";
              Serial.println(s);
            }
          }
          break;
        }
      case 3:
        {
          if (String_msg == "PumpOff")
          {
            PumpOff = true;
            writeDecoder(HIGH, HIGH, 2000); //Desligar a bomba
            s = "Maintenance Suspended";
            Serial.println(s);
          }
          else
          {
            if (String_msg == "PumpOn")
            {
              PumpOff = false;
              writeDecoder(HIGH, HIGH, 2000); //Ligar a bomba
              s = "Maintenance Restarted";
              Serial.println(s);
            }
          }
          break;
        }
      default: {
          break;
        }
    }

    wifiClient.flush();

    String httpHeader = "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=UTF-8\r\n";
    httpHeader += "Content-Length: ";
    httpHeader += s.length();
    httpHeader += "\r\n";
    httpHeader += "Connection: close\r\n\r\n";
    String httpResponse = httpHeader  + s + " ";

    wifiClient.print(httpResponse);
    // Send the response to the client

    // delay(1);

    id = 5;
    Serial.println("Client disonnected");

  }
  /****************************************************************************************FIM DO SERVIDOR***********************************************************************************************/

  /****************************************************************************************SETUP***********************************************************************************************/

  void setup() {
    // put your setup code here, to run once:

    
    
    pinMode(1, OUTPUT);
    Serial.println("Pin mode 1");    
    pinMode(3, OUTPUT);
    Serial.println("Pin mode 3");
    pinMode(15, OUTPUT);
    Serial.println("Pin mode 15");
    
    pinMode(14, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);
    pinMode(4, INPUT);

        
   Serial.begin(115200); 
    
    while(!Serial)
    {
      delay(1000);
    }
    setupWiFi(); //Iniciar a comunicação à internet
    Serial.println("Cloud connected!");
    
    delay(5000); //Dar tempo para o ESP8266 se estabelecer (problemas de hardware obrigam a retirar o ESP8266 da breadboard)
    
    Wire.begin(5); //Sensor de temperatura (DATA)
    setupTemperature(); //Iniciar o sensor de temperatura

    delay(1000);
    
    //Wire.begin(0, 2); //SCL -> 0; SDA -> 2
    setupAccelerometer(); //Iniciar o acelerómetro

    
    
    //14 -> S0
    //12 -> S1
    //13 -> S2
    //4 -> Output Multiplexer Digital

    //3 -> Descodificador_S0
    //1 -> Descodificador_S1
    //15 -> Activação/Desactivação do descodificador
  

    
    //delay(1000);
    
    usrInit();
    Serial.println("Timer starts!");
    
    
  }
  /****************************************************************************************FIM DO SETUP***********************************************************************************************/

  /****************************************************************************************LOOP***********************************************************************************************/
  void loop() {
    
    if (_timeout) //Se disparou o alarme ativar o PID, a função ORP e enviar os dados para a cloud (40 minutos de espera)
    {
      //----------------------EXECUTAR A MANUTENÇÃO----------------------------------------- 
      if (MaintenanceOn) //Se estiver em condições de realizar a manutenção
      {
        //*****************Activar o controlo de pH********************************
        Serial.println("Activate PID");
        double pH = read_pH();
        writeDecoder(LOW,LOW,500); //Ligar a bomba
        //*************************************************************************
        
        //*****************Activar o controlo de ORP*******************************
        Serial.println("Activate ORP");
        int orp = readORP();
        notifyORP(orp);
        //*************************************************************************

        
        //*****************Enviar dados para a cloud*******************************
        Serial. println("Send data to cloud!");
        sendDataToCloud(security);
        _timeout = false;
      //*************************************************************************
      }
      else
      {
        if(!(PumpOff))
        {
          writeDecoder(LOW,LOW,500); //Desligar a bomba
        }
      }
      //-------------------------------------------------------------------------------------
    }  
  
    checkDrowning(SecurityOn); //Verificar se existe um possível afogamento

  //----------------------EXECUTAR O CONTROLO DE TEMPERATURA-----------------------------
    int tempC = (int)getTemperature();
    Serial.print("Temperature is ");
    Serial.println(tempC);
    Serial.println("Temperature control");
    temp_threshold = setTemperature(temp_threshold, tempC); //controlar a temperatura
  //-------------------------------------------------------------------------------------

  //----------------------EXECUTAR O CONTROLO DE PRESSÃO---------------------------------
    float FilterPressure = checkFilterPressure(); //Verificar pressão
    if ((FilterPressure > 1.25)&& (pressureOk == true))
    {
      Serial.println("Pressure High!");
      sendDataToCloud(security);
      pressureOk = false;
    }
    else
    {
      pressureOk = true;
    }
  //-------------------------------------------------------------------------------------

  //----------------------VERIFICAR O NÍVEL DE ÁGUA--------------------------------------
    boolean water0 = readWater0();
    Serial.print("water0 = ");
    Serial.println(water0);
    boolean water1 = readWater1();
    Serial.print("water1 = ");
    Serial.println(water1);
    int w_l = defineWaterLevel(water1, water0); //Verificar o nível de água
    if(w_l2 != w_l)
    {
      w_l2 = w_l;
      //if(wifiClient)
      //{
        Serial.println("Send data to user about water level");
        sendDataToCloud(security);
        if(w_l == 0 || w_l == 3)
        {
          waterLevelOK = false;
        }
        else
        {
          waterLevelOK = true;
        }
      //}
    }
    //-------------------------------------------------------------------------------------
    
    checkDrowning(SecurityOn); //Verificar se existe um possível afogamento


    //----------------------VERIFICAR O NÍVEL DE CLORO-------------------------------------
    boolean chlorine0 = readChlorine0();
    Serial.print("chlorine0 = ");
    Serial.println(chlorine0);
    boolean chlorine1 = readChlorine1();
    Serial.print("chlorine1 = ");
    Serial.println(chlorine1);

    int c_l = defineChlorineLevel(chlorine1, chlorine0); //Verificar o nível do depósito de cloro
    if(c_l2 != c_l)
    {
      c_l2 = c_l;
      //if(wifiClient)
      //{
        Serial.println("Send data to user about chlorine level");
        sendDataToCloud(security);
        if(c_l == 1 || c_l == 2)
        {
          chlorineLevelOk = true;
        }
        else
        {
          chlorineLevelOk = false;
        }
      //}
    }
    //-------------------------------------------------------------------------------------

    checkDrowning(SecurityOn); //Verificar se existe um possível afogamento

   //----------------------VERIFICAR O NÍVEL DE ÁCIDO--------------------------------------
    boolean acid0 = readAcid0();
    Serial.print("acid0 = ");
    Serial.println(acid0);
    boolean acid1 = readAcid1();
    Serial.print("acid1 = ");
    Serial.println(acid1);
    Serial.print("Acid level: ");
    int a_l = defineAcidLevel(acid1, acid0);  //Verificar o nível do depósito de ácido
    if(a_l2 != a_l)
    {
      a_l2 = a_l;

        Serial.println("Send data to user about acid level");
        sendDataToCloud(security);
        if(a_l == 1 || a_l == 2)
        {
          acidLevelOK = true;
        }
        else
        {
          acidLevelOK = false;
        }
    }
  //-------------------------------------------------------------------------------------
  
  checkDrowning(SecurityOn); //Verificar se existe um possível afogamento
  
  //----------------------VERIFICAR SE A MANUTENÇÃO É POSSÍVEL--------------------------
  if(waterLevelOK && chlorineLevelOk && acidLevelOK && pressureOk && !(PumpOff))
  {
    Serial.println("Maintenance running");
    MaintenanceOn = true;
  }
  else
  {
    Serial.println("Suspending Maintenance!");
    MaintenanceOn = false;
    if(!(PumpOff))
    {
      PumpOff = true;
      Serial.println("Turning off pump!");
      writeDecoder(LOW,LOW,500);//Desligar a bomba!
    }
  }
  //-------------------------------------------------------------------------------------
   
   
  //----------------------VERIFICAR SE HOUVE PEDIDO DA APLICAÇÃO------------------------- 
   delay(500);
    wifiClient = wifiServer.available();
    if(wifiClient)
    {
      Serial.println("WifiClient exists!");
      ReceiveAppData();
    }
  //-------------------------------------------------------------------------------------
  }
}

/*NOTA 1: ESTE SISTEMA ESTÁ MAIORITÁRIAMENTE DEPENDENTE DO HARDWARE. ISTO SIGNIFICA QUE *
 *SE HOUVER UMA FALHA NO HARDWARE O SISTEMA IRÁ FUNCIONAR MAL!***************************
 *--------------------------------------------------------------------------------------*
 *NOTA 2: POR PRE-DEFINIÇÃO ESTE SISTEMA ENCONTRA-SE LIGADO À REDE RICS_LAB. ISTO *******
 *SIGNIFICA QUE SE SE LIGAR A OUTRA REDE, É NECESSÁRIO ALTERAR AS CREDENCIAIS E A *******
 *PALAVRA-PASSE (ssid e password respectivamente).***************************************
 *--------------------------------------------------------------------------------------*
 *NOTA 3: ESTE SISTEMA, POR INCAPACIDADE DO PROCESSADOR ESP8266-12E, NÃO POSSUI *********
 *PROCESSAMENTO EM RTOS (TEMPO REAL) PELO QUE A DETECÇÃO DE AFOGAMENTOS É FEITA *********
 *CONSTANTEMENTE (POLLING).**************************************************************
 *--------------------------------------------------------------------------------------*
 *NOTA 4: SE A REDE FOR ALTERADA, O ENDEREÇO DE IP DO SOC TAMBÉM SE ALTERA, PELO QUE É **
 *NECESSÁRIO CONFIGURAR NA APLICAÇÃO O ENDEREÇO DE IP POR FORMA A COMUNICAR DIRECTAMENTE*
 *COM O SISTEMA.*************************************************************************
 *--------------------------------------------------------------------------------------*
 *NOTA 5: ESTE ESTE SISTEMA COMUNICA DIRECTAMENTE PARA A CLOUD IBM BUEMIX USANDO AS *****
 *CREDÊNCIAIS: ORG; DEVICE_TYPE; DEVICE_ID; authMethod; clientId; token E server. É, ****
 *PORTANTO NECESSÁRIO CRIAR UMA CONTA NA CLOUD IBM BLUEMIX, REGISTAR O DISPOSITIVO, *****
 *GERAR UMA TOKEN E COMUNICAR COM ESSE SERVIDOR******************************************/
