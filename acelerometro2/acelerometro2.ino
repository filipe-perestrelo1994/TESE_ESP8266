/* ADXL345
 * i2c bus SDA = GPIO0; SCL = GPIO2
 *
 * Just a little test message.  Go to http://192.168.4.1 in a web browser
 * connected to this access point to see it.
 * USER: admin PASSWOR: admin
 */  

#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
//ADXL345
#include <Wire.h>

/* Set these to your desired credentials. */
//const char *ssid = "ESPap";
//const char *password = "12345678";
ESP8266WebServer server(80);



#define DEBUG true
#define Serial if(DEBUG)Serial

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

float max_x=0;
float min_x=0;
float cal_x=0;
float x = 0;

float max_y=0;
float min_y=0;
float cal_y=0;
float y = 0;

float max_z=0;
float min_z=0;
float cal_z=0;
float z = 0;



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
  //x = x + cal_x;

  
  //Serial.print("x: "); 
  //Serial.print( x*2./512 );
  //Serial.print(" y: ");
  //Serial.print( y*2./512 );
  //Serial.print(" z: ");
  //Serial.print( z*2./512 );
  //Serial.print("Z: "); Serial.print( z);

  //Serial.println( sqrtf(x*x+y*y+z*z)*2./512 );

//getX() = read16(ADXL345_REG_DATAX0);
//x = getX() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  
}

void writeTo(byte address, byte val) 
{
  Wire.beginTransmission(DEVICE); // start transmission to device
  Wire.write(address); // send register address
  Wire.write(val); // send value to write
  Wire.endTransmission(); // end transmission
}

// Reads num bytes starting from address register on device in to _buff array
void readFrom(byte address, int num, byte _buff[]) 
{
  Wire.beginTransmission(DEVICE); // start transmission to device
  Wire.write(address); // sends address to read from
  Wire.endTransmission(); // end transmission
  Wire.beginTransmission(DEVICE); // start transmission to device
  Wire.requestFrom(DEVICE, num); // request 6 bytes from device

  int i = 0;
  while(Wire.available()) // device may send less than requested (abnormal)
  {
    _buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); // end transmission
}

void setup() {
  delay(1000);

  Serial.begin(115200);
  Serial.println();



//ADXL345
  // i2c bus SDA = GPIO0; SCL = GPIO2
  delay(5000);
  Wire.begin(0,2);      
  
  // Put the ADXL345 into +/- 2G range by writing the value 0x01 to the DATA_FORMAT register.
  // FYI: 0x00 = 2G, 0x01 = 4G, 0x02 = 8G, 0x03 = 16G
  writeTo(DATA_FORMAT, 0x00);
  
  // Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeTo(POWER_CTL, 0x08);

  int i =0;
  for(i=0; i<11; i++)
  {
    //uint8_t howManyBytesToRead = 6;
    //readFrom( DATAX0, howManyBytesToRead, _buff);
    float calib_z ;//= (((short)_buff[1]) << 8) | _buff[0];
    calib_z = readAccel();
    if(i==0)
     cal_z = z;
    if(i>0)
     cal_z = cal_z + calib_z;
    Serial.println(calib_z);
    delay(100);
  }

  cal_z = cal_z/10;
  Serial.print("cal_z: ");Serial.println(cal_z); 
  
}

void loop() {
  //server.handleClient();
  current_value_z = readAccel();  // read ONLY x, for the y and x modify the readAccel function
 /* 
  if((current_value_x - cal_x) > max_x)
    max_z = current_value_x - cal_z;
  if((current_value_x - cal_x) < min_x)
    min_z = current_value_x - cal_x;

  
  if((current_value_y - cal_y) > max_y)
    max_y = current_value_y - cal_y;
  if((current_value_y - cal_y) < min_y)
    min_y = current_value_y - cal_y;

    
  if((current_value_z - cal_z) > max_z)
    max_z = current_value_z - cal_z;
  if((current_value_z - cal_z) < min_z)
    min_z = current_value_z - cal_z;
*/
  //Serial.print("x: ");Serial.print(current_value_x);  Serial.print(" x(corrected): ");Serial.print(current_value_x - cal_x);    
  //Serial.print(" Min:" );Serial.print(min_x); Serial.print(" Max:" ); Serial.println(max_x);

  //Serial.print("y: ");Serial.print(current_value_y);  Serial.print(" y(corrected): ");Serial.print(current_value_y - cal_y);    
  //Serial.print(" Min:" );Serial.print(min_y); Serial.print(" Max:" ); Serial.println(max_y);

  Serial.print("z: ");Serial.println(current_value_z);  //Serial.print(" z(corrected): ");Serial.print(current_value_z - cal_z);    
  //Serial.print(" Min:" );Serial.print(min_z); Serial.print(" Max:" ); Serial.println(max_z);  
    
  //Serial.print("Trigger value:"); Serial.print(trigger_value); Serial.print(" Count:"); Serial.println(trigger_count);
  delay(100);   // only read every 100ms

}

