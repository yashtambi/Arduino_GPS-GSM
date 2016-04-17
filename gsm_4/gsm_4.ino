
//Connections: 
//GPS on Serial Port
//GSM on 

#include <stdlib.h>
#include <math.h>
#include <SoftwareSerial.h>

#define knotsToKmph 1.85184        //Knots to mps 
#define R           6371000        //Radius of Earth
#define toRad       3.1416/18000   //Decimal to Radian

#define delay_period  4000      //ms

#define ADC_gain_external       0.35
#define gain_inverse            1/ADC_gain_external
#define ADC_input_channel       A0
#define ADC_conversion_factor   0.00488   // 5/1024

#define URL             "api.thingspeak.com/update?key="
#define Write_API_Key   "H17K19370Z2V8F3X"

#define LED             A5

#define LED_ON          digitalWrite(LED, HIGH)
#define LED_OFF         digitalWrite(LED, LOW)

bool flag = false;

//Variables for GPS
int Gpsdata;                 // For incoming serial data
unsigned int finish  = 0;    // Indicate end of message
unsigned int pos_cnt = 0;    // Position counter
unsigned int lat_cnt = 0;    // Latitude data counter
unsigned int log_cnt = 0;    // Longitude data counter
unsigned int spd_cnt = 0;    // Speed data counter
unsigned int date_cnt = 0;   // Date data counter
unsigned int flg     = 0;    // GPS flag
unsigned int com_cnt = 0;    // Comma counter
unsigned int time_cnt = 0;   // Time Counter
char lat[20];                // Latitude array
char lg[20];                 // Longitude array 
char spd[20];                // Speed Array
char date[20];               // Date Array
char time[20];               // Time Array
char signal;                 // GPS Signal Information
char ew;                     // East/West
char ns;                     // North/South
double lat_i, lg_i;          // Store intitial latitude and longitude
double lat_c, lg_c;          // Store current latitude and longitude
double time_i;               // Store initial time
volatile unsigned long dist = 0;      // Distance convertor
String speed_mps = "";       // Store speed in meter/second
String distance = "";        // Distance Accumulator
volatile float spd_mps = 0;

String battery_voltage = "";

#define GSM_Tx    7
#define GSM_Rx    8

SoftwareSerial mySerial(GSM_Rx, GSM_Tx);


float GetBatteryVoltage (void)
{
  return ((analogRead(ADC_input_channel)) * ADC_conversion_factor * gain_inverse);
}


void GetGPS_Processed (void)
{
  unsigned int dist_c = 0;

  speed_mps = "";

  GetGPS_GPRMC();
  if (signal == 'A')
  {
    if (flag == false)
    {
      lat_i = atof(lat) * toRad;
      lg_i = atof(lg) * toRad;
      flag = true;
    }

    lat_c = atof(lat) * toRad;
    lg_c  = atof(lg) * toRad;

    //process distance and store
    //haversine formula
    volatile double dLat = (lat_c - lat_i);
    volatile double dLon = (lg_c - lg_i);
    volatile double a = sin(dLat / 2) * sin(dLat / 2) + sin(dLon / 2) * sin(dLon / 2) * cos(lat_i) * cos(lat_c);
    dist_c = R * 2 * atan2(sqrt(a), sqrt(1 - a));

    lat_i = lat_c;
    lg_i = lg_c;
  }

  //Calculate speed in Meter/second
  spd_mps = knotsToKmph * (atof(spd));
  speed_mps = String(spd_mps, 3);

  dist += dist_c;
  distance = String(dist);
}


//Function to parse incoming GPS Data
//Since we do not have UART Buffer :/
void GetGPS_GPRMC()
{
  while (finish == 0) {
    while (Serial.available() > 0) {     // Check GPS data
      Gpsdata = Serial.read();
      flg = 1;
      if ( Gpsdata == '$' && pos_cnt == 0) // finding GPRMC header
        pos_cnt = 1;
      if ( Gpsdata == 'G' && pos_cnt == 1)
        pos_cnt = 2;
      if ( Gpsdata == 'P' && pos_cnt == 2)
        pos_cnt = 3;
      if ( Gpsdata == 'R' && pos_cnt == 3)
        pos_cnt = 4;
      if ( Gpsdata == 'M' && pos_cnt == 4)
        pos_cnt = 5;
      if ( Gpsdata == 'C' && pos_cnt == 5 )
        pos_cnt = 6;
      if (pos_cnt == 6 &&  Gpsdata == ',') { // count commas in message
        com_cnt++;
        flg = 0;
      }
      //Time
      if (com_cnt == 1 && flg == 1) {
        time[time_cnt++] =  Gpsdata;
        flg = 0;
      }
      //GPS Signal
      if (com_cnt == 2 && flg == 1) {
        signal =  Gpsdata;
        flg = 0;
      }
      //Latitude
      if (com_cnt == 3 && flg == 1) {
        lat[lat_cnt++] =  Gpsdata;
        flg = 0;
      }
      //North/South
      if (com_cnt == 4 && flg == 1) {
        ns =  Gpsdata;
        flg = 0;
      }
      //Longitude
      if (com_cnt == 5 && flg == 1) {
        lg[log_cnt++] =  Gpsdata;
        flg = 0;
      }
      //East/West
      if (com_cnt == 6 && flg == 1) {
        ew =  Gpsdata;
        flg = 0;
      }
      //Speed (Knots)
      if (com_cnt == 7 && flg == 1) {
        spd[spd_cnt++] =  Gpsdata;
        flg = 0;
      }
      //Date (DDMMYY)
      if (com_cnt == 9 && flg == 1) {
        date[date_cnt++] =  Gpsdata;
        flg = 0;
      }
      //End of GPS_GPRMC Message
      if ( Gpsdata == '*' && com_cnt >= 9) {
        com_cnt = 0;
        lat_cnt = 0;
        log_cnt = 0;
        date_cnt = 0;
        spd_cnt = 0;
        time_cnt = 0;
        flg     = 0;
        finish  = 1;
      }
    }
  }
  finish = 0; pos_cnt = 0;
}


void GPS_init (void)
{
  Serial.begin(9600);
  Serial.write("$PMTK314, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 * 29");  //GPRMC Only
  Serial.write(0x0D);
  Serial.write(0x0A);
  delay(1000);
  Serial.write("$PMTK220, 200 * 2C");
  Serial.write(0x0D);
  Serial.write(0x0A);
  delay(1000);
  Serial.write("$PMTK397, 1.0 * 3C");                                    //Speed threshold 1mps
  Serial.write(0x0D);
  Serial.write(0x0A);
  delay(2000);
  Serial.begin(9600);
  Serial.print("$PMTK251, 115200 * 1F");                                 //Set GPS Baud @ 115200
  Serial.write(0x0D);
  Serial.write(0x0A);
  delay(1000);
  Serial.begin(115200);                                               //Set UART Baud @ 115200
  delay(1000);

  GetGPS_GPRMC();             //Get GPS Data
  GetGPS_GPRMC();             //Get GPS Data
  GetGPS_GPRMC();             //Get GPS Data

  lat_i = atof(lat) * toRad;  //Store initial coordinates
  lg_i = atof(lg) * toRad;
}


void GSM_init (void)
{
  mySerial.begin(9600);

  delay(10000); 
  mySerial.println("AT+CREG?");                                       //Check if sim registered (Debug only)
  delay(2000);

  mySerial.println("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");              //Set connection type as GPRS
  delay(2000);    

  mySerial.println("AT+SAPBR=3,1,\"APN\",\"airtelgprs.com\"");        //Set APN
  delay(2000);

  mySerial.println("AT+SAPBR=1,1");                                   //Open context
  delay(2000);

  mySerial.println("AT+HTTPINIT");                                    //Initialize HTTP
  delay(2000);
  
  mySerial.println("AT+HTTPPARA=\"CID\",1");
  delay(2000);
}


void GSM_update(void)
{
  String update_string = "";
  update_string += URL;
  update_string += Write_API_Key;
  update_string += "&field1=";
  update_string += battery_voltage;
  update_string += "&field2=";
  update_string += lat;
  update_string += "&field3=";
  update_string += lg;
  update_string += "&field4=";
  update_string += spd;

  mySerial.print("AT+HTTPPARA=\"URL\",");
  mySerial.println(update_string);
  delay(1000);

  mySerial.println("AT+HTTPACTION=0");
  delay(10000);
}


void GSM_GetRequest(void)
{
  mySerial.println("AT+HTTPACTION=0");
  delay(5000);
}


void setup() {
  GSM_init();
  GPS_init();
  pinMode(LED, OUTPUT);
}


void loop() {
  float volt = GetBatteryVoltage();

  battery_voltage = String(volt, 3);

  GSM_update();
  
  LED_ON;
  
  GetGPS_Processed();
  
  LED_OFF;
  
  delay(delay_period);
//  GSM_GetRequest();
}
