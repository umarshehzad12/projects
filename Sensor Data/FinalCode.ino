#include <DHT.h>

#include <SoftwareSerial.h>                                                                                       //Library for Serial Communication
#include "DHT.h"                                                                                                  //Library for Humidity and Temperature

#include <stdio.h>                                                                                                //Standard C library
#include <stdlib.h>                                                                                               //Standard C library
#include <string.h>                                                                                               //Library for String Manipulation
#include<DSM501.h>                                                                                                //Library for Dust Sensor
/*---------------------------------------*/

#define GPRS_Tx 8                                                                                                 //Serial Wires for Sim Connection
#define GPRS_Rx 7                                                                                                 //Serial Wires for Sim Connection

#define onModulePin 9                                                                                             //Needed for powering SIM900 module

#define DOUTpin 6                                                                                                 //Digital data for Gas Sensor
#define AOUTpin A0                                                                                                //Analog data for Gas Sensor
#define DSM501_PM10 3                                                                                             //Dust sensor wire with senstivity of 1um
#define DSM501_PM25 4                                                                                             //Dust sensor wire with senstivity of 2.5um

/*---------------------------------------*/
SoftwareSerial GPRS(GPRS_Rx, GPRS_Tx);                                                                            //Setting of SIM900 communication at pin 7 & 8
DSM501 dsm501(DSM501_PM10, DSM501_PM25);                                                                          //Setting of DSM501 communication at pin 3 & 4
DHT dht;                                                                                                          //Creating an instance of DHT Object
/*---------------------------------------*/

float humidity = 0.0, temperature = 0.0;                                                                          //Initializing variables for DHT11
int answer = 0;                                                                                                   //Variable needed in SIM900 communication
char response[100];                                                                                               //Array used in SIM900 communication
int prevTime = millis();                                                                                          //Algorithmic value for DSM501A


#define	uchar	unsigned char
#define	uint	unsigned int
/*---------------------------------------*/
void setup()
{                                                                                                                 //Setup starts
  dsm501.begin(MIN_WIN_SPAN);                                                                                     //Initializing DSM501A with minimum sampling rate
  dht.setup(2);
  pinMode(onModulePin, OUTPUT);                                                                                   //Setting SIM900 Module 'ON'
  delay(100);                                                                                                     //Necessory Delay
  GPRS.begin(9600);                                                                                               //Setting default baud rate at 9600 bits/sec
  Serial.begin(9600);                                                                                             //Setting default baud rate at 9600 bits/sec
  Serial.println("Starting...");
  powerOn();                                                                                                      //Powerup the SIM900 module
  while (sendATcommand("AT", "OK", 1000) == 0);                                                                   //Waiting until SIM900 returns OK in response of simple AT Command
  sendATcommand("", "Call Ready", 5000);                                                                          //Recieving Call Ready status from SIM900
  Serial.println("Done");
  delay(3000);
  setupGPRS();                                                                                                    //Sending necessory commands for SIM900 Internet connection ...
}                                                                                                                 //Setup ends

/*---------------------------------------*/

void loop ()
{                                                                                                                 //Loop Starts
  int value = readDustValue();                                                                                    //It automatically reads value and map it according to device design!
  readTH(&temperature, &humidity);                                                                                //It translates the value taken above to sensible (temp+humid) values
  if (millis()-prevTime >= 10000)                                                                                 //It's a sampling for accurate approximiated reading
  {
      sendTransaction(temperature, value, humidity, readMQ7());
      prevTime = millis();                                                                                        //Resetting the time to be noted again
  }
  delay(100);
  Serial.println("--------------------------------------");                                                       //Now start showing values of serial monitor
  Serial.print(temperature);                                                                                      //Showing temperature
  Serial.print('\t');
  Serial.print(humidity);                                                                                         //Showing humidity
  Serial.print('\t');
  Serial.print(value);                                                                                            //Showing humidity and temperature value
  Serial.print('\t');
  Serial.println(readMQ7());                                                                                      //Showing Gas Sensor value
  Serial.println("--------------------------------------");
}                                                                                                                 //Loop Ends

/*---------------------------------------*/
int readDustValue()
{
  dsm501.update();
  return(dsm501.getPM25());                                                                                       //Get dust value according to 2.5 um senstivity
}

/*---------------------------------------*/
void readTH(float *temp, float *hum)                                                                              //Function for humidity and temperature values
{
  delay(dht.getMinimumSamplingPeriod());                                                                          //Minimum possible sampling time for good approximiation
  *hum = dht.getHumidity();                                                                                       //Update the Humidity at provided address
  *temp = dht.getTemperature();                                                                                   //Update the Temperature at provided address
}

/*---------------------------------------*/
int readMQ7()                                                                                                     //Small Avg. Sampling Algorithm for an avg. value over a sample period
{
  int limit;                                                                                                      //Necessory variable involved in averaging values
  int value;                                                                                                      //Necessory variable involved in averaging values
  long valueSum = 0;                                                                                              //Necessory variable involved in averaging values
  int count = 0;                                                                                                  //Necessory variable involved in averaging values
  for(int i = 0; i<500; i++)                                                                                      //Looping 500 times
  {
    valueSum += analogRead(AOUTpin);                                                                              //Sampling Analog data
    limit = digitalRead(DOUTpin);                                                                                 //Sampling Digital data
    count = i;                                                                                                    //Updating latest count value
  }
  value = valueSum/count;                                                                                         //Taking Avg.
  return value;                                                                                                   //Return Value
}

/*---------------------------------------*/
void setupGPRS()
{
  Serial.println("setupGPRS.");
  sendATcommand("ATE0", "OK", 1000);                                                                              //Sending AT Command for saving current config.
  sendATcommand("AT+IPR=9600", "OK", 1000);                                                                       //Sending AT Command for setting fixed local 'baud rate'
  sendATcommand("AT+CMGF=1", "OK", 100);                                                                          //Sending AT Command for setting Text Mode for communication
  while (sendATcommand("AT+CSTT=\"zongwap\"", "OK", 1000) == 0 );                                                 //Sending AT Command for starting APN config.
  while (sendATcommand("AT+CIICR", "OK", 5000) == 0);                                                             //Sending AT Command for bringing working mode to GPRS 
  sendATcommand("AT+CIFSR", ".", 2000);                                                                           //Sending AT Command for getting local IP address
  sendATcommand("AT+CIFSR=?", "OK", 2000);                                                                        //Sending AT Command for getting local IP address with OK response 
  sendATcommand("AT+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\"", "OK", 2000);                                                //Sending AT Command for configuring DNS Server for decoding web address to IP
  sendATcommand("AT+CGATT=1", "OK", 5000);                                                                        //Sending AT Command for checking if module is connected to GPRS network or not?
  sendATcommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", "OK", 2000);                                                 //Sending AT Command for Bearer setting for IP based applications 
  sendATcommand("AT+SAPBR=3,1,\"APN\",\"zongwap\"", "OK", 2000);                                                  //Sending AT Command for setting APN (Access Point Name) for cellular network 
  sendATcommand("AT+SAPBR=1,1", "OK", 5000);                                                                      //Sending AT Command for opening bearer settings
}

/*---------------------------------------*/
void sendTransaction(float t, unsigned int d, unsigned int h, unsigned int mq)                                     //This function takes parameters and send it +recieve its response
{
  while (sendATcommand("AT+HTTPINIT", "OK", 2000) == 0);                                                           //Initializing the SIM900 http protocol connection
  httpSetParameter(t, d, h, mq);
  sendATcommand("AT+HTTPACTION=0", "+HTTPACTION:0,200,", 30000);                                                   //Setting GET method as sending data
  Serial.print("Response:" );                                                                                      //monitoring on Serial Monitor
  Serial.println(response);                                                                                        //monitoring on Serial Monitor
  delay(1000);
  sendATcommand("AT+HTTPREAD=0,100", "OK", 5000);                                                                  //Recieving its Response!
  Serial.print("Response:" );                                                                                      //Printing Response
  Serial.println(response);                                                                                        //Printing Response
  while (sendATcommand("AT+HTTPTERM", "OK", 1000) == 0);                                                           //Terminating the SIM900 http protocol connection
}

/*---------------------------------------*/
void httpSetParameter(float t, unsigned int d, unsigned int h, unsigned int mq)                                   //setting parameters in cURL format!
{
  char t_[10];
  dtostrf( t, 4, 2, t_ );
  //http://damp-crag-29984.herokuapp.com/server?t=14.56&d=99.26&h=123456&mq=22
  //http://immunization.ipal.itu.edu.pk/index.php/ApiEngine/IPushdata?rfid=4536822837&mcode=255645435345&amount=223&rate=5.5*/
  const char *URL = "damp-crag-29984.herokuapp.com/server?";
  char command[200];
  sprintf(command, "AT+HTTPPARA=\"URL\",\"%st=%s&d=%i&h=%i&mq=%i\"", URL, t_, d, h, mq);
  sendATcommand(command, "OK", 1000);
}

/*---------------------------------------*/
void powerOn()                                                                                                      
{
  digitalWrite(onModulePin, LOW);                                                                                 //Turning it OFF and then ON and OFF again power ups the module 
  delay(1000);                                                                                                    //Turning it OFF and then ON and OFF again power ups the module 
  digitalWrite(onModulePin, HIGH);                                                                                //Turning it OFF and then ON and OFF again power ups the module 
  delay(2000);                                                                                                    //Turning it OFF and then ON and OFF again power ups the module 
  digitalWrite(onModulePin, LOW);                                                                                 //Turning it OFF and then ON and OFF again power ups the module 
  delay(4000);                                                                                                    //Turning it OFF and then ON and OFF again power ups the module 
}

/*---------------------------------------*/
void powerDown()
{
  sendATcommand("AT+CPOWD=1", "NORMAL POWER DOWN", 5000);                                                         //We can shutdown the SIM900 via our own AT Command
}

/*---------------------------------------*/
int sendATcommand(char* ATcommand, char* expected_answer1, unsigned int timeout)                                  //Function to send AT command to SIM900 by setHTTParameters function
{
  GPRS.begin(9600);
  Serial.println(ATcommand);
  unsigned int x = 0;
  answer = 0;
  unsigned long previous;
  memset(response, '\0', 100);                                                                                    // Initialize the string
  delay(100);
  while ( GPRS.available() > 0) GPRS.read();                                                                      // Clean the input buffer
  GPRS.println(ATcommand);                                                                                        // Send the AT command
  previous = millis();
  do {
    if (GPRS.available() != 0) {
      while (GPRS.available())
      {
        response[x] = GPRS.read();
        x++;
      }
    }
    if (strstr(response, expected_answer1) != NULL)
    {
      answer = 1;
    }
    else
      answer = 0;
  }
  while ((answer == 0) && ((millis() - previous) < timeout));
  while (GPRS.available())
  {
    response[x] = GPRS.read();
    x++;
  }
  if ((millis() - previous) > timeout)
    Serial.println("TimeOut.");
  Serial.println(response);
  return answer;
}
