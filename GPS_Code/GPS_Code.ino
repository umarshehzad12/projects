/*The open source pothole detection kit has been developed at Make-i-stan in collaboration with ICFJ Knight Fellowships*/

#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "Wire.h"
SoftwareSerial mySerial(10, 11); // RX, TX
TinyGPS gps;


void setup()  
{
  Wire.begin(8);     // Open serial communications and wait for port to open:
  Serial.begin(9600);     // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  delay(500);
  Wire.onRequest(requestEvent); // regi
}
  //3147636, 7434279

  long lat = 1000000, lon = 1000000;
void requestEvent() {
  String d = String(lat) + "," + String (lon);
  char buffer[16];
 d.toCharArray(buffer, 16);
  Wire.write(buffer); // respond with message of 6 bytes
   Serial.println(buffer);
  // as expected by master
}
int x = 0;
void loop() // run over and over
{
  if (mySerial.available()) 
  {
      char c = mySerial.read();
      if (gps.encode(c)) 
      {
          Serial.println("Acquired Data");
          Serial.println("-------------");
    
          gps.get_position(&lat, &lon);



          Serial.print("Lat/Long(10^-5 deg): "); Serial.print(lat); Serial.print(", "); Serial.print(lon); 
          //delay(500);
          Serial.println();
      }
    }
}
