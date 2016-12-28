    #include <SoftwareSerial.h>
    #include <TinyGPS.h>
     
    //long lat,lon; // create variable for latitude and longitude object
     
    SoftwareSerial gpsSerial(10, 11); // create gps sensor connection
    TinyGPS gps; // create gps object
    long lat = 1000000, lon = 1000000;
    
    void setup(){
      Serial.begin(9600); // connect serial
      gpsSerial.begin(9600); // connect gps sensor
    }
     
    void loop(){
      while(gpsSerial.available()){ // check for gps data
       if(gps.encode(gpsSerial.read())){ // encode gps data
        gps.get_position(&lat,&lon); // get latitude and longitude
        // display position
        Serial.print("Position: ");
        Serial.print("lat: ");Serial.print(lat);Serial.print(" ");// print latitude
        Serial.print("lon: ");Serial.println(lon); // print longitude
       }
      }
    }
