    #include <SoftwareSerial.h>
    #include <TinyGPS.h>
     
    //long lat,lon; // create variable for latitude and longitude object
     
    SoftwareSerial gpsSerial(10, 11); // create gps sensor connection
    TinyGPS gps; // create gps object
    long lat = 1000000, lon = 1000000;
    unsigned long fix_age, time_, date, speed_, course_;
    unsigned long chars;
    unsigned short sentences, failed_checksum;
    
    void setup(){
      Serial.begin(9600); // connect serial
      gpsSerial.begin(9600); // connect gps sensor
    }
     
    void loop(){
      while(gpsSerial.available()){ // check for gps data
       if(gps.encode(gpsSerial.read())){ // encode gps data
        gps.get_position(&lat,&lon); // get latitude and longitude
        // display position
        // time in hhmmsscc, date in ddmmyy
      gps.get_datetime(&date, &time_, &fix_age);
 
      // returns speed in 100ths of a knot
      speed_ = gps.speed();
 
      // course in 100ths of a degree
      course_ = gps.course();
        Serial.print("Position: ");
        Serial.print("lat: ");Serial.print(lat);Serial.print(" ");// print latitude
        Serial.print("lon: ");Serial.print(lon); Serial.print(" "); //print longitude
        
        Serial.print("fix_age: ");Serial.print(fix_age);Serial.print(" ");// print fix_age
        Serial.print("time: ");Serial.print(time_); Serial.print(" "); // print time

        Serial.print("date: ");Serial.print(date);Serial.print(" ");// print date
        Serial.print("speed: ");Serial.print(speed_); Serial.print(" "); // print speed in knots

        Serial.print("course: ");Serial.print(course_); Serial.println(" "); // print course
        
       }
      }
    }
