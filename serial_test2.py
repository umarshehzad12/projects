import serial

s = []
text_file=open("gyro_data1.txt", 'w')
i=0
while True:
        ser = serial.Serial('/dev/ttyACM0',9600)
        if i%2:
                #if ser.in_waiting():
                #read_serial=ser.readline()
                s = ser.readline()
     
                print s
        
                #print read_serial
                text_file.write(s)
                i=i+1

        ser2= serial.Serial('/dev/ttyACM0',9600)

        if (i%2==0):
                s = ser2.readline()
                print s
                text_file.write(s)
                i=i+1

#text_file.flush()
#text_file.close

