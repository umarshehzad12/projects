import serial

ser = serial.Serial('/dev/ttyACM0',9600)
s = []
text_file=open("gyro_data1.txt", 'w')

while True:
        #if ser.in_waiting():
                #read_serial=ser.readline()
        s = ser.readline()
        print s
                #print read_serial
        text_file.write(s)
text_file.flush()
text_file.close()
ser.close()
