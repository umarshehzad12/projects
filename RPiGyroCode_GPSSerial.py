#!/usr/bin/python

import smbus
import math
import datetime
import serial

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

text_file = open("gyro_data_gps.txt", 'w+')
ser = serial.Serial('/dev/ttyACM0',9600)
gps = []
s = []

i=0

while i<1000:
        #GPS read from serial code
        date_=str(datetime.datetime.now().time())
        gps = ser.readline()
        print gps
                #print read_serial
        text_file.write(date_ + ", " + gps)

        #Gyro data read code
        
	def read_byte(adr):
	    return bus.read_byte_data(address, adr)

	def read_word(adr):
	    high = bus.read_byte_data(address, adr)
	    low = bus.read_byte_data(address, adr+1)
	    val = (high << 8) + low
	    return val

	def read_word_2c(adr):
	    val = read_word(adr)
	    if (val >= 0x8000):
        	return -((65535 - val) + 1)
	    else:
        	return val

	def dist(a,b):
	    return math.sqrt((a*a)+(b*b))

	def get_y_rotation(x,y,z):
	    radians = math.atan2(x, dist(y,z))
	    return -math.degrees(radians)

	def get_x_rotation(x,y,z):
	    radians = math.atan2(y, dist(x,z))
	    return math.degrees(radians)

	bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
	address = 0x68       # This is the address value read via the i2cdetect command

# Now wake the 6050 up as it starts in sleep mode
	bus.write_byte_data(address, power_mgmt_1, 0)

	print "gyro data"
	print "---------"

	gyro_xout = read_word_2c(0x43)
	gyro_yout = read_word_2c(0x45)
	gyro_zout = read_word_2c(0x47)

# 	 "gyro_xout: ", gyro_xout, " scaled: ", (gyro_xout / 131), "gyro_yout: ", gyro_yout, " scaled: ", (gyro_yout / 131), "gyro_zout: ", gyro_zout, " scaled: ", (gyro_zout / 131)
	gyroString=  str(gyro_xout) + ", " + str(gyro_xout / 131) + ", " + str(gyro_yout) + ", " + str(gyro_yout / 131) + ", " + str(gyro_zout) + ", " + str(gyro_zout / 131)
	date_=str(datetime.datetime.now().time())
	#gyroString= "gyro_xout: " + str(gyro_xout) + ", scaled: " + str(gyro_xout / 131), ", gyro_yout: " + str(gyro_yout) + ", scaled: " + str(gyro_yout / 131) + ", gyro_zout: " + str(gyro_zout) + ", scaled: " + str(gyro_zout / 131)
        

	s= gyroString
	#s=s[2:-2]
	text_file.write(", " + s + ", " + date_ + "\n")
	print s
	print
	print "accelerometer data"
	print "------------------"

	accel_xout = read_word_2c(0x3b)
	accel_yout = read_word_2c(0x3d)
	accel_zout = read_word_2c(0x3f)

	accel_xout_scaled = accel_xout / 16384.0
	accel_yout_scaled = accel_yout / 16384.0
	accel_zout_scaled = accel_zout / 16384.0
	i=i+1
# 	print "accel_xout: ", accel_xout, " scaled: ", accel_xout_scaled, "accel_yout: ", accel_yout, " scaled: ", accel_yout_scaled, "accel_zout: ", accel_zout, " scaled: ", accel_zout_scaled, "x rotation: " , get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled), "y rotation: " , get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
text_file.flush()
text_file.close()

