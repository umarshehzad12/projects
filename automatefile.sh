#!/home/pi python
#chmod +x RPiGyroCode2.py
sleep 10 #10 Sec Delay
cd /home/pi
sudo python RPiGyroCode2.py&

sudo python serial_test.py&
avconv -f video4linux2 -i /dev/video0 video.avi
