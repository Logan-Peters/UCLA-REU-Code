import time
import IMU
import sys
import math
import datetime
import os
import thread
import socket
handInView = False
legoPlaced = False
sum = 0
def receieveHandInView():
    Host = '0.0.0.0'
    global sum
    timerWhileHandInFrame = 0
    global handInView
    global legoPlaced
    Port = 65432
    #print("...") 
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((Host, Port))
    while True:
        s.listen(5)
        conn, addr = s.accept()
    
        print('connected by', addr)
        while True:
            data = conn.recv(1024)
            if data == 'yes':
                handInView = True
            if data == 'no':
                handInView = False
            if(handInView and timerWhileHandInFrame == 0):
                timerWhileHandInFrame = int(round(time.time()))
            else:
                if(sum != 0 and 0 != (int(round(time.time())) - timerWhileHandInFrame)):
                    if(sum/(int(round(time.time())) - timerWhileHandInFrame) > 30000):
                         legoPlaced = True
                    print(sum/(int(round(time.time())) - timerWhileHandInFrame))
                timerWhileHandInFrame = 0
                sum = 0
            if(legoPlaced):
                conn.sendall("placed")
                print("placed")
                legoPlaced = False
            else:
                conn.sendall("not placed")
            if not data:
                break



RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070  	# [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA =  0.40 		# Complementary filter constant
#detect accelerometer and gyroscope
IMU.detectIMU()
IMU.initIMU()
#opens file data.txt to append to it
filename = "data.txt"
file = open(filename, "a")
thread.start_new_thread(receieveHandInView,() )
rate_gyr_x = 0.0
rate_gyr_y = 0.0
rate_gyr_z = 0.0
gyroXangle = 0.0
gyroYangle = 0.0
gyroZangle = 0.0
a = datetime.datetime.now()
prevAccX = 0
prevAccY = 0
prevAccZ = 0
timer = 0
state = 0

while True:
	ACCx = IMU.readACCx()
	ACCy = IMU.readACCy()
	ACCz = IMU.readACCz()
	GYRx = IMU.readGYRx()
	GYRy = IMU.readGYRy()
	GYRz = IMU.readGYRz()

	#LP is the loop period which is how long between gyro reads
	b = datetime.datetime.now() - a
        a = datetime.datetime.now()
    	LP = b.microseconds/(1000000*1.0)
	#gets UNIX time
    	millis = int(round(time.time()*1000))

	#Convert Gyro raw to degrees per second
	rate_gyr_x =  GYRx * G_GAIN
	rate_gyr_y =  GYRy * G_GAIN
	rate_gyr_z =  GYRz * G_GAIN

	#Calculate the angles from the gyro.
	gyroXangle+=rate_gyr_x*LP
    	gyroYangle+=rate_gyr_y*LP
    	gyroZangle+=rate_gyr_z*LP

	#Convert acceleration from mg/LSB to m/s^2
	Ax = ACCx
	Ay = ACCy
	Az = ACCz
        
        if(handInView and abs(Ax - prevAccX) + abs(Ay - prevAccY) + abs(Az - prevAccZ) + sum > 800):
                sum =  abs(Ax - prevAccX) + abs(Ay - prevAccY) + abs(Az - prevAccZ) + sum

            
	#print writes to console reminder to delete later when things look good
	#file.write() writes to data.txt
        prevAccX = Ax
        prevAccY = Ay
        prevAccZ = Az

	file.write("Time Stamp %i  %3.3f  %3.3f  %3.3f  %3.3f  %3.3f  %3.3f  %3.3f  %3.3f  %3.3f \n" %(millis,gyroXangle,gyroYangle,gyroZangle,rate_gyr_x,rate_gyr_y,rate_gyr_z,Ax,Ay,Az))
	time.sleep(0.03)
file.close()
