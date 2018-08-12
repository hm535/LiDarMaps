import thread, time, sys, traceback, serial
from numpy import *

f = open("/users/USER/Documents/CalUnmanned/LiDAR/lidardata.txt", "wb")

comport = 'COM12'
baudrate = 115200

index = 0
init_level = 0

ser = serial.Serial(comport, baudrate)


def update_readings (angle, data):       # Function to update file with good data
    
    # (1) Unpack the raw data
    
    x0 = data[0]
    x1 = data[1]
    x2 = data[2]
    x3 = data[3]
    
    # (2) Process the data
    angle_rad = angle *pi/180.0
    dist_mm = x0 | ((x1 & 0x3f) << 8)    # Distance is encoded in the 13th or 14th bit
    quality = x2 | (x3 << 8)             # Quality is in the 16th bit
    
    # Export good data (regardless of quality)
    if not x1 & 0x80:                    # These flags are raised if the data is bad
        f.write('%d' % angle)
        f.write (': ')
        f.write('%d' % dist_mm)
        f.write('\r\n')

def computeChecksum (data):              # Computes and returns the checksum as an int
    
    # group the data by word (little-endian)
    data_list = []
    for t in range (10):
        data_list.append(data[2*t] + (data[2*t +1] <<8))
    
    # compute the checksum in 32 bits
    chk = 0
    for i in data_list:
        chk = (chk << 1) + i
    
    # return a value wraped and truncated to 15 bits
    checksum = (chk & 0x7FFF) + (chk >> 15)   # wrap around 15 bits
    checksum = checksum & 0x7FFF              # truncate to 15 bits
    return int(checksum)

def readLidar():
    global init_level, angle, index
    nb_errors = 0                     
    while True:
        time.sleep(0.00001)           # limits use of processor power
        if init_level == 0:
            b = ord(ser.read(1))      # indicates start byte
            if b == 0xFA:
                init_level = 1
            else:
                init_level = 0
            
        elif init_level == 1:         # indicates position index
            b = ord(ser.read(1))
            if b >= 0xA0 and b <= 0xF9:
                index = b - 0xA0
                init_level = 2
            elif b != 0xFA:
                init_level = 0
        
        elif init_level == 2 :        # indicates speed and data
            # for speed:
            b_speed = [ ord(b) for b in ser.read(2)]
            
            # for data:
            b_data0 = [ ord(b) for b in ser.read(4)]
            b_data1 = [ ord(b) for b in ser.read(4)]
            b_data2 = [ ord(b) for b in ser.read(4)]
            b_data3 = [ ord(b) for b in ser.read(4)]
            
            # collate all the data
            allData = [ 0xFA, index+0xA0 ] + b_speed + b_data0 + b_data1 + b_data2 + b_data3
            
            # checksum
            b_checksum = [ord(b) for b in ser.read(2)]
            incoming_checksum = int(b_checksum[0]) + (int(b_checksum[1]) << 8)
            
            # verify that the received checksum is correct
            # if correct, print the data
            if computeChecksum(allData) == incoming_checksum:                 
                update_readings(index * 4 + 0, b_data0)
                update_readings(index * 4 + 1, b_data1)
                update_readings(index * 4 + 2, b_data2)
                update_readings(index * 4 + 3, b_data3)
                    
            # if the checksums are not equal, ignore data. Increase error count
            else: 
                nb_errors +=1    
                #print ("error in checksum muhahaha")
                
            init_level = 0            # reset and wait for next packet
        else: 
            init_level = 0            # default
                


th = thread.start_new_thread(readLidar, ())

ser.close()
f.close()

