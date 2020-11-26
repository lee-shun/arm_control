#!/usr/bin/env python
from __future__ import print_function

import rospy
from ur5_control.srv import ServiceInt,ServiceIntRequest,ServiceIntResponse

import serial
import time
import struct


class gripper:
    def __init__(self):	
        self.ser = serial.Serial("/dev/ttyACM0")
        #for i in range(20):
            #try: 
                #self.ser = serial.Serial("/dev/ttyACM" + str(i))
            #except:
                #continue
        self.ser.baudrate = 9600
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.timeout = 2.5
        self.ser.reset_input_buffer()
        self.msg = struct.pack('!b',1)
        self.msg2 = struct.pack('!b',2)
        rospy.init_node('gripper',anonymous=True)
        self.Service = rospy.Service('ur5_control/gripper/command',ServiceInt,self.svc_callback_gripper)
        rospy.spin()

    
    def svc_callback_gripper(self,req):
        res = ServiceIntResponse()
        res.Response = 0
        if req.Data == 0:
            self.release()
        elif req.Data == 1:
            self.grasp()
        elif req.Data == 2:
            res.Response = self.read()
        return res
        



    def grasp(self):
        self.ser.write('0x31')
	
    def release(self):
        self.ser.write('0x32')
        
    def read(self):
        response = self.ser.read(2)
        if response==None:
            response=0
        #response = hexShow(response)
        return int(response)
        

def hexShow(argv):  
    result = ''  
    hLen = len(argv)  
    for i in xrange(hLen):  
        hvol = ord(argv[i])  
        hhex = '%02x'%hvol  
        result += hhex+' '  
    return result
	


if __name__=='__main__':
    a=gripper()
    #a.release()
    time.sleep(1)
    a.grasp()
    time.sleep(1)
    b=a.read()
    print(b)
    #time.sleep(1)
    a.release()
