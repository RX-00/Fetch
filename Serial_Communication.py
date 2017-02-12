#!/usr/bin/env python

import sys
import serial
import rospy
import time
import syslog
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray

port = '/dev/ttyACM0'

arduino = serial.Serial(port, 9600, timeout = 1)

def sendData(base, shoulder, elbow, claw):
    arduino.flush()
    baseData = base
    shoulderData = shoulder
    elbowData = elbow
    clawData = claw
    baseDataB = str(baseData) + 'b'
    shoulderDataS = str(shoulderData) + 's'
    elbowDataE = str(elbowData) + 'e'
    clawDataC = str(clawData) + 'c'
    print("sent value: ")
    print(baseDataB)
    arduino.write(baseDataB)
    time.sleep(6)
    print("sent value: ")
    print(shoulderDataS)
    arduino.write(shoulderDataS)
    time.sleep(6)
    print("sent value: ")
    print(elbowDataE)
    arduino.write(elbowDataE)
    time.sleep(6)
    print("sent value: ")
    print(clawDataC)
    arduino.write(clawDataC)
    time.sleep(6)

def callBack(data):
    base = 90
    shoulder = 90
    elbow = 90
    claw = 90
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", posData.data)
    print rospy.get_name(), "I heard %s"%str(data.data)
    for i in range(4):
        print data.data[i]
        if(i == 0): base = data.data[i]
        if(i == 1): shoulder = data.data[i]
        if(i == 2): elbow = data.data[i]
        if(i == 3): claw = data.data[i]
    sendData(base, shoulder, elbow, claw)

def listener():
    rospy.init_node("Serial_Communication") #might want to change that to true if it doesn't work
    rospy.Subscriber("posData", Int32MultiArray, callBack)
    rospy.spin()

if __name__ == "__main__":
    #sendData(90, 90, 90, 90)
    listener()
