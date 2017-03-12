#!/usr/bin/env python
#THIS IS A PUBLISHER NODE, ultrasonic data to arm_control_node

import sys
import serial
import rospy
import time
import syslog
import math
from std_msgs.msg import String
from std_msgs.msg import Int16, Int32, Int64, Float32, Header, UInt64, UInt64, String
from sensor_msgs.msg import Imu
from SerialDataGateway import SerialDataGateway

class LaunchpadClass(object):

    def __init__(self):
        print "Init Launchpad Class"
        # Variables for the sensors
        self._counter = 0
        self._left_encoder_val = 0
        self._right_encoder_val = 0
        self._ultrasonic_val = 0
        self._qx = 0
        self._qy = 0
        self._qz = 0
        self._qw = 0
        self._left_wheel_spd = 0
        self._right_wheel_spd = 0
        self._lastUpdate_Microsec = 0
        self._sec_Since_Last_Update = 0
        self._robot_Heading = 0
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baudRate = int(rospy.get_param("~baudRate", 9600))

        rospy.loginfo("Launchpad has serial port: " + port + ", baud rate: " + str(baudRate))
        self._Launchpad_Serial = SerialDataGateway(port, baudRate, self.Handle_Received_Line)
        rospy.loginfo("Initiated Launchpad Serial Communication")

        #Publishers for sensors
        # Publisher for left and right encoder values
        self.left_encoder_pub = rospy.Publisher('lwheel', Int64, queue_size = 10)
        self.right_encoder_pub = rospy.Publisher('rwheel', Int64, queue_size = 10)
        # Publisher for Ultrasonic Distance Sensor
        self.ultrasonic_pub = rospy.Publisher('ultrasonic_distance', Float32, queue_size = 10)
        # Publisher for IMU rotation quaternion values
        self._qx_pub = rospy.Publisher('qx', Float32, queue_size = 10)
        self._qy_pub = rospy.Publisher('qy', Float32, queue_size = 10)
        self._qz_pub = rospy.Publisher('qz', Float32, queue_size = 10)
        self._qw_pub = rospy.Publisher('qw', Float32, queue_size = 10)
        # Publisher for all Launchpad Serial Data
        self.SerialPublisher = rospy.Publisher('_Launchpad_Serial', String, queue_size = 10)
        #Publishers for IMU data topic
        self.frame_id = 'base/link'

        self.cal_offset = 0.0
        self.orientation = 0.0
        self.cal_buffer =[]
        self.cal_buffer_length = 1000
        self.imu_data = Imu(header=rospy.Header(frame_id="base_link"))
        self.imu_data.orientation_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
        self.imu_data.angular_velocity_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
        self.imu_data.linear_acceleration_covariance = [-1,0,0,0,0,0,0,0,0]
        self.gyro_measurement_range = 150.0 
        self.gyro_scale_correction = 1.35
        self.imu_pub = rospy.Publisher('imu/data', Imu,queue_size = 10)

        self.deltat = 0
        self.lastUpdate = 0

        #New addon for computing quaternion
        self.pi = 3.14159
        self.GyroMeasError = float(self.pi * ( 40 / 180 ))
        self.beta = float(math.sqrt(3 / 4) * self.GyroMeasError)

        self.GyroMeasDrift = float(self.pi * ( 2 / 180 ))
        self.zeta = float(math.sqrt(3 / 4) * self.GyroMeasDrift)

        self.beta = math.sqrt(3 / 4) * self.GyroMeasError

        self.q = [1,0,0,0]

        #Speed Subscriber
        self.left_motor_spd_pub = rospy.Subscriber('left_wheel_speed', Float32, self.Update_Left_Speed)
        self.right_motor_spd_pub = rospy.Subscriber('right_wheel_speed', Float32, self.Update_Right_Speed)

        def Update_Left_Speed(self, left_spd):
            self._left_wheel_spd = left_spd.data
            rospy.login(left_spd.data)
            speed_msg = 's %d %d\r' %(int(self._left_wheel_spd), int(self._right_wheel_spd))
            self.WriteSerial(speed_msg)

        def Update_Right_Speed(self, right_spd):
            self._right_wheel_spd = right_spd.data
            rospy.loginfo(right_spd.data)
            speed_msg = 's %d %d\r' %(int(self._left_wheel_spd), int(self._right_wheel_spd))
            self.WriteSerial(speed_msg)

        #Calculate orientation from accelerometer and gyrometer
        def Handle_Received_Line(self, line):
            self._counter = self._counter + 1
            self.SerialPublisher.publish(String(str(self._counter) + ", in: " + line))
            if(len(line) > 0):
                lineParts = line.split('\t')
                try:
                    if(lineParts[0] == 'e'):
                        self._left_encoder_val = long(lineParts[1])
                        self._right_encoder_val = long(lineParts[2])

                        self.left_encoder_pub.publish(self._left_encoder_val)
                        self.right_encoder_pub.publish(self._right_encoder_val)
                    if(lineParts[0] == 'u'):
                        self._ultrasonic_val = float(lineParts[1])
                        self.ultrasonic_pub.publish(self._ultrasonic_val)
                    if(lineParts[0] == 'i'):
                        self._qx = float(lineParts[1])
                        self._qy = float(lineParts[2])
                        self._qz = float(lineParts[3])
                        self._qw = float(lineParts[4])

                        self._qx_pub.publish(self._qx)
                        self._qy_pub.publish(self._qy)
                        self._qz_pub.publish(self._qz)
                        self._qw_pub.publish(self._qw)

                        imu_msg = Imu()
                        h = Header()
                        h.stamp = rospy.Time.now()
                        h.frame_id = self.frame_id

                        imu_msg.header = h
                        imu_msg.orientation_covariance = (-1., ) * 9
                        imu_msg.angular_velocity_covariance = (-1., ) * 9
                        imu_msg.linear_acceleration_covariance = (-1., ) * 9
                        imu_msg.orientation.x = self._qx
                        imu_msg.orientation.y = self._qy
                        imu_msg.orientation.z = self._qz
                        imu_msg.orientation.w = self._qw

                        self.imu_pub.publish(imu_msg)
                except:
                    rospy.logwarn("Error in the sensor values")
                    rospy.logwarn(lineParts)
                    pass
        def WriteSerial(self, message):
            self.SerialPublisher.publish(String(str(self._counter) + ", out: " + message))
            self.SerialDataGateway.Write(message)

        def Start(self):
            rospy.lodgebug("Starting")
            self.SerialDataGateway.Start()

        def Stop(self):
            rospy.logdebug("Stopping")
            self.SerialDataGateway.Stop()

        #Sending and Subscribe Speed functions are missing, not sure if needed

        def Reset_Launchpad(self):
            print("Reset")
            reset = 'r\r'
            self.WriteSerial(reset)
            time.sleep(1)
            self.WriteSerial(reset)
            time.sleep(2)


if __name__ == '__main__':
    rospy.init_node('Serial_Senors_And_Motors', anonymous = True)
    launchpad = LaunchpadClass()
    print("Initiated launchpad node")
    try:
        launchpad.Start()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("Error in Serial_Sensors_And_Motors main function")
    launchpad.Reset_Launchpad()
    launchpad.Stop()
