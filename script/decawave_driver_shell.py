#!/usr/bin/env python

import rospy
import tf
import time

import serial
import struct

from geometry_msgs.msg import PointStamped
from ros_decawave.msg import Tag, Anchor, AnchorArray, Acc


class DecawaveDriver(object):
    """ docstring for DecawaveDriver """
    def __init__(self):
        rospy.init_node('decawave_driver', anonymous=False)
        # Getting Serial Parameters
        self.port_ = rospy.get_param('port', '/dev/ttyACM0')
        self.baudrate_ = int(rospy.get_param('baudrate', '115200'))
        self.tf_publisher_ = rospy.get_param('tf_publisher', 'True')
        self.rate_ = int(rospy.get_param('rate', '10'))
        # Initiate Serial
        self.ser = serial.Serial(self.port_, self.baudrate_, timeout=0.1)
        rospy.loginfo("\33[96mConnected to %s at %i\33[0m", self.ser.portstr, self.baudrate_)
        self.get_uart_mode()
        self.switch_uart_mode()
        #self.get_tag_status()
        #self.get_tag_version()
        self.anchors = AnchorArray()
        self.anchors.anchors = []
        self.tag = Tag()
        self.accel = Acc()


    def get_uart_mode(self):
        """ Check UART Mode Used """ 
        rospy.loginfo("\33[96mChecking which UART mode is the gateway...\33[0m")
        self.mode_ = 'UNKNOWN'
        self.ser.flushInput()
        self.ser.write(b'\r') # Test Mode
        time.sleep(0.1)
        while(self.ser.inWaiting() == 0):
            pass
        cc = self.ser.readline()
        if cc == '@\x01\x01': # GENERIC MODE
            rospy.loginfo("\33[96mDevice is on GENERIC MODE!  It must to be changed to SHELL MODE!\33[0m")
            self.mode_ = "GENERIC"
        else: # SHELL MODE
            rospy.loginfo("\33[96mDevice is on SHELL MODE! Ok!\33[0m")
            self.mode_ = "SHELL"
        
        return self.mode_
    

    def switch_uart_mode(self):
        self.ser.flushInput()
        if self.mode_ == "GENERIC":
            rospy.loginfo("\33[96mChanging UART mode to SHELL MODE...\33[0m")
            self.ser.write(b'\r\r') # Go to Shell Mode         
            while(self.ser.inWaiting()==0):
                pass
            time.sleep(1.0)
            self.ser.flushInput()
            rospy.loginfo("\33[96m%s\33[0m", self.ser.readline().replace('\n', ''))
        elif self.mode_ == "UNKNOWN":
            rospy.logerr("%s", "Unknown Mode Detected! Please reset the device and try again!")


    def get_tag_version(self):
        self.ser.flushInput()
        self.ser.write(b'\x15\x00') # Status
        while(self.ser.inWaiting() < 21):
            pass
        version = self.ser.read(21)
        data_ = struct.unpack('<BBBBBLBBLBBL', bytearray(version))
        rospy.loginfo("\33[96m--------------------------------\33[0m")
        rospy.loginfo("\33[96mFirmware Version:0x"+format(data_[5], '04X')+"\33[0m")
        rospy.loginfo("\33[96mConfiguration Version:0x"+format(data_[8], '04X')+"\33[0m")
        rospy.loginfo("\33[96mHardware Version:0x"+format(data_[11], '04X')+"\33[0m")
        rospy.loginfo("\33[96m--------------------------------\33[0m")

    #def get_tag_status(self):
    #    self.ser.flushInput()
    #    self.ser.write(b'\x32\x00') # Status
    #    while(self.ser.inWaiting()==0):
    #        pass
    #    status = self.ser.readline()
    #    data_ = struct.unpack('<BBBBBB', bytearray(status))
    #    if data_[0] != 64 and data_[2] != 0:
    #        rospy.logwarn("Get Status Failed! Packet does not match!")
    #        print("%s", data_)
    #    if data_[5] == 3:
    #        rospy.loginfo("\33[96mTag is CONNECTED to a UWB network and LOCATION data are READY!\33[0m")
    #    elif data_[5] == 2:
    #        rospy.logwarn("Tag is CONNECTED to a UWB network but LOCATION data are NOT READY!")
    #    elif data_[5] == 1:
    #        rospy.logwarn("Tag is NOT CONNECTED to a UWB network but LOCATION data are READY!")
    #    elif data_[5] == 0:
    #        rospy.logwarn("Tag is NOT CONNECTED to a UWB network and LOCATION data are NOT READY!")

    def get_tag_acc(self):
        """ Read Acc Value: The values are raw values. So to convert them to g you first have to divide the 
        values by 2^6 ( as it is shifted) and then multiply it into 0.004 (assuming you are using the 
        +-2g scale). With regards to the getting the accelerometer readings to the UART, I have written 
        specific functions to read the data . I could put the github link up if you want."""
        self.ser.flushInput()
        self.ser.write(b'av\r') # Test Mode
        while(self.ser.inWaiting() == 0):
            pass
        cc = ''
        t = rospy.Time.now()
        while not 'acc' in cc:
            cc = self.ser.readline()
            if rospy.Time.now() - t > rospy.Duration(0.5):
                rospy.logwarn("Could not get accel data!")
        cc = cc.replace('\r\n', '').replace('acc: ', '').split(',')
        if len(cc) == 3:
            self.accel.x = float(int(cc[0].replace('x = ', ''))>>6) * 0.04
            self.accel.y = float(int(cc[1].replace('y = ', ''))>>6) * 0.04
            self.accel.z = float(int(cc[2].replace('z = ', ''))>>6) * 0.04
            self.accel.header.frame_id = 'tag'
            self.accel.header.stamp = rospy.Time.now()


    def get_tag_location(self):
        self.ser.flushInput()
        self.ser.write(b'lec\r') # Test Mode
        while(self.ser.inWaiting() == 0):
            pass
        cc = ''
        t = rospy.Time.now()
        while not 'DIST' in cc:
            cc = self.ser.readline()
            print (cc)
            if rospy.Time.now() - t > rospy.Duration(0.5):
                rospy.logwarn("Could not get tag data!")
                self.ser.flushInput()
                self.ser.write(b'\r') # Test Mode
        #cc = ''
        #t = rospy.Time.now()
        #while not 'acc' in cc:
        #    cc = self.ser.readline()
        #    if rospy.Time.now() - t > rospy.Duration(0.5):
        #        rospy.logwarn("Could not get accel data!")
        #cc = cc.replace('\r\n', '').replace('acc: ', '').split(',')
        #if len(cc) == 3:
        #    self.accel.x = float(int(cc[0].replace('x = ', ''))/64.0) * 0.04
        #    self.accel.y = float(int(cc[1].replace('y = ', ''))/64.0) * 0.04
        #    self.accel.z = float(int(cc[2].replace('z = ', ''))/64.0) * 0.04
        #    self.accel.header.frame_id = 'tag'
        #    self.accel.header.stamp = rospy.Time.now()
        

    def tf_callback(self, timer):
        if self.tf_publisher_ == 'True':
            self.br.sendTransform((self.tag.x, self.tag.y, self.tag.z),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "tag",
                        "world")
            for anchor in self.anchors.anchors:
                self.br.sendTransform((anchor.x, anchor.y, anchor.z),
                    tf.transformations.quaternion_from_euler(0, 0, 0),
                    rospy.Time.now(),
                    anchor.header.frame_id,
                    "world")


    def run(self):
        self.rate = rospy.Rate(self.rate_)
        rospy.loginfo("\33[96mInitiating Driver...\33[0m")
        self.tag_pub_ = rospy.Publisher('pose', Tag, queue_size=1)
        self.anchors_pub_ = rospy.Publisher('status', AnchorArray, queue_size=1)
        self.acc_pub_ = rospy.Publisher('accelerometer', Acc, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.2), self.tf_callback)
        self.br = tf.TransformBroadcaster()
        while not rospy.is_shutdown():
            self.get_tag_acc()
            self.acc_pub_.publish(self.accel)
            #self.get_tag_location()
            #self.tag.header.stamp = rospy.Time.now()
            #self.tag_pub_.publish(self.tag)
            #self.anchors.header.stamp = rospy.Time.now()
            #self.anchors_pub_.publish(self.anchors)
            self.rate.sleep()

# Main function
if __name__ == '__main__':
    try:
        dd = DecawaveDriver()
        dd.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Decawave Driver]: Closed!")

