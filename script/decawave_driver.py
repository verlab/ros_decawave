#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_decawave')
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
        self.port_ = rospy.get_param('~port', '/dev/ttyACM0')
        self.baudrate_ = int(rospy.get_param('~baudrate', '115200'))
        self.tf_publisher_ = str(rospy.get_param('~tf_publisher', "True"))
        self.tf_reference_ = str(rospy.get_param('~tf_reference', 'world'))
        self.tag_name_ = str(rospy.get_param('~tag_name', 'tag'))
        self.rate_ = int(rospy.get_param('~rate', '10'))
        self.serial_timeout = rospy.Duration(0.5)
        # Initiate Serial
        self.ser = serial.Serial(self.port_, self.baudrate_, timeout=0.1)
        rospy.loginfo("\33[96mConnected to %s at %i\33[0m", self.ser.portstr, self.baudrate_)
        self.get_uart_mode()
        self.switch_uart_mode()
        self.get_tag_status()
        self.get_tag_version()
        self.anchors = AnchorArray()
        self.anchors.anchors = []
        self.tag = Tag()


    def get_uart_mode(self):
        """ Check UART Mode Used """ 
        rospy.loginfo("\33[96mChecking which UART mode is the gateway...\33[0m")
        self.mode_ = 'UNKNOWN'
        self.ser.write(b'\r') # Test Mode
        time.sleep(0.1)
        while(self.ser.inWaiting() == 0):
            pass
        cc = self.ser.readline()
        if cc == '\r\n' and self.ser.readline() == 'dwm> ': # SHELL MODE
            rospy.loginfo("\33[96mDevice is on SHELL MODE! It must to be changed to GENERIC MODE!\33[0m")
            self.mode_ = "SHELL"
        elif cc == '@\x01\x01': # GENERIC MODE
            rospy.loginfo("\33[96mDevice is on GENERIC MODE! Ok!\33[0m")
            self.mode_ = "GENERIC"
        return self.mode_
    

    def switch_uart_mode(self):
        if self.mode_ == "SHELL":
            rospy.loginfo("\33[96mChanging UART mode to GENERIC MODE...\33[0m")
            self.ser.write(b'quit\r') # Go to Generic Mode         
            while(self.ser.inWaiting()==0):
                pass
            self.ser.readline()
            rospy.loginfo("\33[96m%s\33[0m", self.ser.readline().replace('\n', ''))
        elif self.mode_ == "UNKNOWN":
            rospy.logerr("\33[96m%s\33[0m", "Unknown Mode Detected! Please reset the device and try again!")

    def get_tag_version(self):
        self.ser.flushInput()
        self.ser.write(b'\x15\x00') # Status
        now = rospy.Time.now()
        while(self.ser.inWaiting() < 21):
            if (rospy.Time.now() - now) > self.serial_timeout: 
                rospy.logwarn("Malformed packet! Ignoring tag version.")
                self.ser.flushInput()
                return None
        version = self.ser.read(21)
        data_ = struct.unpack('<BBBBBLBBLBBL', bytearray(version))
        rospy.loginfo("\33[96m--------------------------------\33[0m")
        rospy.loginfo("\33[96mFirmware Version:0x"+format(data_[5], '04X')+"\33[0m")
        rospy.loginfo("\33[96mConfiguration Version:0x"+format(data_[8], '04X')+"\33[0m")
        rospy.loginfo("\33[96mHardware Version:0x"+format(data_[11], '04X')+"\33[0m")
        rospy.loginfo("\33[96m--------------------------------\33[0m")
        
    def get_tag_acc(self):
        # Acc is not implemented on Generic Mode
        self.ser.flushInput()
        self.ser.write(b'\x19\x33\x04') # Status
        while(self.ser.inWaiting() == 0):
            pass
        data_ = self.ser.readline()
        print ("%s", data_)

    def get_tag_status(self):
        self.ser.flushInput()
        self.ser.write(b'\x32\x00') # Status
        while(self.ser.inWaiting()==0):
            pass
        status = self.ser.readline()
        data_ = struct.unpack('<BBBBBB', bytearray(status))
        if data_[0] != 64 and data_[2] != 0:
            rospy.logwarn("Get Status Failed! Packet does not match!")
            print("%s", data_)
        if data_[5] == 3:
            rospy.loginfo("\33[96mTag is CONNECTED to a UWB network and LOCATION data are READY!\33[0m")
        elif data_[5] == 2:
            rospy.logwarn("Tag is CONNECTED to a UWB network but LOCATION data are NOT READY!")
        elif data_[5] == 1:
            rospy.logwarn("Tag is NOT CONNECTED to a UWB network but LOCATION data are READY!")
        elif data_[5] == 0:
            rospy.logwarn("Tag is NOT CONNECTED to a UWB network and LOCATION data are NOT READY!")


    def get_tag_location(self):
        self.ser.flushInput()
        self.ser.write(b'\x0c\x00')
        now = rospy.Time.now()
        while (self.ser.inWaiting() < 21):
            if (rospy.Time.now() - now) > self.serial_timeout: 
                rospy.logwarn("Malformed packet! Ignoring tag location.")
                self.ser.flushInput()
                return None
        data_ = self.ser.read(21)
        data_ = struct.unpack('<BBBBBlllBBBB', bytearray(data_))
        self.tag.x = float(data_[5])/1000.0
        self.tag.y = float(data_[6])/1000.0
        self.tag.z = float(data_[7])/1000.0
        self.tag.qf = float(data_[8])/100.0
        self.tag.n_anchors = int(data_[11])
        self.tag.header.frame_id = self.tag_name_

        self.anchor_packet_size = 20 ## Size of anchor packet in bytes
        now = rospy.Time.now()
        while (self.ser.inWaiting() < self.anchor_packet_size * self.tag.n_anchors):
            if (rospy.Time.now() - now) > self.serial_timeout: 
                rospy.logwarn("Malformed packet! Ignoring anchors location.")
                self.ser.flushInput()
                return None
        self.anchors.anchors = [] ## Clean Anchors list
        for i in range(self.tag.n_anchors):

            data_ = self.ser.read(self.anchor_packet_size)
            data_ = struct.unpack('<HlBlllB', bytearray(data_))
            a = Anchor()
            a.header.frame_id = str(format(data_[0], '04X'))
            a.header.stamp = rospy.Time.now()
            a.distance = float(data_[1])/1000.0
            a.dist_qf = float(data_[2])/100.0
            a.x = float(data_[3])/1000.0
            a.y = float(data_[4])/1000.0
            a.z = float(data_[5])/1000.0
            a.qf = float(data_[6])/100.0
            self.anchors.anchors.append(a)


    def tf_callback(self, timer):
        if self.tf_publisher_ == "True":
            self.br.sendTransform((self.tag.x, self.tag.y, self.tag.z),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        self.tag_name_,
                        self.tf_reference_)
            for anchor in self.anchors.anchors:
                self.br.sendTransform((anchor.x, anchor.y, anchor.z),
                    tf.transformations.quaternion_from_euler(0, 0, 0),
                    rospy.Time.now(),
                    anchor.header.frame_id,
                    self.tf_reference_)


    def run(self):
        self.rate = rospy.Rate(self.rate_)
        rospy.loginfo("\33[96mInitiating Driver...\33[0m")
        self.tag_pub_ = rospy.Publisher('tag_pose', Tag, queue_size=1)
        self.anchors_pub_ = rospy.Publisher('tag_status', AnchorArray, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tf_callback)
        self.br = tf.TransformBroadcaster()
        while not rospy.is_shutdown():
            self.get_tag_location()
            self.tag.header.stamp = rospy.Time.now()
            self.tag_pub_.publish(self.tag)
            self.anchors.header.stamp = rospy.Time.now()
            self.anchors_pub_.publish(self.anchors)
            self.rate.sleep()

# Main function
if __name__ == '__main__':
    try:
        dd = DecawaveDriver()
        dd.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Decawave Driver]: Closed!")

