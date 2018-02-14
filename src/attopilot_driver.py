#!/usr/bin/env python
# coding=utf-8

# Author : Simon CHANU
# Notes  : Uniquement un noeud de traitement
# Input  : tension ou courant en Float32
import rospy
from std_msgs.msg import Float32

def cb_voltage(msg):
    global pub_volt
    # 255.75 pts sur 5V avec un ratio de 1:15.7
    coeffTheorique = 1/255.75*5.0*15.7
    coeffAjustement = 1.0
    pub_volt.publish(msg.data * coeffAjustement)

def cb_current(msg):
    global pub_curr
    # 255.75 pts sur 5V avec un ratio de 1:15.7
    coeffTheorique = 1/255.75*5.0*15.7
    coeffAjustement = 1.0
    pub_curr.publish(msg.data * coeffAjustement)

if __name__ == '__main__':

    rospy.init_node('driver_attopilot')
    rospy.loginfo("Node Initialised")

    sub_voltage = ('~volt_topic', "battery1_voltage_raw")
    sub_current = ('~curr_topic', "battery1_current_raw")

    # Publication sans le suffixe _raw
    rospy.Publisher(sub_voltage[:-4], Float32, queue_size=1)
    rospy.Publisher(sub_current[:-4], Float32, queue_size=1)

    # Abonnenements
    pub_volt = rospy.Subscriber(sub_voltage, Float32, cb_voltage)
    pub_curr = rospy.Subscriber(sub_current, Float32, cb_current)

    rospy.spin()
