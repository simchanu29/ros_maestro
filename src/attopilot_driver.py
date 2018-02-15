#!/usr/bin/env python
# coding=utf-8

# Author : Simon CHANU
# Notes  : Uniquement un noeud de traitement
# Input  : tension ou courant en Float32
import rospy
from std_msgs.msg import Float32
import numpy as np

class Attopilot:
    def __init__(self):
        self.curr_tab = []
        self.volt_tab = []
        self.tab_len = 100
        self.voltage_limit = 20.0
        self.current_limit = 30.0

    def cb_voltage(self, msg):
        # 255.75 pts sur 5V avec un ratio de 1:15.7
        coeffTheorique = 1/255.75*5.0*15.7
        coeffAjustement = 1.0/13.83
        voltage_raw = msg.data * coeffAjustement
        self.volt_tab.append(voltage_raw)
        if len(self.volt_tab)>self.tab_len:
            self.volt_tab = self.volt_tab[1:]

        voltage = np.mean(self.volt_tab)
        if voltage<self.voltage_limit:
            rospy.logerr("Battery too low (<20V) [safety margin : 2V]")
        self.pub_volt.publish(voltage)

    def cb_current(self, msg):
        # 255.75 pts sur 5V avec un ratio de 1:15.7
        coeffTheorique = 1/255.75*5.0*15.7
        coeffAjustement = 1.0/49.8
        current_raw = msg.data * coeffAjustement
        self.curr_tab.append(current_raw)
        if len(self.curr_tab)>self.tab_len:
            self.curr_tab = self.curr_tab[1:]

        current = np.mean(self.curr_tab)
        if current<self.current_limit:
            rospy.logerr("Consumption too high on one battery (>15A) [2 batteries : 30A]")
        self.pub_curr.publish(current)

if __name__ == '__main__':

    curr_tab = []
    volt_tab = []

    attopilot = Attopilot()

    rospy.init_node('driver_attopilot')
    rospy.loginfo("Node Initialised")

    sub_voltage = rospy.get_param('~volt_topic', "battery1_voltage_raw")
    sub_current = rospy.get_param('~curr_topic', "battery1_current_raw")

    # Publication sans le suffixe _raw
    attopilot.pub_volt = rospy.Publisher(sub_voltage[:-4], Float32, queue_size=1)
    attopilot.pub_curr = rospy.Publisher(sub_current[:-4], Float32, queue_size=1)

    # Abonnenements
    rospy.Subscriber(sub_voltage, Float32, attopilot.cb_voltage)
    rospy.Subscriber(sub_current, Float32, attopilot.cb_current)

    rospy.spin()
