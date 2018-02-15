#!/usr/bin/env python
# coding=utf-8

# Author : Simon CHANU
# Notes  : Uniquement un noeud de traitement TOSO HOTFIX a faire propre.
# Input  : Service de gestion du relais
import rospy
from std_msgs.msg import Float32
from ros_maestro.msg import PwmCmd

def activate_relay(req):
    global pin
    pwmmsg = PwmCmd()
    pwmmsg.pin = pin
    pwmmsg.command = 100.0
    pub.publish(pwmmsg)

def desactivate_relay(req):
    pub.publish(msg.daorique * coeffAjustement)

if __name__ == '__main__':

    rospy.init_node('driver_attopilot')
    rospy.loginfo("driver_maestro Node Initialised")

    pin = ('~relay_pin', "6")

    # Publication sans le suffixe _raw
    rospy.Publisher('pwm_cmd', PwmCmd, queue_size=1)


    rospy.spin()
