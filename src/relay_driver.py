#!/usr/bin/env python
# coding=utf-8

# Author : Simon CHANU
# Notes  : Uniquement un noeud de traitement
# Input  : tension ou courant en Float32
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from pwmboard_msgs.msg import PwmCmd

def toogle_relay(req):
    global pub
    msg = PwmCmd()
    msg.pin = 6
    response = SetBoolResponse(0, 'relay desactivated (open)')
    if req.data:
        msg.command = 100
        response = SetBoolResponse(0, 'relay activated (closed)')
    else:
        msg.command = -100

    pub.publish(msg)
    return response

if __name__ == '__main__':

    rospy.init_node('driver_relay')
    rospy.loginfo("Node Initialised")

    s = rospy.Service('toogle_relay', SetBool, toogle_relay)

    # Publication sans le suffixe _raw
    pub = rospy.Publisher('pwm_cmd', PwmCmd, queue_size=1)
    print "service setup"

    rospy.spin()
