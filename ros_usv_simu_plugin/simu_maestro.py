#!/usr/bin/env python
# coding=utf-8

# Node ROS simu_pwm board

"""
S'abonne aux commandes de PWM
Renvoie sur le topic associé au moteur la commande (-100 à 100)

En fait cette node sert juste de hub de redistribution au niveau de la simulation.
En effet dans le simulateur chaque moteur va écouter sur son canal propre.

Cette node ne peut être utilisée que si la node de hardware ros_maestro est installée.
De manière générale ce sera souvent le cas qu'un driver simulé ne puisse fonctionner que si sa contrepartie réelle est
installée.
"""

import rospy
import numpy as np
import rosparam, rospkg
from pwmboard_msgs.msg import PwmCmd
from std_msgs.msg import Int16, Float32

class PwmOutput:
    def __init__(self, pin):
        # Define publisher
        self.pub = rospy.Publisher('pwm_out_'+str(pin), Int16, queue_size=1)

    def publish(self, pwm):
        self.pub.publish(int(pwm))

class PwmInput:
    def __init__(self, pin, pubname):
        self.pub = rospy.Publisher('/'+str(pubname), Float32, queue_size=1)
        self.sub = rospy.Subscriber('pwm_in_'+str(pin), Float32, self.cb)

    def cb(self, msg):
        # Ce qui est reçois du 0-5V et renvoie du 0-1023
        # msg = Int16((msg.data/5.0*1023))
        # print "publishing ",msgs
        self.pub.publish(int(msg.data/5.0*1024))


class SimPWMBoard():
    def __init__(self, actuators, sensors):
        self.out_tab = []
        self.in_tab = []

        # O/I simulated hardware
        for device in actuators:
            pin = actuators[device]['pin']
            self.out_tab.append(PwmOutput(pin))
        for device in sensors:
            pin = sensors[device]['pin']
            self.in_tab.append(PwmInput(pin, device))

        # Inputs software
        self.sub = rospy.Subscriber('/cmd_pwm', PwmCmd, self.update_cmd_pwm)

    def update_cmd_pwm(self, msg):
        pin = int(msg.pin)
        # Gère les messages entre -100 et 100
        if np.abs(msg.command)>100.0:
            msg.command = np.clip(msg.command, -100.0, 100.0)
        self.out_tab[pin].publish(msg.command/2.0*10+1500)


if __name__ == '__main__':
    rospy.init_node('simu_maestro')

    # === COMMON ===

    # Necessite le formalisme de configuration de ros_usv_simulator

    # Necessite le formalisme de configuration de ros_usv_simulator

    # La node doit se lancer en sachant où chercher sa config
    node_name = rospy.get_name()
    device_type_name = rospy.get_param(node_name + '_type_name')

    # Config
    ns = rospy.get_namespace()
    nslen = len(ns)
    prefix_len = nslen + 5  # On enlève le namespace et simu_
    reduced_node_name = node_name[prefix_len:]
    config_node = rospy.get_param('robot/' + device_type_name + '/' + reduced_node_name)
    print 'CONFIG NODE '+reduced_node_name
    print config_node

    # Pas sur qu'on en ai besoin mais au cas où
    device_type = config_node['type']
    config_device = rospy.get_param('device_types/' + device_type)
    print 'CONFIG DEVICE '+reduced_node_name
    print config_device

    # === SPECIFIC ===

    # Import de la config maestro
    # Specifque aux plugins de ros_usv_simulator
    config_pkg_name = rospy.get_param('config_pkg_name', 'ros_helios_config')
    internal_pkg_path = config_node['config_path']
    print 'config_pkg_name : '+config_pkg_name
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path(config_pkg_name)
    rosparam_result = rosparam.load_file(pkg_path+'/'+internal_pkg_path+'pwmconfig.yaml')
    print 'config_pkg_path : '+pkg_path+'/'+internal_pkg_path+'pwmconfig.yaml'
    yaml = rosparam_result[0][0]
    print 'yaml result'
    print yaml

    config_maestro = yaml['maestro']['device']
    data_type_maestro = yaml['maestro']['data_types']

    del rospack
    del pkg_path
    del rosparam_result
    del yaml

    actuators = {}
    sensors = {}
    for device in config_maestro:
        print data_type_maestro[config_maestro[device]['data_type']]['type']
        if data_type_maestro[config_maestro[device]['data_type']]['type'] == 'input':
            sensors[device] = config_maestro[device]
        if data_type_maestro[config_maestro[device]['data_type']]['type'] == 'output':
            actuators[device] = config_maestro[device]

    # Launch node
    simu = SimPWMBoard(actuators, sensors)

    rospy.spin()
