#!/usr/bin/env python
# coding=utf-8

# Author : Simon
# Notes  : Utilise un message specifique
# Input  : Reçois les commandes des autres noeuds pour les moteurs sous la forme d'une pin et d'une commande en pwm
import rospy
from std_msgs.msg import Float32
from maestro.maestro import Controller
# from maestro_sim.maestro import Controller # En cas de problème de driver, celui-ci est plus compréhensible et se debug bien
from ros_maestro.msg import PwmCmd

# 1000 : marche avant
# 1500 : statique
# 2000 : marche arrière
# Channel : 0:L 1:R

class PWMBoard(Controller):
    def __init__(self, port, devices, sensors, data_types):
        Controller.__init__(self, ttyStr=port)

        # Formattage des données (quelle pin de la carte associée à quoi)
        self.devices_by_pins = self.gen_dic_by_pin_keys(devices)
        self.devices_by_name = devices
        self.types = data_types
        self.sensors = sensors
        print 'devices_by_pins : ', self.devices_by_pins

        for device in self.devices_by_name:
            pin = self.devices_by_name[device]['pin']
            command_type = self.devices_by_name[device]['command_type']
            self.setAccel(pin, self.types[data_type]['accel'])

    def gen_dic_by_pin_keys(self, devices):
        """
        Transforme la table de hachage où on accède aux numéros des pins par le nom de l'appareil en une table de
        hachage où on accède au nom de l'appareil par son numéro de pin associé
        :param pwm_devices:
        :return pin_dic:
        """
        pin_dic = dict()
        for device in devices:
            print 'device :', device
            pin = int(devices[device]['pin'])
            pin_dic[pin] = device
        return pin_dic

    def cb_pwm(self, msg):

        print 'pin :', msg.pin  # pin en int
        print 'cmd :', msg.command  # commande en float

        # Gestion du type de commande
        device_name = self.devices_by_pins[msg.pin]
        print 'device_name', device_name
        type = self.devices_by_name[device_name]['command_type']
        print 'type', type
        range = self.types[type]['range']
        range_min = range[0]
        range_max = range[1]
        range_tot = range_max - range_min
        range_zero = range_min + range_tot / 2.0
        print 'range', range

        # Calcul de la commande en pwm
        cmd = (msg.command - range_zero) * 1000 / range_tot + 1500
        print 'pwm sent to board :', int(cmd)

        # Envoi de la commande (traduction en polulu 0-2000 = 0-8192)
        cmd = int(cmd*4.096)
        print 'cmd sent to board :', int(cmd)
        self.setTarget(int(msg.pin), int(cmd))

    def publisher(self, device):
        pub = sensors[device]['publisher']
        pin = int(sensors[device]['pin'])

        # rospy.loginfo("getting positions")
        val = self.getPosition(pin)
        # rospy.loginfo("Sensors values")
        pub.publish(val)

if __name__ == '__main__':

    rospy.init_node('driver_maestro')

    rospy.loginfo("driver_maestro Node Initialised")
    port = rospy.get_param('~port', "/dev/ttyACM0")
    devices = rospy.get_param('~device')
    data_types = rospy.get_param('~data_type')

    maestro = PWMBoard(port, devices, data_types)
    rospy.Subscriber('pwm_cmd', PwmCmd, maestro.cb_pwm)

    sensors = {}
    for device in devices:
        if devices[device]['type']=='input':
            sensors[device] = devices[device]
            sensors['publisher'] = rospy.Publisher(devices[device]['data_type'], Float32, queue_size=1)

    while not rospy.is_shutdown():
        try:
            rospy.rostime.wallsleep(0.5)
            for device in sensors:
                maestro.publish(device)
        except rospy.ROSInterruptException:
            maestro.close()