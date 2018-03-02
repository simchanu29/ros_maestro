# ros_maestro
Gestion de la carte polulu maestro

### Installation

Clonez ce repository dans un workspace ROS : `git clone https://github.com/simchanu29/ros_maestro.git`

### Utilisation

Ce package a été écrit en python 2.7, il utilise le driver `maestro.py` fourni par [FRC4564](https://github.com/FRC4564/Maestro).
Il utilise un fichier de configuration dont un example est présent dans le dossier `config`

Le principe de ce package n'est pas d'être optimisé mais de servir d'outil souple pour la conception de prototypes sous ROS.

Pour lancer l'exemple : `roslaunch ros_maestro test.launch`

Pour lancer les noeuds avec une configuration spécifique, on inclus le launchfile `maestro.launch` avec ses arguments. Un exemple est ci-dessous :
```xml
<launch>

    <include file="$(find ros_maestro)/launch/maestro.launch">
      <arg name="config_path" value="$(find ros_maestro)/config/pwmconfig_example.yaml">
      <arg name="port" value="/dev/polulu_servo_serial" >
    </include>

    <node name="attopilot1" pkg="ros_maestro" type="attopilot_driver.py">
      <param name="~volt_topic" value="battery1_voltage_raw"/>
      <param name="~curr_topic" value="battery1_current_raw"/>
    </node>

    <node name="relay" pkg="ros_maestro" type="relay_driver.py" />

</launch>
```

### Troubleshooting

 - Si le noeud n'arrive pas à se connecter à la Maestro, assurez vous d'avoir les droits sur le port tty concerné.
