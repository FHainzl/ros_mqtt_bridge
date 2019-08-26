#!/usr/bin/env python

import json

from connector.msg import State
from mqtt_bridge.ros2mqtt import ROS2MQTT


class StateBridge(ROS2MQTT):
    """
    Implementation of ROS-->MQTT class of MQTT-Bridge for states

    In this case, a machine running ROS is connected to all sensors.
    The StateBridge subscribes to the ROS state messages, parses them and saves
    the fields in a Python dictionary, which is then converted to JSON and
    published via MQTT.
    """

    def __init__(self, ros_topic="/state", mqtt_topic="RL/state",
                 client_id="state_bridge",
                 host="localhost", port="1883", keepalive=600):
        self.mqtt_topic = mqtt_topic
        ROS2MQTT.__init__(self, ros_topic=ros_topic, ros_msg_type=State,
                          client_id=client_id,
                          host=host, port=port, keepalive=keepalive)

    def forward_ros_msg(self, ros_msg):
        # type: (State)->None
        """
        Conversion of ROS message to JSON string and published as MQTT message
        """
        stamp = ros_msg.header.stamp.to_sec()
        q = ros_msg.q
        dq = ros_msg.dq
        phi = ros_msg.phi
        dphi = ros_msg.dphi

        msg_dict = {
            "stamp": stamp,
            "q": q,
            "dq": dq,
            "phi": phi,
            "dphi": dphi
        }

        mqtt_msg = json.dumps(msg_dict)
        self.mqtt_client.publish(self.mqtt_topic, mqtt_msg)
