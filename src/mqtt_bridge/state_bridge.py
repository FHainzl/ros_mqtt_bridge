#!/usr/bin/env python

import json

from connector.msg import State
from mqtt_bridge.ros2mqtt import ROS2MQTT


class StateBridge(ROS2MQTT):
    def __init__(self, host="localhost", port="1883", keepalive=600):
        ROS2MQTT.__init__(self,
                          ros_topic="/state", ros_msg_type=State,
                          mqtt_topic=None, client_id="state_bridge",
                          host=host, port=port, keepalive=keepalive)

    def forward_ros_msg(self, ros_msg):
        # type: (State)->None
        """
        Subscribe to state message in ROS and publish it in MQTT
        :param ros_msg:
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
        self.mqtt_client.publish("RL/state", mqtt_msg)
