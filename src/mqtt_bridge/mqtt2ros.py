#!/usr/bin/env python

import paho.mqtt.client as mqtt
import time

from mqtt_bridge.mqtt_client import MQTTClient


class MQTT2ROS(MQTTClient):
    def __init__(self, mqtt_topic,
                 client_id="mqtt2ros", host="localhost", port="1883",
                 keepalive=600):
        """
        Subscribe to a MQTT topic and publish each MQTT message as a ROS message

        To use this class, implement forward_mqtt_msg(), which parses the
        MQTT message and converts it to a ROS message.

        :param mqtt_topic: Name of MQTT topic to subscribe to
        :param client_id: unique client id string used when connecting to the broker
        :param host: the hostname or IP address of the remote broker
        :param port: the network port of the server host to connect to. Defaults to 1883.
        :param keepalive: maximum period in seconds allowed between communications with the broker
        """
        MQTTClient.__init__(self, client_id, host, port, keepalive,
                            mqtt_subscribe_topic=mqtt_topic)

    def forward_mqtt_msg(self, msg):
        raise NotImplementedError

    def on_message(self, client, userdata, msg):
        self.forward_mqtt_msg(msg)
