#!/usr/bin/env python

import paho.mqtt.client as mqtt
import time

from mqtt_bridge.mqtt_client import MQTTClient


class MQTT2ROS(MQTTClient):
    def __init__(self, mqtt_topic, client_id="mqtt2ros",
                 host="localhost", port="1883", keepalive=600):
        MQTTClient.__init__(self, client_id, host, port, keepalive,
                            mqtt_subscribe_topic=mqtt_topic)

    def forward_mqtt_msg(self, msg):
        raise NotImplementedError

    def on_message(self, client, userdata, msg):
        self.forward_mqtt_msg(msg)
