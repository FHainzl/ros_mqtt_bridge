#!/usr/bin/env python
"""
Based on https://github.com/EmaroLab/mqtt_ros_bridge/blob/master/src/bridge.py
"""

import paho.mqtt.client as mqtt
import time


class MQTT2ROS:
    def __init__(self, mqtt_topic, client_id="mqtt2ros",
                 host="localhost", port="1883", keepalive=600):
        self.mqtt_topic = mqtt_topic
        self.client_id = client_id
        self.host = host
        self.port = port
        self.keepalive = keepalive

        self.disconnect_flag = False
        self.rc = 1
        self.timeout = 0

        self.client = mqtt.Client(self.client_id, clean_session=True)

        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.on_unsubscribe = self.on_unsubscribe
        self.client.on_subscribe = self.on_subscribe

        self.connect()

    def connect(self):
        while self.rc != 0:
            # try:
            self.rc = self.client.connect(self.host, self.port,
                                          self.keepalive)
            # except:
            #     print "Connection failed!"
            time.sleep(1)

    def forward_msg(self, msg):
        raise NotImplementedError

    def run(self):
        self.client.loop_forever()

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        self.client.subscribe(self.mqtt_topic)
        self.timeout = 0

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            if not self.disconnect_flag:
                print
                "Unexpected disconnection."
                print
                "Trying reconnection"
                self.rc = rc
                self.connect()

    def on_message(self, client, userdata, msg):
        self.forward_msg(msg)

    def unsubscribe(self):
        print "Unsubscribing"
        self.client.unsubscribe(self.mqtt_topic)

    def disconnect(self):
        print "Disconnecting"
        self.disconnect_flag = True
        self.client.disconnect()

    def on_unsubscribe(self, client, userdata, mid):
        if self.mqtt_topic == '#':
            print "Unsubscribed to all the topics"
        else:
            print "Unsubscribed to " + self.mqtt_topic

    def on_subscribe(self, client, userdata, mid, granted_qos):
        if self.mqtt_topic == '#':
            print "Subscribed to all the topics"
        else:
            print "Subscribed to " + self.mqtt_topic

    def hook(self):
        self.unsubscribe()
        self.disconnect()
        print "Shutting down"
