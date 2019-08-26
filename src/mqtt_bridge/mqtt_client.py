"""
Based on https://github.com/EmaroLab/mqtt_ros_bridge/blob/master/src/bridge.py
"""

import time
import paho.mqtt.client as mqtt


class MQTTClient:
    def __init__(self, client_id, host, port, keepalive,
                 mqtt_subscribe_topic=None):
        """
        Check https://pypi.org/project/paho-mqtt/#client for documentation

        :param client_id: unique client id string used when connecting to the broker
        :param host: the hostname or IP address of the remote broker
        :param port: the network port of the server host to connect to. Defaults to 1883.
        :param keepalive: maximum period in seconds allowed between communications with the broker
        :param mqtt_subscribe_topic: Optional MQTT topic to subscribe to
        """

        self.client_id = client_id
        self.host = host
        self.port = port
        self.keepalive = keepalive

        self.disconnect_flag = False
        self.rc = 1

        self.mqtt_client = mqtt.Client(self.client_id, clean_session=True)

        self.mqtt_client.on_disconnect = self.on_disconnect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.on_subscribe = self.on_subscribe

        self.connect()
        if mqtt_subscribe_topic is not None:
            self.subscribe(mqtt_subscribe_topic)

    def spin(self):
        self.mqtt_client.loop_forever()

    def on_message(self, client, userdata, msg):
        raise NotImplementedError

    def connect(self):
        while self.rc != 0:
            self.rc = self.mqtt_client.connect(self.host, self.port,
                                               self.keepalive)
            time.sleep(1)

    def subscribe(self, topic):
        self.mqtt_client.subscribe(topic)

    def on_subscribe(self, client, userdata, mid, granted_qos):
        print "Subscribed!"

    def disconnect(self):
        print "Disconnecting"
        self.disconnect_flag = True
        self.mqtt_client.disconnect()

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            if not self.disconnect_flag:
                print "Unexpected disconnection."
                print "Trying reconnection"
                self.rc = rc
                self.connect()
