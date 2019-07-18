import paho.mqtt.client as mqtt

import time


class MQTTClient:
    def __init__(self, client_id, host, port, keepalive,
                 mqtt_subscribe_topic=None):
        self.client_id = client_id
        self.host = host
        self.port = port
        self.keepalive = keepalive

        self.disconnect_flag = False
        self.rc = 1
        self.timeout = 0

        self.client = mqtt.Client(self.client_id, clean_session=True)

        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.on_subscribe = self.on_subscribe

        self.connect()
        if mqtt_subscribe_topic is not None:
            self.subscribe(mqtt_subscribe_topic)

    def connect(self):
        while self.rc != 0:
            self.rc = self.client.connect(self.host, self.port, self.keepalive)
            time.sleep(1)

    def subscribe(self, topic):
        self.client.subscribe(topic)

    def run(self):
        self.client.loop_forever()

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            if not self.disconnect_flag:
                print "Unexpected disconnection."
                print "Trying reconnection"
                self.rc = rc
                self.connect()

    def on_message(self, client, userdata, msg):
        self.process_msg(msg)

    def process_msg(self, msg):
        pass

    def on_subscribe(self, client, userdata, mid, granted_qos):
        print "Subscribed!"

    def disconnect(self):
        print "Disconnecting"
        self.disconnect_flag = True
        self.client.disconnect()

    def hook(self):
        self.disconnect()
        print "Shutting down"
