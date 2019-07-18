import rospy

from mqtt_bridge.mqtt_client import MQTTClient


class ROS2MQTT(MQTTClient):
    def __init__(self, ros_topic, ros_msg_type,
                 client_id, host, port, keepalive, mqtt_topic):
        MQTTClient.__init__(self, client_id, host, port, keepalive, mqtt_topic)

        self.ros_topic = ros_topic
        self.ros_msg_type = ros_msg_type

        self.rospy_sub = rospy.Subscriber(self.ros_topic, self.ros_msg_type,
                                          callback=self.forward_msg)

    def forward_msg(self, msg):
        raise NotImplementedError
