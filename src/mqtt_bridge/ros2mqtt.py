import rospy

from mqtt_bridge.mqtt_client import MQTTClient


class ROS2MQTT(MQTTClient):
    def __init__(self, ros_topic, ros_msg_type,
                 client_id, host, port, keepalive):
        """
        Subscribe to a ROS topic and publish each ROS message as a MQTT message

        To use this class, implement forward_ros_msg(), which parses the
        ROS message and converts it to an MQTT message.

        :param ros_topic: Name of ROS topic to subscribe to
        :param ros_msg_type: Type of messages in subscribed ROS topic
        :param client_id: unique client id string used when connecting to the broker
        :param host: the hostname or IP address of the remote broker
        :param port: the network port of the server host to connect to. Defaults to 1883.
        :param keepalive: maximum period in seconds allowed between communications with the broker
        """

        self.ros_topic = ros_topic
        self.ros_msg_type = ros_msg_type

        MQTTClient.__init__(self, client_id, host, port, keepalive)

        self.rospy_sub = rospy.Subscriber(self.ros_topic, self.ros_msg_type,
                                          callback=self.forward_ros_msg)

    def forward_ros_msg(self, msg):
        raise NotImplementedError
