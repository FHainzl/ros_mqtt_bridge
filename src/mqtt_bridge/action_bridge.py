from mqtt_bridge.mqtt2ros import MQTT2ROS
import rospy

from connector.msg import Action


class ActionBridge(MQTT2ROS):
    def __init__(self, mqtt_topic, ros_topic,
                 mqtt_client_id="action_bridge",
                 host="localhost", port="1883", keepalive=600):
        MQTT2ROS.__init__(self, mqtt_topic, client_id=mqtt_client_id,
                          host=host, port=port, keepalive=keepalive)

        self.ros_msg_type = Action
        self.ros_topic = ros_topic

        self.ros_pub = rospy.Publisher(self.ros_topic, self.ros_msg_type,
                                       queue_size=1)

    def forward_msg(self, msg):
        """
        Forward mqtt message to ros
        """
        # ddq = msg.ddq
        # ros_msg = Action(ddq = ddq)
        # self.ros_pub.publish(ros_msg)
