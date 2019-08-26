import json

from mqtt_bridge.mqtt2ros import MQTT2ROS
import rospy

from connector.msg import Action


class ActionBridge(MQTT2ROS):
    """
    Implementation of MQTT-->ROS class of MQTT-Bridge for actions

    In this case, a machine with a GPU evaluates the policy to obtain actions
    and publishes them via MQTT messages.
    The ActionBridge subscribes to the MQTT action messages, parses the
    JSON string in the message, creates a ROS message and publishes it.
    """

    def __init__(self, mqtt_topic="RL/action", ros_topic="/action",
                 mqtt_client_id="action_bridge",
                 host="localhost", port="1883", keepalive=600):
        self.ros_msg_type = Action
        self.ros_topic = ros_topic
        MQTT2ROS.__init__(self, mqtt_topic, client_id=mqtt_client_id,
                          host=host, port=port, keepalive=keepalive)

        self.ros_pub = rospy.Publisher(self.ros_topic, self.ros_msg_type,
                                       queue_size=1)

    def forward_mqtt_msg(self, ros_action_msg):
        """
        Parse JSON in MQTT message, create ROS message and publish it.
        """
        try:
            action_json = ros_action_msg.payload
            action = json.loads(action_json.decode())

            stamp = action["stamp"]
            sent = action["sent"]
            ddq = action["ddq"]

            assert len(ddq) == 7

            now = rospy.Time.now().to_sec()
            # rospy.loginfo("Stamp " + str(stamp))
            rospy.loginfo("Ping-pong took " + str(now - stamp))
            rospy.loginfo("Pong took " + str(now - sent))

            ros_action_msg = Action()
            stamp = rospy.Time(secs=stamp)
            ros_action_msg.header.stamp = stamp
            ros_action_msg.ddq = ddq

            self.ros_pub.publish(ros_action_msg)

        except Exception as e:
            print(e)
