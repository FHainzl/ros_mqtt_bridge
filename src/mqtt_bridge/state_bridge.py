#!/usr/bin/env python

import rospy

from bridge import Bridge


def main():
    state_bridge = Bridge("state", client_id="state_bridge")

    while not rospy.is_shutdown():
        state_bridge.looping()
