#!/usr/bin/env python3
import time

import rospy
from sensor_msgs.msg import Joy


class JoyMux:
    def __init__(self):
        self.output_topic = rospy.get_param("~output_topic", "joy")
        self.controller_topic = rospy.get_param("~controller_topic", "/joy/controller")
        self.voice_topic = rospy.get_param("~voice_topic", "/joy/voice")
        self.follow_topic = rospy.get_param("~follow_topic", "/joy/follow")
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))
        self.active_timeout = float(rospy.get_param("~active_timeout", 0.35))
        self.controller_timeout = float(rospy.get_param("~controller_timeout", 1.0))
        self.axis_deadband = float(rospy.get_param("~axis_deadband", 0.02))
        self.publish_neutral_when_idle = bool(rospy.get_param("~publish_neutral_when_idle", False))

        self.sources = {
            "voice": {"msg": None, "stamp": 0.0, "active": False, "timeout": self.active_timeout},
            "follow": {"msg": None, "stamp": 0.0, "active": False, "timeout": self.active_timeout},
            "controller": {"msg": None, "stamp": 0.0, "active": True, "timeout": self.controller_timeout},
        }

        self.pub = rospy.Publisher(self.output_topic, Joy, queue_size=10)
        rospy.Subscriber(self.controller_topic, Joy, self.callback_factory("controller"), queue_size=10)
        rospy.Subscriber(self.voice_topic, Joy, self.callback_factory("voice"), queue_size=10)
        rospy.Subscriber(self.follow_topic, Joy, self.callback_factory("follow"), queue_size=10)

        rospy.loginfo(
            "Joy mux: controller=%s voice=%s follow=%s -> %s",
            self.controller_topic,
            self.voice_topic,
            self.follow_topic,
            self.output_topic,
        )

    def callback_factory(self, source):
        def callback(msg):
            self.sources[source]["msg"] = msg
            self.sources[source]["stamp"] = time.time()
            self.sources[source]["active"] = self.is_active(msg) if source != "controller" else True
        return callback

    def is_active(self, msg):
        if any(abs(axis) > self.axis_deadband for axis in msg.axes):
            return True
        return any(button != 0 for button in msg.buttons)

    def neutral_message(self):
        msg = Joy()
        msg.header.stamp = rospy.Time.now()
        msg.axes = [0.0] * 8
        msg.buttons = [0] * 13
        return msg

    def select_message(self):
        now = time.time()
        for source in ("voice", "follow"):
            entry = self.sources[source]
            if entry["msg"] is not None and entry["active"] and now - entry["stamp"] <= entry["timeout"]:
                return entry["msg"]

        controller = self.sources["controller"]
        if controller["msg"] is not None and now - controller["stamp"] <= controller["timeout"]:
            return controller["msg"]

        if self.publish_neutral_when_idle:
            return self.neutral_message()
        return None

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            msg = self.select_message()
            if msg is not None:
                msg.header.stamp = rospy.Time.now()
                self.pub.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("joy_mux")
    JoyMux().spin()
