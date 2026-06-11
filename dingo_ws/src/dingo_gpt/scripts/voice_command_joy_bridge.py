#!/usr/bin/env python3

import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String


class VoiceCommandJoyBridge(object):
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))
        self.linear_axis_value = float(rospy.get_param("~linear_axis_value", 0.5))
        self.turn_axis_value = float(rospy.get_param("~turn_axis_value", 0.5))
        self.step_duration = float(rospy.get_param("~step_duration", 0.35))
        self.turn_duration = float(rospy.get_param("~turn_duration", 0.8))
        self.turn_around_duration = float(rospy.get_param("~turn_around_duration", 1.6))
        self.seconds_per_90_degrees = float(rospy.get_param("~seconds_per_90_degrees", 3.0))
        self.sit_duration = float(rospy.get_param("~sit_duration", 2.0))
        self.button_pulse_duration = float(rospy.get_param("~button_pulse_duration", 0.18))
        self.default_steps = int(rospy.get_param("~default_steps", 1))
        self.max_steps = int(rospy.get_param("~max_steps", 30))
        self.step_limit_message = "Don't try to move over 30 steps to prevent user input the edge case"
        self.speak_ack = rospy.get_param("~speak_ack", True)

        self.pub_joy = rospy.Publisher("joy", Joy, queue_size=10)
        self.pub_tts = rospy.Publisher("/tts_input", String, queue_size=10)
        self.pub_follow_enable = rospy.Publisher("/person_follow/enable", Bool, queue_size=1, latch=True)
        rospy.Subscriber("/robot_command", String, self.command_callback, queue_size=10)

        self.lock = threading.Lock()
        self.busy = False
        self.in_trot = False

    def command_callback(self, msg):
        try:
            command = json.loads(msg.data)
        except ValueError:
            rospy.logwarn("Voice bridge: ignoring invalid JSON command: %s", msg.data)
            return

        if command.get("category") != "Control mode":
            return

        with self.lock:
            if self.busy:
                rospy.logwarn("Voice bridge: command ignored because another command is running")
                if self.speak_ack:
                    self.pub_tts.publish("I am still finishing the previous command.")
                return
            self.busy = True

        worker = threading.Thread(target=self.run_command_safely, args=(command,))
        worker.daemon = True
        worker.start()

    def run_command_safely(self, command):
        try:
            self.run_command(command)
        except Exception as exc:
            rospy.logerr("Voice bridge: command failed: %s", exc)
            self.publish_neutral()
        finally:
            with self.lock:
                self.busy = False

    def run_command(self, command):
        action = (command.get("action") or "").lower()
        direction = (command.get("direction") or "").lower()
        raw_text = command.get("raw_text") or ""

        rospy.loginfo("Voice bridge: executing %s", json.dumps(command, sort_keys=True))

        if action not in ["follow"]:
            self.pub_follow_enable.publish(Bool(data=False))

        if action == "move":
            self.run_move(direction, self.get_steps(command))
        elif action == "turn":
            self.run_turn(direction, raw_text, self.get_degrees(command))
        elif action == "sit":
            self.run_sit()
        elif action == "stand":
            self.run_stand()
        elif action == "stop":
            self.run_stop()
        elif action == "follow":
            self.run_follow()
        elif action == "stop_follow":
            self.run_stop_follow()
        else:
            rospy.logwarn("Voice bridge: unsupported action %r", action)
            if self.speak_ack:
                self.pub_tts.publish("I heard the command, but I cannot map it to motion yet.")

    def run_move(self, direction, steps):
        if steps > self.max_steps:
            rospy.logwarn("Voice bridge: rejecting move command with %s steps; max is %s", steps, self.max_steps)
            if self.speak_ack:
                self.pub_tts.publish(self.step_limit_message)
            return

        if direction not in ["forward", "backward", "left", "right"]:
            rospy.logwarn("Voice bridge: move command missing direction")
            return

        axes = self.neutral_axes()
        if direction == "forward":
            axes[1] = self.linear_axis_value
        elif direction == "backward":
            axes[1] = -self.linear_axis_value
        elif direction == "left":
            axes[0] = self.linear_axis_value
        elif direction == "right":
            axes[0] = -self.linear_axis_value

        self.ensure_trot()
        self.hold_axes(axes, max(steps, 1) * self.step_duration)
        self.publish_neutral()
        self.ensure_rest()

        if self.speak_ack:
            self.pub_tts.publish("Done.")

    def run_turn(self, direction, raw_text, degrees):
        axes = self.neutral_axes()
        duration = self.turn_duration
        if direction == "right":
            axes[3] = -self.turn_axis_value
        elif direction == "left":
            axes[3] = self.turn_axis_value
        elif "around" in raw_text.lower():
            axes[3] = self.turn_axis_value
            degrees = degrees or 180
        else:
            axes[3] = self.turn_axis_value

        if degrees:
            duration = max(0.1, (float(degrees) / 90.0) * self.seconds_per_90_degrees)
        elif "around" in raw_text.lower():
            duration = self.turn_around_duration

        self.ensure_trot()
        self.hold_axes(axes, duration)
        self.publish_neutral()
        self.ensure_rest()

        if self.speak_ack:
            self.pub_tts.publish("Done.")

    def run_sit(self):
        axes = self.neutral_axes()
        axes[7] = -1.0
        self.hold_axes(axes, self.sit_duration)
        self.publish_neutral()
        if self.speak_ack:
            self.pub_tts.publish("Sitting down.")

    def run_stand(self):
        axes = self.neutral_axes()
        axes[7] = 1.0
        self.hold_axes(axes, self.sit_duration)
        self.publish_neutral()
        if self.speak_ack:
            self.pub_tts.publish("Standing up.")

    def run_stop(self):
        self.pub_follow_enable.publish(Bool(data=False))
        self.publish_neutral()
        self.ensure_rest()
        if self.speak_ack:
            self.pub_tts.publish("Stopped.")

    def run_follow(self):
        self.pub_follow_enable.publish(Bool(data=True))
        if self.speak_ack:
            self.pub_tts.publish("Starting follow mode. Stand still while I lock on.")

    def run_stop_follow(self):
        self.pub_follow_enable.publish(Bool(data=False))
        self.publish_neutral()
        self.ensure_rest()
        if self.speak_ack:
            self.pub_tts.publish("Follow mode stopped.")

    def ensure_trot(self):
        if not self.in_trot:
            self.pulse_button(5)
            self.in_trot = True
            rospy.sleep(0.15)

    def ensure_rest(self):
        if self.in_trot:
            self.pulse_button(5)
            self.in_trot = False
            rospy.sleep(0.15)

    def pulse_button(self, button_index):
        buttons = self.neutral_buttons()
        buttons[button_index] = 1
        self.hold_message(self.neutral_axes(), buttons, self.button_pulse_duration)
        self.publish_neutral()

    def hold_axes(self, axes, duration):
        self.hold_message(axes, self.neutral_buttons(), duration)

    def hold_message(self, axes, buttons, duration):
        period = 1.0 / self.rate_hz
        end_time = time.time() + max(duration, period)
        while not rospy.is_shutdown() and time.time() < end_time:
            self.publish_joy(axes, buttons)
            time.sleep(period)

    def publish_neutral(self):
        self.publish_joy(self.neutral_axes(), self.neutral_buttons())

    def publish_joy(self, axes, buttons):
        msg = Joy()
        msg.header.stamp = rospy.Time.now()
        msg.axes = list(axes)
        msg.buttons = list(buttons)
        self.pub_joy.publish(msg)

    def get_steps(self, command):
        steps = command.get("steps")
        try:
            steps = int(steps)
        except (TypeError, ValueError):
            steps = self.default_steps
        return max(1, steps)

    def get_degrees(self, command):
        degrees = command.get("degrees")
        try:
            degrees = int(degrees)
        except (TypeError, ValueError):
            return None
        return max(1, min(360, degrees))

    def neutral_axes(self):
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def neutral_buttons(self):
        return [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


if __name__ == "__main__":
    rospy.init_node("voice_command_joy_bridge")
    VoiceCommandJoyBridge()
    rospy.loginfo("Voice bridge: listening on /robot_command and publishing Joy on joy")
    rospy.spin()
