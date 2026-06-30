import rospy
import numpy as np
import time
from dingo_control.State import BehaviorState, State
from dingo_control.Command import Command
from dingo_utilities.Utilities import deadband, clipped_first_order_filter
from sensor_msgs.msg import Joy


class InputInterface:
    def __init__(self, config):
        self.config = config
        self.previous_gait_toggle = 0
        self.previous_state = BehaviorState.REST
        self.previous_hop_toggle = 0
        self.previous_joystick_toggle = 0
        self.previous_imu_activate_toggle = 0
        self.previous_imu_deactivate_toggle = 0
        self.previous_home_toggle = 0

        self.rounding_dp = 2

        self.hop_event = 0
        self.trot_event = 0
        self.joystick_control_event = 0
        self.imu_activate_event = 0
        self.imu_deactivate_event = 0
        self.home_event = 0

        self.axis_x = int(rospy.get_param("~joy_axis_x", 1))
        self.axis_y = int(rospy.get_param("~joy_axis_y", 0))
        self.axis_yaw = int(rospy.get_param("~joy_axis_yaw", 2))
        self.axis_pitch = int(rospy.get_param("~joy_axis_pitch", 5))
        self.axis_height = int(rospy.get_param("~joy_axis_height", 7))
        self.axis_roll = int(rospy.get_param("~joy_axis_roll", 6))
        self.button_trot = int(rospy.get_param("~joy_button_trot", 5))
        self.button_hop = int(rospy.get_param("~joy_button_hop", 1))
        self.button_activate = int(rospy.get_param("~joy_button_activate", 4))
        self.button_imu_activate = int(rospy.get_param("~joy_button_imu_activate", 11))
        self.button_imu_deactivate = int(rospy.get_param("~joy_button_imu_deactivate", 12))
        self.button_home = int(rospy.get_param("~joy_button_home", 0))
        self.joy_topic = rospy.get_param("~joy_topic", "joy")

        self.input_messages = rospy.Subscriber(self.joy_topic, Joy, self.input_callback)
        self.current_command = Command()
        self.new_command = Command()
        self.developing_command = Command()

        self.debounce_time = float(rospy.get_param("~joy_debounce_time", 0.3))
        self.last_gait_toggle_time = 0
        self.last_hop_toggle_time = 0
        self.last_joystick_toggle_time = 0
        self.last_home_toggle_time = 0

    def _button(self, msg, index):
        return msg.buttons[index] if len(msg.buttons) > index else 0

    def _axis(self, msg, index):
        return msg.axes[index] if len(msg.axes) > index else 0.0

    def input_callback(self, msg):
        self.developing_command = Command()

        now = time.time()

        gait_toggle = self._button(msg, self.button_trot)
        if self.trot_event != 1:
            if (gait_toggle == 1 and self.previous_gait_toggle == 0
                    and (now - self.last_gait_toggle_time) > self.debounce_time):
                self.trot_event = 1
                self.last_gait_toggle_time = now

        hop_toggle = self._button(msg, self.button_hop)
        if self.hop_event != 1:
            if (hop_toggle == 1 and self.previous_hop_toggle == 0
                    and (now - self.last_hop_toggle_time) > self.debounce_time):
                self.hop_event = 1
                self.last_hop_toggle_time = now

        joystick_toggle = self._button(msg, self.button_activate)
        if self.joystick_control_event != 1:
            if (joystick_toggle == 1 and self.previous_joystick_toggle == 0
                    and (now - self.last_joystick_toggle_time) > self.debounce_time):
                self.joystick_control_event = 1
                self.last_joystick_toggle_time = now

        imu_activate_toggle = self._button(msg, self.button_imu_activate)
        if self.imu_activate_event != 1:
            self.imu_activate_event = (imu_activate_toggle == 1 and self.previous_imu_activate_toggle == 0)

        imu_deactivate_toggle = self._button(msg, self.button_imu_deactivate)
        if self.imu_deactivate_event != 1:
            self.imu_deactivate_event = (imu_deactivate_toggle == 1 and self.previous_imu_deactivate_toggle == 0)

        home_toggle = self._button(msg, self.button_home)
        if self.home_event != 1:
            if (home_toggle == 1 and self.previous_home_toggle == 0
                    and (now - self.last_home_toggle_time) > self.debounce_time):
                self.home_event = 1
                self.last_home_toggle_time = now

        self.previous_gait_toggle = gait_toggle
        self.previous_hop_toggle = hop_toggle
        self.previous_joystick_toggle = joystick_toggle
        self.previous_imu_activate_toggle = imu_activate_toggle
        self.previous_imu_deactivate_toggle = imu_deactivate_toggle
        self.previous_home_toggle = home_toggle

        x_vel = self._axis(msg, self.axis_x) * self.config.max_x_velocity
        y_vel = self._axis(msg, self.axis_y) * self.config.max_y_velocity
        self.developing_command.horizontal_velocity = np.round(np.array([x_vel, y_vel]), self.rounding_dp)
        self.developing_command.yaw_rate = np.round(self._axis(msg, self.axis_yaw), self.rounding_dp) * self.config.max_yaw_rate

        self.developing_command.pitch = np.round(self._axis(msg, self.axis_pitch), self.rounding_dp) * self.config.max_pitch
        self.developing_command.height_movement = np.round(self._axis(msg, self.axis_height), self.rounding_dp)
        self.developing_command.roll_movement = -np.round(self._axis(msg, self.axis_roll), self.rounding_dp)

        self.new_command = self.developing_command

    def get_command(self, state, message_rate):
        self.current_command = self.new_command

        self.current_command.trot_event = self.trot_event
        self.current_command.hop_event = self.hop_event
        self.current_command.joystick_control_event = self.joystick_control_event
        self.current_command.imu_activate_event = self.imu_activate_event
        self.current_command.imu_deactivate_event = self.imu_deactivate_event
        self.current_command.home_event = self.home_event
        self.trot_event = 0
        self.hop_event = 0
        self.joystick_control_event = 0
        self.imu_activate_event = 0
        self.imu_deactivate_event = 0
        self.home_event = 0

        message_dt = 1.0 / message_rate

        deadbanded_pitch = deadband(
            self.current_command.pitch, self.config.pitch_deadband
        )
        pitch_rate = clipped_first_order_filter(
            state.pitch,
            deadbanded_pitch,
            self.config.max_pitch_rate,
            self.config.pitch_time_constant,
        )
        self.current_command.pitch = np.clip(state.pitch + message_dt * pitch_rate, -0.35, 0.35)
        self.current_command.height = np.clip(state.height - message_dt * self.config.z_speed * self.current_command.height_movement, -0.26, -0.08)
        self.current_command.roll = np.clip(state.roll + message_dt * self.config.roll_speed * self.current_command.roll_movement, -0.3, 0.3)

        return self.current_command
