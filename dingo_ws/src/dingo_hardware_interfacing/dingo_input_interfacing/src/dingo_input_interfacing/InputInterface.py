import rospy
import numpy as np
from dingo_control.State import BehaviorState, State
from dingo_control.Command import Command
from dingo_utilities.Utilities import deadband, clipped_first_order_filter
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Int32




class InputInterface:
    def __init__(self, config):
        self.config = config
        self.previous_gait_toggle = 0
        self.previous_state = BehaviorState.REST
        self.previous_hop_toggle = 0
        self.previous_joystick_toggle = 0


        self.rounding_dp = 2


        self.hop_event = 0
        self.trot_event = 0
        self.joystick_control_event = 0


        self.current_command = Command()
        self.new_command = Command()
        self.developing_command = Command()
       
        self.stances = {
            'Stand': {'roll': 0.0, 'pitch': 0.0, 'height': -0.20, 'x_shift': -0.005, 'stance_y': 0.0},
            'LieDown': {'roll': 0.0, 'pitch': 0.0, 'height': -0.10, 'x_shift': 0.02, 'stance_y': 0.0},
            'LookDown': {'roll': 0.0, 'pitch': -0.26, 'height': -0.20, 'x_shift': -0.005, 'stance_y': 0.0},
            'LookDown_10deg': {'roll': 0.0, 'pitch': -0.174, 'height': -0.20, 'x_shift': -0.001, 'stance_y': 0.0},
            'LookDown_20deg': {'roll': 0.0, 'pitch': -0.349, 'height': -0.20, 'x_shift': -0.001, 'stance_y': 0.0},
            'LookDown_30deg': {'roll': 0.0, 'pitch': -0.523, 'height': -0.20, 'x_shift': -0.014, 'stance_y': 0.0}
        }
        self.target_pitch = 0.0
        self.target_height = -0.20
        self.target_roll = 0.0
        self.target_x_shift = 0.0
        self.target_y_shift = 0.0
        self.current_x_shift = 0.0
        self.current_y_shift = 0.0


        self.target_yaw = 0.0
        self.trigger_yaw_macro = False
        self.macro_yaw_active = False
        self.yaw_macro_start_ticks = 0
        self.yaw_macro_total_ticks = 0
        self.macro_yaw_rate = 0.0
       
        self.trigger_step_macro = False
        self.step_macro_active = False
        self.step_start_ticks = 0
        self.target_steps = 1


        # ALWAYS initialize subscribers last to avoid race conditions
        # where callbacks trigger before attributes are defined!
        self.input_messages = rospy.Subscriber("joy", Joy, self.input_callback)
        rospy.Subscriber("/cmd_height", Float64, self.cmd_height_callback)
        rospy.Subscriber("/cmd_pitch", Float64, self.cmd_pitch_callback)
        rospy.Subscriber("/cmd_yaw", Float64, self.cmd_yaw_callback)
        rospy.Subscriber("/cmd_steps", Int32, self.cmd_steps_callback)


    def cmd_height_callback(self, msg):
        self.target_height = msg.data


    def cmd_pitch_callback(self, msg):
        self.target_pitch = msg.data


    def cmd_yaw_callback(self, msg):
        self.target_yaw = msg.data
        self.trigger_yaw_macro = True


    def cmd_steps_callback(self, msg):
        if msg.data > 0:
            self.target_steps = msg.data
            self.trigger_step_macro = True


    def set_stance(self, stance_name):
        stance = self.stances[stance_name]
        self.target_roll = stance['roll']
        self.target_pitch = stance['pitch']
        self.target_height = stance['height']
        self.target_x_shift = stance['x_shift']
        self.target_y_shift = stance['stance_y']


    def input_callback(self, msg):
        self.developing_command = Command()
        ####### Handle discrete commands ########
        # Check if requesting a state transition to trotting, or from trotting to resting
        gait_toggle = msg.buttons[5] #R1
        if self.trot_event != 1:
            self.trot_event = (gait_toggle == 1 and self.previous_gait_toggle == 0)


        # Check if requesting a state transition to hopping, from trotting or resting
        hop_toggle = msg.buttons[0] #x
        if self.hop_event != 1:
            self.hop_event = (hop_toggle == 1 and self.previous_hop_toggle == 0)            
       
        joystick_toggle = msg.buttons[4] #L1
        if self.joystick_control_event != 1:
            self.joystick_control_event = (joystick_toggle == 1 and self.previous_joystick_toggle == 0)


        # Update previous values for toggles and state
        self.previous_gait_toggle = gait_toggle
        self.previous_hop_toggle = hop_toggle
        self.previous_joystick_toggle = joystick_toggle


        # Check stance buttons
        if msg.buttons[1] == 1:
            self.set_stance('Stand')
        elif msg.buttons[2] == 1:
            self.set_stance('LieDown')
        elif msg.buttons[3] == 1:
            self.set_stance('LookDown')
        elif msg.buttons[6] == 1:
            self.set_stance('LookDown_10deg')
        elif msg.buttons[7] == 1:
            self.set_stance('LookDown_20deg')
        elif msg.buttons[8] == 1:
            self.set_stance('LookDown_30deg')


        # Check macro buttons
        if msg.buttons[9] == 1:
            self.trigger_yaw_macro = True
            self.target_yaw = 1.57 # ~90 degrees (1.57 rad)
           
        if msg.axes[3] != 0:
            # Manual override of yaw cancels macro
            self.macro_yaw_active = False
           
        if msg.buttons[10] == 1:
            self.trigger_step_macro = True
            self.target_steps = 1


        ####### Handle continuous commands ########
        x_vel = (msg.axes[1] ) * self.config.max_x_velocity #ly
        y_vel = msg.axes[0] * self.config.max_y_velocity #lx
        self.developing_command.horizontal_velocity =  np.round(np.array([x_vel, y_vel]),self.rounding_dp)
        self.developing_command.yaw_rate = np.round(msg.axes[3],self.rounding_dp) * self.config.max_yaw_rate #rx


        self.developing_command.pitch = self.target_pitch + np.round(msg.axes[4],self.rounding_dp) * self.config.max_pitch #ry
        self.developing_command.height_movement = np.round(msg.axes[7],self.rounding_dp) #dpady
        self.developing_command.roll_movement = -np.round(msg.axes[6],self.rounding_dp) #dpadx


        self.new_command = self.developing_command
       
    def get_command(self, state, message_rate):


        self.current_command = self.new_command


        self.current_command.trot_event = self.trot_event
        self.current_command.hop_event  = self.hop_event
        self.current_command.joystick_control_event = self.joystick_control_event
        self.hop_event = 0
        self.trot_event = 0
        self.joystick_control_event = 0


        message_dt = 1.0 / message_rate


        # Update targets based on manual continuous movement
        if self.current_command.height_movement != 0:
            self.target_height = np.clip(self.target_height - message_dt * self.config.z_speed * self.current_command.height_movement, -0.27, -0.05)
        if self.current_command.roll_movement != 0:
            self.target_roll = np.clip(self.target_roll + message_dt * self.config.roll_speed * self.current_command.roll_movement, -0.3, 0.3)


        deadbanded_pitch = deadband(
            self.current_command.pitch, self.config.pitch_deadband
        )
        pitch_rate = clipped_first_order_filter(
            state.pitch,
            deadbanded_pitch,
            self.config.max_pitch_rate,
            self.config.pitch_time_constant,
        )
       
        self.current_command.pitch  = np.clip(state.pitch + message_dt * pitch_rate, -0.55, 0.55)
       
        # Smoothly interpolate state towards targets for height and shifts
        height_diff = self.target_height - state.height
        self.current_command.height = np.clip(state.height + np.sign(height_diff) * min(abs(height_diff), message_dt * self.config.z_speed * 1.5), -0.27, -0.05)


        roll_diff = self.target_roll - state.roll
        self.current_command.roll = np.clip(state.roll + np.sign(roll_diff) * min(abs(roll_diff), message_dt * self.config.roll_speed * 1.5), -0.3, 0.3)
       
        x_diff = self.target_x_shift - self.current_x_shift
        self.current_x_shift += np.sign(x_diff) * min(abs(x_diff), message_dt * 0.05) # 0.05 m/s
        self.current_command.x_shift = self.current_x_shift


        y_diff = self.target_y_shift - self.current_y_shift
        self.current_y_shift += np.sign(y_diff) * min(abs(y_diff), message_dt * 0.05)
        self.current_command.y_shift = self.current_y_shift


        # Execute Yaw Macro: physically trot to turn the robot
        if self.trigger_yaw_macro and not self.macro_yaw_active and state.behavior_state == BehaviorState.REST:
            self.macro_yaw_active = True
            self.trigger_yaw_macro = False # Reset the trigger
            self.current_command.trot_event = True
            self.yaw_macro_start_ticks = state.ticks
           
            # Reset pitch and roll, keep height
            self.target_pitch = 0.0
            self.target_roll = 0.0
           
            if self.target_yaw == 0.0:
                self.yaw_macro_total_ticks = 0
                self.macro_yaw_rate = 0.0
            else:
                # Calculate exactly how many gait cycles we need to reach target_yaw
                desired_yaw_rate = 1.2 # rad/s turning speed
               
                # The controller math uses self.config.dt (0.01) to step the gait, not message_dt!
                radians_per_phase = desired_yaw_rate * (self.config.phase_length * self.config.dt)
               
                # Number of full gait cycles needed
                num_phases = max(1, int(np.ceil(abs(self.target_yaw) / radians_per_phase)))
                self.yaw_macro_total_ticks = num_phases * self.config.phase_length
               
                # Physical slip multiplier (robots slip on the floor when turning in place)
                slip_multiplier = 1.35
               
                # Adjust actual yaw_rate so it hits the target perfectly at the end of the last phase
                self.macro_yaw_rate = slip_multiplier * (self.target_yaw / (self.yaw_macro_total_ticks * self.config.dt))


        if self.macro_yaw_active:
            self.current_command.yaw_rate = self.macro_yaw_rate
            self.current_command.horizontal_velocity = np.array([0.0, 0.0])
           
            # Wait for the exact calculated number of ticks
            if state.ticks - self.yaw_macro_start_ticks >= self.yaw_macro_total_ticks:
                self.current_command.trot_event = True # Flip back to REST
                self.macro_yaw_active = False
                self.current_command.yaw_rate = 0.0


        # Execute Step Macro
        if self.trigger_step_macro and not self.step_macro_active and state.behavior_state == BehaviorState.REST:
            self.step_macro_active = True
            self.trigger_step_macro = False # Reset the trigger
            self.current_command.trot_event = True
            self.step_start_ticks = state.ticks


        if self.step_macro_active:
            # Force horizontal velocity
            self.current_command.horizontal_velocity = np.array([0.15, 0.0])
            if state.ticks - self.step_start_ticks >= self.config.phase_length * self.target_steps:
                self.current_command.trot_event = True # Flip back to REST
                self.step_macro_active = False
                self.current_command.horizontal_velocity = np.array([0.0, 0.0])


        return self.current_command

