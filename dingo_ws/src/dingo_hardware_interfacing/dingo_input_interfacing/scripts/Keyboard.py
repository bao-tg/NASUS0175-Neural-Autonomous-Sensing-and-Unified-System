import rospy
import sys, signal
import os
from sensor_msgs.msg import Joy

class Keyboard:
    def __init__(self, keyboard):
        self.keyboard = keyboard
        self.used_keys = ['w','a','s','d','z','x','1','2', '7','8','9','0', self.keyboard.Key.shift, self.keyboard.Key.backspace, self.keyboard.Key.up, self.keyboard.Key.down, self.keyboard.Key.left, self.keyboard.Key.right]
        self.speed_multiplier = 1
        self.joystick_message_pub = rospy.Publisher("joy", Joy, queue_size=10)
        self.keyboard_listener = self.keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.keyboard_listener.start()

        self.current_joy_message = Joy()
        self.current_joy_message.axes = [0.,0.,0.,0.,0.,0.,0.,0.]
        self.current_joy_message.buttons = [0,0,0,0,0,0,0,0,0,0,0,0,0]

        
    def on_press(self,key):
        if hasattr(key, 'char'):
            key = key.char
        msg = self.current_joy_message

        if key == self.keyboard.Key.shift:
            self.speed_multiplier = 2
        elif key == 'w' or key == 'W':
            msg.axes[1] = 0.5*self.speed_multiplier
        elif key == 's' or key == 'S':
            msg.axes[1] = -0.5*self.speed_multiplier
        elif key == 'a' or key == 'A':
            msg.axes[0] = 0.5*self.speed_multiplier
        elif key == 'd' or key == 'D':
            msg.axes[0] = -0.5*self.speed_multiplier
        elif key == 'z' or key == 'Z':
            msg.buttons[11] = 1
        elif key == 'x' or key == 'X':
            msg.buttons[12] = 1
        elif key == '1':
            msg.buttons[5] = 1
        elif key == '2':
            msg.buttons[0] = 1
        elif key == self.keyboard.Key.backspace:
            msg.buttons[4] = 1
        elif key == self.keyboard.Key.up:
            msg.axes[4] = 0.5*self.speed_multiplier
        elif key == self.keyboard.Key.down:
            msg.axes[4] = -0.5*self.speed_multiplier
        elif key == self.keyboard.Key.left:
            msg.axes[3] = 0.5*self.speed_multiplier
        elif key == self.keyboard.Key.right:
            msg.axes[3] = -0.5*self.speed_multiplier
        elif key == '0':
            msg.axes[7] = 0.0
        elif key == '9':
            msg.axes[7] = 0.0
        elif key == '8':
            msg.axes[6] = 0.0
        elif key == '7':
            msg.axes[6] = 0.0
        else: return
        self.current_joy_message = msg
        return

    def on_release(self, key):
        if hasattr(key, 'char'):
            key = key.char

        msg = self.current_joy_message

        if key == self.keyboard.Key.shift:
            self.speed_multiplier = 1
        elif key == 'w' or key == 'W':
            msg.axes[1] = 0.0
        elif key == 's' or key == 'S':
            msg.axes[1] = 0.0
        elif key == 'a' or key == 'A':
            msg.axes[0] = 0.0
        elif key == 'd' or key == 'D':
            msg.axes[0] = 0.0
        elif key == 'z' or key == 'Z':
            msg.buttons[11] = 0
        elif key == 'x' or key == 'X':
            msg.buttons[12] = 0
        elif key == '1':
            msg.buttons[5] = 0
        elif key == '2':
            msg.buttons[0] = 0
        elif key == self.keyboard.Key.backspace:
            msg.buttons[4] = 0
        elif key == self.keyboard.Key.up:
            msg.axes[4] = 0.0
        elif key == self.keyboard.Key.down:
            msg.axes[4] = 0.0
        elif key == self.keyboard.Key.left:
            msg.axes[3] = 0.0
        elif key == self.keyboard.Key.right:
            msg.axes[3] = 0.0
        elif key == '0':
            msg.axes[7] = 0.0
        elif key == '9':
            msg.axes[7] = 0.0
        elif key == '8':
            msg.axes[6] = 0.0
        elif key == '7':
            msg.axes[6] = 0.0

        
        self.current_joy_message = msg
    
    def publish_current_command(self):
        self.current_joy_message.header.stamp = rospy.Time.now()
        self.joystick_message_pub.publish(self.current_joy_message)

def signal_handler(sig, frame):
    sys.exit(0)

def main():
    """Main program
    """
    rospy.init_node("keyboard_input_listener")
    rate = rospy.Rate(30)

    if not os.getenv("DISPLAY"):
        rospy.logfatal("DISPLAY is not set. The keyboard node requires access to an X display.")
        rospy.sleep(1)
        sys.exit(0)

    try:
        from pynput import keyboard
    except ImportError as exc:
        rospy.logfatal("Unable to start keyboard listener: %s", exc)
        rospy.logfatal("If running in Docker, allow local X access on the host with: xhost +local:root")
        rospy.sleep(1)
        sys.exit(1)

    signal.signal(signal.SIGINT, signal_handler)
    keyboard_listener = Keyboard(keyboard)

    while not rospy.is_shutdown():
        keyboard_listener.publish_current_command()
        rate.sleep()

main()