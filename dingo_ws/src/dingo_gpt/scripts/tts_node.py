#!/usr/bin/env python3

import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import rospy
from std_msgs.msg import String

from dingo_openai_utils import synthesize_speech


class TTSNode(object):
    def __init__(self):
        self.model = rospy.get_param("~model", "tts-1")
        self.voice = rospy.get_param("~voice", "alloy")
        self.player = rospy.get_param("~player", "aplay")
        self.keep_audio = rospy.get_param("~keep_audio", False)
        self.device = rospy.get_param("~playback_device", "")
        rospy.Subscriber("/tts_input", String, self.text_callback, queue_size=10)

    def text_callback(self, msg):
        text = msg.data.strip()
        if not text:
            return

        rospy.loginfo("TTS: speaking: %s", text)
        output_path = None
        try:
            output_path = synthesize_speech(text, voice=self.voice, model=self.model, response_format="wav")
            cmd = [self.player]
            if self.player == "aplay" and self.device:
                cmd.extend(["-D", self.device])
            cmd.append(output_path)
            subprocess.check_call(cmd)
        except Exception as exc:
            rospy.logerr("TTS: failed: %s", exc)
        finally:
            if output_path and not self.keep_audio:
                try:
                    os.remove(output_path)
                except OSError:
                    pass


if __name__ == "__main__":
    rospy.init_node("tts_node")
    TTSNode()
    rospy.loginfo("TTS: listening for text on /tts_input")
    rospy.spin()
