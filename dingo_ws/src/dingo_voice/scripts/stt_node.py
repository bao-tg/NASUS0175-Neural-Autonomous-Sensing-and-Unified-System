#!/usr/bin/env python3

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import rospy
from std_msgs.msg import String

from dingo_openai_utils import transcribe_audio


class STTNode(object):
    def __init__(self):
        self.model = rospy.get_param("~model", "gpt-4o-transcribe")
        self.language = rospy.get_param("~language", "")
        self.prompt = rospy.get_param("~prompt", "Robot commands may include: Hey Robot, move forward, move backward, sit, stand up, stop, describe what you see.")
        self.publish_to_gpt_input = rospy.get_param("~publish_to_gpt_input", True)

        self.pub_text = rospy.Publisher("/stt_text", String, queue_size=10)
        self.pub_gpt_input = rospy.Publisher("/gpt_input", String, queue_size=10)
        rospy.Subscriber("/voice/audio_file", String, self.audio_callback, queue_size=1)

    def audio_callback(self, msg):
        audio_path = msg.data.strip()
        if not audio_path:
            return
        if not os.path.exists(audio_path):
            rospy.logerr("STT: audio file does not exist: %s", audio_path)
            return

        rospy.loginfo("STT: transcribing %s", audio_path)
        try:
            text = transcribe_audio(audio_path, model=self.model, language=self.language or None, prompt=self.prompt or None)
        except Exception as exc:
            rospy.logerr("STT: transcription failed: %s", exc)
            return

        if not text:
            rospy.logwarn("STT: transcription returned empty text")
            return

        rospy.loginfo("STT: %s", text)
        self.pub_text.publish(text)
        if self.publish_to_gpt_input:
            self.pub_gpt_input.publish(text)


if __name__ == "__main__":
    rospy.init_node("stt_node")
    STTNode()
    rospy.loginfo("STT: listening for audio paths on /voice/audio_file")
    rospy.spin()
