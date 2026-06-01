#!/usr/bin/env python3

import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import rospy
from std_msgs.msg import String

from dingo_openai_utils import chat_completion


VALID_CATEGORIES = set(["Describe the scene", "Control mode", "Unknown"])


class IntentLLMNode(object):
    def __init__(self):
        self.model = rospy.get_param("~model", "gpt-3.5-turbo")
        self.speak_unknown = rospy.get_param("~speak_unknown", False)
        self.pub_category = rospy.Publisher("/intent/category", String, queue_size=10)
        self.pub_command = rospy.Publisher("/robot_command", String, queue_size=10)
        self.pub_scene = rospy.Publisher("/scene_describe_request", String, queue_size=10)
        self.pub_tts = rospy.Publisher("/tts_input", String, queue_size=10)
        rospy.Subscriber("/stt_text", String, self.text_callback, queue_size=10)

    def text_callback(self, msg):
        text = msg.data.strip()
        if not text:
            return

        rospy.loginfo("Intent: classifying: %s", text)
        command = self.classify(text)
        category = command.get("category", "Unknown")
        command_json = json.dumps(command, sort_keys=True)

        self.pub_category.publish(category)
        self.pub_command.publish(command_json)
        rospy.loginfo("Intent: %s", command_json)

        if category == "Describe the scene":
            self.pub_scene.publish(text)
        elif category == "Unknown" and self.speak_unknown:
            self.pub_tts.publish("I heard you, but I do not know which robot mode to use yet.")

    def classify(self, text):
        rule_result = self.rule_classify(text)
        if rule_result:
            return rule_result

        prompt = (
            "Classify a voice command for a quadruped robot. Return only JSON with keys: "
            "category, action, direction, steps, degrees, raw_text. category must be exactly one of "
            "\"Describe the scene\", \"Control mode\", \"Unknown\". "
            "Use Control mode for sit, stand up, move forward/backward/left/right, turn, stop. "
            "For turn commands, extract degrees when the user says degrees; turn around means 180 degrees. "
            "Use Describe the scene when the user asks what the robot sees."
        )
        try:
            response_text = chat_completion(
                [
                    {"role": "system", "content": prompt},
                    {"role": "user", "content": text},
                ],
                model=self.model,
                max_tokens=160,
                temperature=0.0,
            )
            parsed = json.loads(self.extract_json(response_text))
            parsed["raw_text"] = text
            if parsed.get("category") not in VALID_CATEGORIES:
                parsed["category"] = "Unknown"
            parsed.setdefault("action", None)
            parsed.setdefault("direction", None)
            parsed.setdefault("steps", None)
            parsed.setdefault("degrees", None)
            return parsed
        except Exception as exc:
            rospy.logerr("Intent: LLM classification failed: %s", exc)
            return {"category": "Unknown", "action": None, "direction": None, "steps": None, "degrees": None, "raw_text": text}

    def rule_classify(self, text):
        lower = text.lower()
        if any(phrase in lower for phrase in ["describe what you see", "what do you see", "describe the scene", "look around"]):
            return {"category": "Describe the scene", "action": "describe_scene", "direction": None, "steps": None, "raw_text": text}

        action = None
        direction = None
        if "sit" in lower:
            action = "sit"
        elif "stand" in lower or "stand up" in lower:
            action = "stand"
        elif "stop" in lower:
            action = "stop"
        elif "move" in lower or "walk" in lower or "go" in lower or "turn" in lower:
            action = "move"
            for candidate in ["forward", "backward", "left", "right"]:
                if candidate in lower:
                    direction = candidate
                    break
            if "back" in lower and direction is None:
                direction = "backward"
            if "turn" in lower:
                action = "turn"

        if action:
            return {
                "category": "Control mode",
                "action": action,
                "direction": direction,
                "steps": self.extract_steps(lower),
                "degrees": self.extract_degrees(lower) if action == "turn" else None,
                "raw_text": text,
            }
        return None

    def extract_degrees(self, lower):
        if "turn around" in lower or "turnaround" in lower:
            return 180

        match = re.search(r"\b(\d{1,3})\s*(degree|degrees|deg)\b", lower)
        if match:
            return int(match.group(1))

        degree_match = re.search(r"\b([a-z -]+)\s+(degree|degrees)\b", lower)
        if degree_match:
            value = self.words_to_number(degree_match.group(1).split())
            if value is not None:
                return value
        return None

    def words_to_number(self, words):
        values = {
            "zero": 0,
            "one": 1,
            "two": 2,
            "three": 3,
            "four": 4,
            "five": 5,
            "six": 6,
            "seven": 7,
            "eight": 8,
            "nine": 9,
            "ten": 10,
            "eleven": 11,
            "twelve": 12,
            "thirteen": 13,
            "fourteen": 14,
            "fifteen": 15,
            "sixteen": 16,
            "seventeen": 17,
            "eighteen": 18,
            "nineteen": 19,
            "twenty": 20,
            "thirty": 30,
            "forty": 40,
            "fifty": 50,
            "sixty": 60,
            "seventy": 70,
            "eighty": 80,
            "ninety": 90,
        }
        current = 0
        found = False
        for word in words:
            word = word.strip("-")
            if word in ["turn", "left", "right", "around", "degrees", "degree"]:
                current = 0
                found = False
                continue
            if word == "hundred" and current:
                current *= 100
                found = True
                continue
            if word in values:
                current += values[word]
                found = True
        if not found:
            return None
        return current

    def extract_steps(self, lower):
        words = {
            "one": 1,
            "two": 2,
            "three": 3,
            "four": 4,
            "five": 5,
            "six": 6,
            "seven": 7,
            "eight": 8,
            "nine": 9,
            "ten": 10,
        }
        match = re.search(r"\b(\d+)\s+(step|steps)\b", lower)
        if match:
            return int(match.group(1))
        for word, value in words.items():
            if re.search(r"\b%s\s+(step|steps)\b" % word, lower):
                return value
        return None

    def extract_json(self, text):
        start = text.find("{")
        end = text.rfind("}")
        if start >= 0 and end > start:
            return text[start : end + 1]
        return text


if __name__ == "__main__":
    rospy.init_node("intent_llm_node")
    IntentLLMNode()
    rospy.loginfo("Intent: listening for text on /stt_text")
    rospy.spin()
