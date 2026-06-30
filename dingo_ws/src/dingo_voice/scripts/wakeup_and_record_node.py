#!/usr/bin/env python3

import audioop
import os
import re
import sys
import math
import struct
import subprocess
import tempfile
import time
import wave

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import rospy
from std_msgs.msg import String

from dingo_openai_utils import transcribe_audio


def normalize_text(text):
    return re.sub(r"[^a-z0-9 ]+", "", text.lower()).strip()


def compact_text(text):
    return normalize_text(text).replace(" ", "")


class WakeupAndRecordNode(object):
    def __init__(self):
        self.wake_phrase = rospy.get_param("~wake_phrase", "hey robot")
        aliases = rospy.get_param("~wake_aliases", "hey robot,hey robots,hey robo,hey robert")
        self.wake_aliases = [item.strip() for item in aliases.split(",") if item.strip()]
        if self.wake_phrase not in self.wake_aliases:
            self.wake_aliases.insert(0, self.wake_phrase)
        self.audio_device = rospy.get_param("~audio_device", "plughw:2,0")
        self.sample_rate = int(rospy.get_param("~sample_rate", 16000))
        self.channels = int(rospy.get_param("~channels", 1))
        self.chunk_seconds = float(rospy.get_param("~wake_chunk_seconds", 2.0))
        self.command_max_seconds = float(rospy.get_param("~command_max_seconds", 6.0))
        self.silence_seconds = float(rospy.get_param("~silence_seconds", 1.2))
        self.silence_rms = int(rospy.get_param("~silence_rms", 450))
        self.wake_min_rms = int(rospy.get_param("~wake_min_rms", 250))
        self.min_command_seconds = float(rospy.get_param("~min_command_seconds", 0.6))
        self.command_start_delay = float(rospy.get_param("~command_start_delay", 0.15))
        self.wake_cooldown_seconds = float(rospy.get_param("~wake_cooldown_seconds", 2.0))
        self.language = rospy.get_param("~language", "")
        self.stt_model = rospy.get_param("~stt_model", "gpt-4o-transcribe")
        self.stt_prompt = rospy.get_param("~stt_prompt", "")
        self.wake_engine = rospy.get_param("~wake_engine", "cloud")
        self.feedback_mode = rospy.get_param("~wake_feedback_mode", "beep")
        self.beep_frequency = float(rospy.get_param("~beep_frequency", 880.0))
        self.beep_duration = float(rospy.get_param("~beep_duration", 0.16))
        self.beep_volume = float(rospy.get_param("~beep_volume", 0.35))
        self.playback_device = rospy.get_param("~playback_device", "plughw:2,0")
        self.beep_path = self.create_beep_wav()

        self.pub_audio = rospy.Publisher("/voice/audio_file", String, queue_size=10)
        self.pub_stt_text = rospy.Publisher("/stt_text", String, queue_size=10)
        self.pub_tts = rospy.Publisher("/tts_input", String, queue_size=10)
        self.pub_wake_text = rospy.Publisher("/wake_text", String, queue_size=10)

    def run(self):
        if self.wake_engine == "picovoice":
            rospy.logwarn("Picovoice mode is configured, but this node needs a .ppn and pvporcupine/pyaudio installed.")
            rospy.logwarn("Falling back to cloud wake detection until Picovoice credentials are configured.")
        self.run_cloud_wake_loop()

    def run_cloud_wake_loop(self):
        rospy.loginfo("Wake: cloud wake detection active for phrase '%s'", self.wake_phrase)
        rospy.logwarn("Wake: cloud mode sends short audio chunks to OpenAI. Use Picovoice later for offline always-on wake word.")
        while not rospy.is_shutdown():
            wake_path = None
            try:
                wake_path = self.record_fixed_wav(self.chunk_seconds, "dingo_wake_")
                rms = self.wav_rms(wake_path)
                if rms < self.wake_min_rms:
                    rospy.loginfo("Wake: skipped quiet chunk rms=%s", rms)
                    continue
                text = transcribe_audio(wake_path, model=self.stt_model, language=self.language or None, prompt=self.stt_prompt or None)
                rospy.loginfo("Wake: heard chunk: %s", text)
                self.pub_wake_text.publish(text)
                if self.is_wake_phrase(text):
                    inline_command = self.extract_inline_command(text)
                    if inline_command:
                        rospy.loginfo("Wake: inline command after wake phrase: %s", inline_command)
                        self.pub_stt_text.publish(inline_command)
                        rospy.sleep(self.wake_cooldown_seconds)
                    else:
                        self.handle_wake()
            except Exception as exc:
                rospy.logerr("Wake: loop failed: %s", exc)
                time.sleep(1.0)
            finally:
                if wake_path:
                    try:
                        os.remove(wake_path)
                    except OSError:
                        pass


    def is_wake_phrase(self, text):
        heard = compact_text(text)
        for alias in self.wake_aliases:
            if compact_text(alias) in heard:
                return True
        return False


    def extract_inline_command(self, text):
        normalized = normalize_text(text)
        for alias in sorted(self.wake_aliases, key=len, reverse=True):
            alias_text = normalize_text(alias)
            index = normalized.find(alias_text)
            if index >= 0:
                command = normalized[index + len(alias_text) :].strip()
                if len(command) >= 3:
                    return command
        return ""

    def wav_rms(self, path):
        with wave.open(path, "rb") as wav_file:
            data = wav_file.readframes(wav_file.getnframes())
            width = wav_file.getsampwidth()
        if not data:
            return 0
        return audioop.rms(data, width)

    def handle_wake(self):
        rospy.loginfo("Wake: detected '%s'", self.wake_phrase)
        self.play_wake_feedback()
        rospy.sleep(self.command_start_delay)
        command_path = self.record_until_silence()
        rospy.loginfo("Wake: command audio saved: %s", command_path)
        self.pub_audio.publish(command_path)
        rospy.sleep(self.wake_cooldown_seconds)

    def play_wake_feedback(self):
        if self.feedback_mode == "none":
            return
        if self.feedback_mode == "tts":
            self.pub_tts.publish("Hi, what can I help you?")
            return
        if self.feedback_mode != "beep":
            rospy.logwarn("Wake: unknown wake_feedback_mode '%s'; using beep", self.feedback_mode)
        cmd = ["aplay", "-q"]
        if self.playback_device:
            cmd.extend(["-D", self.playback_device])
        cmd.append(self.beep_path)
        try:
            subprocess.check_call(cmd)
        except Exception as exc:
            rospy.logerr("Wake: failed to play beep: %s", exc)

    def create_beep_wav(self):
        fd, path = tempfile.mkstemp(prefix="dingo_wake_beep_", suffix=".wav")
        os.close(fd)
        sample_rate = 24000
        samples = int(sample_rate * max(0.02, self.beep_duration))
        amplitude = int(32767 * max(0.0, min(1.0, self.beep_volume)))
        with wave.open(path, "wb") as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(sample_rate)
            frames = bytearray()
            for index in range(samples):
                value = int(amplitude * math.sin(2.0 * math.pi * self.beep_frequency * index / sample_rate))
                frames.extend(struct.pack("<h", value))
            wav_file.writeframes(bytes(frames))
        return path

    def record_fixed_wav(self, seconds, prefix):
        fd, path = tempfile.mkstemp(prefix=prefix, suffix=".wav")
        os.close(fd)
        cmd = [
            "arecord",
            "-q",
            "-D",
            self.audio_device,
            "-f",
            "S16_LE",
            "-r",
            str(self.sample_rate),
            "-c",
            str(self.channels),
            "-d",
            str(int(seconds)),
            path,
        ]
        try:
            subprocess.check_call(cmd)
        except FileNotFoundError:
            rospy.logerr("Wake: arecord is not installed in this container. Install alsa-utils or rebuild the image.")
            raise
        except subprocess.CalledProcessError as exc:
            rospy.logerr("Wake: arecord failed for device %s with exit code %s", self.audio_device, exc.returncode)
            raise
        return path

    def record_until_silence(self):
        fd, path = tempfile.mkstemp(prefix="dingo_command_", suffix=".wav")
        os.close(fd)

        cmd = [
            "arecord",
            "-q",
            "-D",
            self.audio_device,
            "-f",
            "S16_LE",
            "-r",
            str(self.sample_rate),
            "-c",
            str(self.channels),
            "-t",
            "raw",
        ]
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        bytes_per_sample = 2
        frame_count = int(self.sample_rate * 0.1)
        read_size = frame_count * bytes_per_sample * self.channels
        max_chunks = int(self.command_max_seconds / 0.1)
        silence_chunks_needed = int(self.silence_seconds / 0.1)
        min_chunks = int(self.min_command_seconds / 0.1)
        silent_chunks = 0
        chunks = []

        try:
            for index in range(max_chunks):
                data = process.stdout.read(read_size)
                if not data:
                    break
                chunks.append(data)
                rms = audioop.rms(data, bytes_per_sample)
                if rms < self.silence_rms and index >= min_chunks:
                    silent_chunks += 1
                else:
                    silent_chunks = 0
                if silent_chunks >= silence_chunks_needed:
                    break
        finally:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except Exception:
                process.kill()

        audio_data = b"".join(chunks)
        if len(audio_data) % bytes_per_sample:
            audio_data = audio_data[: -(len(audio_data) % bytes_per_sample)]
        if not audio_data:
            audio_data = struct.pack("<h", 0) * self.sample_rate

        with wave.open(path, "wb") as wav_file:
            wav_file.setnchannels(self.channels)
            wav_file.setsampwidth(bytes_per_sample)
            wav_file.setframerate(self.sample_rate)
            wav_file.writeframes(audio_data)
        return path


if __name__ == "__main__":
    rospy.init_node("wakeup_and_record_node")
    WakeupAndRecordNode().run()
