#!/usr/bin/env python3
import os
import struct

import rospy
from sensor_msgs.msg import Image


class ImagePlotNode:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.plot_frequency = max(1, int(rospy.get_param("~plot_frequency", 100)))
        self.counter = 0

        package_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        self.output_dir = rospy.get_param("~output_dir", os.path.join(package_dir, "image"))
        os.makedirs(self.output_dir, exist_ok=True)

        self.subscriber = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        rospy.loginfo(
            "image_plot subscribed to %s; saving every %d frames into %s",
            self.image_topic,
            self.plot_frequency,
            self.output_dir,
        )

    def image_callback(self, msg):
        self.counter += 1
        if self.counter % self.plot_frequency != 0:
            return

        try:
            path = os.path.join(self.output_dir, "frame_%06d.ppm" % self.counter)
            self.save_ppm(msg, path)
            rospy.loginfo("image_plot saved %s", path)
        except Exception as exc:
            rospy.logwarn("image_plot failed to save frame %d: %s", self.counter, exc)

    @staticmethod
    def save_ppm(msg, path):
        if msg.encoding not in ("rgb8", "bgr8", "mono8"):
            raise ValueError("unsupported encoding: %s" % msg.encoding)

        width = msg.width
        height = msg.height
        data = bytes(msg.data)

        if msg.encoding == "mono8":
            expected_step = width
            rows = []
            for y in range(height):
                start = y * msg.step
                row = data[start:start + expected_step]
                rows.append(b"".join(bytes((pixel, pixel, pixel)) for pixel in row))
            rgb_data = b"".join(rows)
        else:
            expected_step = width * 3
            rows = []
            for y in range(height):
                start = y * msg.step
                row = data[start:start + expected_step]
                if msg.encoding == "bgr8":
                    row = b"".join(row[i + 2:i + 3] + row[i + 1:i + 2] + row[i:i + 1] for i in range(0, len(row), 3))
                rows.append(row)
            rgb_data = b"".join(rows)

        with open(path, "wb") as handle:
            handle.write(("P6\n%d %d\n255\n" % (width, height)).encode("ascii"))
            handle.write(rgb_data)


def main():
    rospy.init_node("image_plot")
    ImagePlotNode()
    rospy.spin()


if __name__ == "__main__":
    main()
