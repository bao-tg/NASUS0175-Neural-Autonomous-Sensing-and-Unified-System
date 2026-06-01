#!/usr/bin/env python3

import argparse
import os
import urllib.request


def main():
    parser = argparse.ArgumentParser(description="Download one JPEG from the CSI capture server")
    parser.add_argument("--url", default="http://127.0.0.1:8081/capture.jpg")
    parser.add_argument("--output", default="/tmp/dingo_scene_test.jpg")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()

    with urllib.request.urlopen(args.url, timeout=args.timeout) as response:
        data = response.read()
    with open(args.output, "wb") as output_file:
        output_file.write(data)
    print("wrote %s (%d bytes)" % (args.output, os.path.getsize(args.output)))


if __name__ == "__main__":
    main()
