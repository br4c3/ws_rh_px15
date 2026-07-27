#!/usr/bin/env python3
"""Decode the QGC RTP/H.264 UDP feed and emit length-prefixed JPEG frames."""

import os
import signal
import struct
import subprocess
import sys
import threading


JPEG_START = b"\xff\xd8"
JPEG_END = b"\xff\xd9"
MAX_BUFFER_SIZE = 16 * 1024 * 1024


class JpegStreamParser:
    """Split the concatenated JPEG byte stream produced by GStreamer."""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, chunk):
        if chunk:
            self.buffer.extend(chunk)

        frames = []
        while True:
            start = self.buffer.find(JPEG_START)
            if start < 0:
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                break
            if start:
                del self.buffer[:start]

            end = self.buffer.find(JPEG_END, len(JPEG_START))
            if end < 0:
                if len(self.buffer) > MAX_BUFFER_SIZE:
                    self.buffer.clear()
                break

            end += len(JPEG_END)
            frames.append(bytes(self.buffer[:end]))
            del self.buffer[:end]

        return frames


def integer_setting(name, fallback, minimum, maximum):
    raw_value = os.environ.get(name, str(fallback))
    try:
        value = int(raw_value)
    except ValueError as error:
        raise ValueError(f"{name} must be an integer: {raw_value}") from error
    if value < minimum or value > maximum:
        raise ValueError(
            f"{name} must be between {minimum} and {maximum}: {value}"
        )
    return value


def settings():
    return {
        "bind_address": os.environ.get(
            "QCS_VIDEO_BIND_ADDRESS",
            "0.0.0.0",
        ),
        "port": integer_setting("QCS_VIDEO_PORT", 5600, 1, 65535),
        "width": integer_setting("QCS_VIDEO_WIDTH", 1280, 16, 7680),
        "height": integer_setting("QCS_VIDEO_HEIGHT", 720, 16, 4320),
        "fps": integer_setting("QCS_VIDEO_FPS", 15, 1, 120),
        "jpeg_quality": integer_setting(
            "QCS_VIDEO_JPEG_QUALITY",
            75,
            20,
            95,
        ),
        "latency": integer_setting(
            "QCS_VIDEO_LATENCY_MS",
            80,
            0,
            2000,
        ),
    }


def gstreamer_pipeline(configuration):
    return [
        "gst-launch-1.0",
        "-q",
        "udpsrc",
        f"address={configuration['bind_address']}",
        f"port={configuration['port']}",
        "reuse=false",
        (
            "caps=application/x-rtp,media=video,clock-rate=90000,"
            "encoding-name=H264,payload=96"
        ),
        "!",
        "rtpjitterbuffer",
        f"latency={configuration['latency']}",
        "drop-on-latency=true",
        "!",
        "rtph264depay",
        "!",
        "h264parse",
        "!",
        "avdec_h264",
        "!",
        "queue",
        "max-size-buffers=1",
        "max-size-bytes=0",
        "max-size-time=0",
        "leaky=downstream",
        "!",
        "videorate",
        "drop-only=true",
        "!",
        f"video/x-raw,framerate={configuration['fps']}/1",
        "!",
        "videoscale",
        "!",
        (
            f"video/x-raw,width={configuration['width']},"
            f"height={configuration['height']},pixel-aspect-ratio=1/1"
        ),
        "!",
        "videoconvert",
        "!",
        "jpegenc",
        f"quality={configuration['jpeg_quality']}",
        "!",
        "fdsink",
        "fd=1",
        "sync=false",
    ]


def forward_stderr(pipe):
    for line in iter(pipe.readline, b""):
        sys.stderr.write(f"[GStreamer] {line.decode(errors='replace')}")
        sys.stderr.flush()


def emit_frame(frame):
    sys.stdout.buffer.write(struct.pack(">I", len(frame)))
    sys.stdout.buffer.write(frame)
    sys.stdout.buffer.flush()


def run():
    try:
        configuration = settings()
    except ValueError as error:
        sys.stderr.write(f"Configuration error: {error}\n")
        return 2

    sys.stderr.write(
        "Waiting for RTP/H.264 video on "
        f"{configuration['bind_address']}:{configuration['port']} "
        f"({configuration['width']}x{configuration['height']} "
        f"@ {configuration['fps']} fps)\n"
    )
    sys.stderr.flush()

    try:
        process = subprocess.Popen(
            gstreamer_pipeline(configuration),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
    except FileNotFoundError:
        sys.stderr.write(
            "gst-launch-1.0 was not found. Install GStreamer tools and "
            "good/bad/ugly/libav plugins.\n"
        )
        return 127

    def stop_process(_signal_number, _frame):
        if process.poll() is None:
            process.terminate()

    signal.signal(signal.SIGTERM, stop_process)
    signal.signal(signal.SIGINT, stop_process)

    stderr_thread = threading.Thread(
        target=forward_stderr,
        args=(process.stderr,),
        daemon=True,
    )
    stderr_thread.start()

    parser = JpegStreamParser()
    try:
        while process.poll() is None:
            chunk = process.stdout.read(64 * 1024)
            if not chunk:
                break
            for frame in parser.feed(chunk):
                emit_frame(frame)
    except (BrokenPipeError, KeyboardInterrupt):
        pass
    finally:
        if process.poll() is None:
            process.terminate()
        try:
            return process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            return process.wait()


if __name__ == "__main__":
    raise SystemExit(run())
