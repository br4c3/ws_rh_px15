import sys
import unittest
from pathlib import Path


BRIDGE_DIRECTORY = Path(__file__).resolve().parents[1] / "bridge"
sys.path.insert(0, str(BRIDGE_DIRECTORY))

from udp_camera_bridge import JpegStreamParser, gstreamer_pipeline


class UdpCameraBridgeTests(unittest.TestCase):
    def test_parses_jpeg_frames_split_across_chunks(self):
        parser = JpegStreamParser()
        first = b"\xff\xd8first\xff\xd9"
        second = b"\xff\xd8second\xff\xd9"

        self.assertEqual(parser.feed(b"noise" + first[:5]), [])
        self.assertEqual(parser.feed(first[5:] + second), [first, second])

    def test_builds_expected_qgc_rtp_pipeline(self):
        pipeline = gstreamer_pipeline(
            {
                "bind_address": "0.0.0.0",
                "port": 5600,
                "width": 1280,
                "height": 720,
                "fps": 15,
                "jpeg_quality": 75,
                "latency": 80,
            }
        )

        self.assertIn("port=5600", pipeline)
        self.assertIn("encoding-name=H264,payload=96", " ".join(pipeline))
        self.assertIn("video/x-raw,framerate=15/1", pipeline)
        self.assertIn("video/x-raw,width=1280,height=720,pixel-aspect-ratio=1/1", pipeline)


if __name__ == "__main__":
    unittest.main()
