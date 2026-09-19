import unittest
from types import SimpleNamespace

import cv2
import numpy as np

from llmdecision_ros.action_server import PushabilityActionServer


class PushabilityActionServerImageTests(unittest.TestCase):
    @staticmethod
    def _roi(x, y, width, height):
        return SimpleNamespace(
            x_offset=x,
            y_offset=y,
            width=width,
            height=height,
        )

    def test_crop_marks_intersection_in_crop_coordinates(self):
        image = np.zeros((80, 100, 3), dtype=np.uint8)
        success, encoded = cv2.imencode(".png", image)
        self.assertTrue(success)

        crop = PushabilityActionServer._crop_and_mark(
            encoded.tobytes(),
            self._roi(10, 10, 60, 50),
            self._roi(0, 20, 30, 20),
        )

        self.assertEqual(crop.shape, (50, 60, 3))
        # The full-image target intersects the crop at x=[10, 30), y=[20, 40).
        self.assertGreater(int(crop[10, 0, 1]), 200)
        self.assertGreater(int(crop[29, 19, 1]), 200)
        self.assertEqual(int(crop[40, 40].max()), 0)

    def test_non_overlapping_target_fails_closed(self):
        image = np.zeros((80, 100, 3), dtype=np.uint8)
        success, encoded = cv2.imencode(".png", image)
        self.assertTrue(success)
        with self.assertRaisesRegex(ValueError, "does not overlap"):
            PushabilityActionServer._crop_and_mark(
                encoded.tobytes(),
                self._roi(10, 10, 40, 40),
                self._roi(80, 60, 10, 10),
            )


if __name__ == "__main__":
    unittest.main()
