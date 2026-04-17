#!/usr/bin/env python3
"""
vision_node.py (ROS1)
- Subscribes to a camera topic (default: /usb_cam/image_raw)
- Detects Connect4 cell states using HSV color masks:
    0 = empty, 1 = PURPLE, 2 = BLUE
- Publishes std_msgs/Int8MultiArray on /board_state (flattened 6x7)
- Shows an overlay on the live camera feed with per-cell state labels
Press 'q' in the OpenCV window to quit.
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from std_msgs.msg import Int8MultiArray, MultiArrayDimension

ROWS = 6
COLS = 7


class VisionNode:
    def __init__(self):
        rospy.init_node("vision_node", anonymous=True)

        # Topics
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.pub = rospy.Publisher("/board_state", Int8MultiArray, queue_size=1)
        self.sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)

        # cv_bridge
        self.bridge = CvBridge()

        # Detection tuning
        self.px_thresh = int(rospy.get_param("~px_thresh", 80))  # raise if false positives
        self.dominance_margin = int(rospy.get_param("~dominance_margin", 10))  # reduces ties/flicker

        # Sample only the center region of each cell (more robust)
        self.use_center_crop = bool(rospy.get_param("~center_crop", True))
        self.center_crop_ratio = float(rospy.get_param("~center_crop_ratio", 0.55))  # 0..1

        # Overlay tuning
        self.alpha_fill = float(rospy.get_param("~alpha_fill", 0.30))
        self.show_rc = bool(rospy.get_param("~show_rc", False))

        # HSV thresholds (OpenCV HSV: H 0..179, S 0..255, V 0..255)
        # PURPLE
        p_h_min = int(rospy.get_param("~purple_h_min", 130))
        p_h_max = int(rospy.get_param("~purple_h_max", 165))
        p_s_min = int(rospy.get_param("~purple_s_min", 80))
        p_v_min = int(rospy.get_param("~purple_v_min", 80))
        self.lower_purple = np.array([p_h_min, p_s_min, p_v_min])
        self.upper_purple = np.array([p_h_max, 255, 255])

        # BLUE
        b_h_min = int(rospy.get_param("~blue_h_min", 95))
        b_h_max = int(rospy.get_param("~blue_h_max", 130))
        b_s_min = int(rospy.get_param("~blue_s_min", 80))
        b_v_min = int(rospy.get_param("~blue_v_min", 80))
        self.lower_blue = np.array([b_h_min, b_s_min, b_v_min])
        self.upper_blue = np.array([b_h_max, 255, 255])

        # Small kernel for cleaning masks
        self.kernel = np.ones((3, 3), np.uint8)

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo(f"[vision_node] Subscribed to: {self.image_topic}")
        rospy.loginfo("[vision_node] Publishing: /board_state (0 empty, 1 purple, 2 blue)")
        rospy.loginfo("[vision_node] Press 'q' in the window to quit.")

    def on_shutdown(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"CV Bridge error: {e}")
            return

        board, overlay_frame = self.detect_board_and_overlay(frame)

        # Publish board as Int8MultiArray (flatten row-major)
        msg_out = Int8MultiArray()

        # Optional: include dimensions so subscribers know it's 6x7
        msg_out.layout.dim = [
            MultiArrayDimension(label="rows", size=ROWS, stride=ROWS * COLS),
            MultiArrayDimension(label="cols", size=COLS, stride=COLS),
        ]
        msg_out.layout.data_offset = 0

        msg_out.data = board.flatten().astype(np.int8).tolist()
        self.pub.publish(msg_out)

        # Display overlay
        cv2.imshow("Connect4 Overlay (P=1, B=2, .=0)", overlay_frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            rospy.signal_shutdown("User pressed q")

    def detect_board_and_overlay(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h, w, _ = frame.shape
        cell_h = h // ROWS
        cell_w = w // COLS

        board = np.zeros((ROWS, COLS), dtype=np.int8)

        out = frame.copy()
        overlay = frame.copy()

        # Center crop region inside each cell
        if self.use_center_crop:
            crop_h = max(1, int(cell_h * self.center_crop_ratio))
            crop_w = max(1, int(cell_w * self.center_crop_ratio))
            dh = (cell_h - crop_h) // 2
            dw = (cell_w - crop_w) // 2
        else:
            crop_h, crop_w, dh, dw = cell_h, cell_w, 0, 0

        for r in range(ROWS):
            for c in range(COLS):
                y1, y2 = r * cell_h, (r + 1) * cell_h
                x1, x2 = c * cell_w, (c + 1) * cell_w

                yy1 = y1 + dh
                yy2 = min(y2 - dh, y1 + dh + crop_h)
                xx1 = x1 + dw
                xx2 = min(x2 - dw, x1 + dw + crop_w)

                roi = hsv[yy1:yy2, xx1:xx2]

                # Masks
                mask_purple = cv2.inRange(roi, self.lower_purple, self.upper_purple)
                mask_blue = cv2.inRange(roi, self.lower_blue, self.upper_blue)

                # Clean masks a bit (reduces speckles)
                mask_purple = cv2.morphologyEx(mask_purple, cv2.MORPH_OPEN, self.kernel, iterations=1)
                mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, self.kernel, iterations=1)

                purple_px = cv2.countNonZero(mask_purple)
                blue_px = cv2.countNonZero(mask_blue)

                # Decide state: 1=purple, 2=blue
                if purple_px > self.px_thresh and purple_px > (blue_px + self.dominance_margin):
                    state = 1
                    fill_bgr = (255, 0, 255)  # purple/magenta (BGR)
                    text = "P"
                elif blue_px > self.px_thresh and blue_px > (purple_px + self.dominance_margin):
                    state = 2
                    fill_bgr = (255, 0, 0)    # blue (BGR)
                    text = "B"
                else:
                    state = 0
                    fill_bgr = (70, 70, 70)   # subtle gray for empty
                    text = "."

                board[r, c] = state

                # Semi-transparent fill per cell
                cv2.rectangle(overlay, (x1, y1), (x2, y2), fill_bgr, -1)

                # Grid lines and label
                cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 1)
                cx, cy = x1 + cell_w // 2, y1 + cell_h // 2
                cv2.putText(out, text, (cx - 10, cy + 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

                if self.show_rc:
                    cv2.putText(out, f"{r},{c}", (x1 + 4, y1 + 16),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        # Blend overlay onto out
        out = cv2.addWeighted(overlay, self.alpha_fill, out, 1.0 - self.alpha_fill, 0)

        # Legend
        cv2.putText(out, "Overlay: P=1 (purple), B=2 (blue), .=0 (empty)   press q to quit",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        return board, out


if __name__ == "__main__":
    try:
        node = VisionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
