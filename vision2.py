#!/usr/bin/env python3
"""
vision_node.py (ROS1)

Manual 4-corner calibration version:
- Click the 4 outer corners of the board in the camera window
- The node uses those 4 corners as the board quadrilateral
- Warps the board to a flat view
- Detects 6x7 slot states:
    0 = empty, 1 = RED, 2 = BLUE
- Publishes std_msgs/Int8MultiArray on /board_state
- Projects the grid + slot overlays back onto the board

Controls:
- Left click: add corner point
- r: reset calibration
- q: quit
"""

import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int8MultiArray, MultiArrayDimension

ROWS = 6
COLS = 7
WINDOW_NAME = "Connect4 Overlay (R=1, B=2, .=0)"


class VisionNode:
    def __init__(self):
        rospy.init_node("vision_node", anonymous=True)

        # Topic name
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")

        # Bridge
        self.bridge = CvBridge()

        # Detection tuning
        self.px_thresh = int(rospy.get_param("~px_thresh", 45))
        self.dominance_margin = int(rospy.get_param("~dominance_margin", 8))

        # Warped board resolution
        self.warp_cell_px = int(rospy.get_param("~warp_cell_px", 100))

        # Circular slot sampling
        self.slot_radius_ratio = float(rospy.get_param("~slot_radius_ratio", 0.28))
        self.circle_pts = int(rospy.get_param("~circle_pts", 24))

        # Overlay
        self.alpha_fill = float(rospy.get_param("~alpha_fill", 0.35))
        self.show_rc = bool(rospy.get_param("~show_rc", False))

        # HSV thresholds (OpenCV HSV: H 0..179)
        # RED (two ranges)
        r1_h_min = int(rospy.get_param("~red1_h_min", 0))
        r1_h_max = int(rospy.get_param("~red1_h_max", 10))
        r2_h_min = int(rospy.get_param("~red2_h_min", 170))
        r2_h_max = int(rospy.get_param("~red2_h_max", 179))
        r_s_min = int(rospy.get_param("~red_s_min", 80))
        r_v_min = int(rospy.get_param("~red_v_min", 80))

        self.lower_red1 = np.array([r1_h_min, r_s_min, r_v_min], dtype=np.uint8)
        self.upper_red1 = np.array([r1_h_max, 255, 255], dtype=np.uint8)
        self.lower_red2 = np.array([r2_h_min, r_s_min, r_v_min], dtype=np.uint8)
        self.upper_red2 = np.array([r2_h_max, 255, 255], dtype=np.uint8)

        # BLUE
        b_h_min = int(rospy.get_param("~blue_h_min", 95))
        b_h_max = int(rospy.get_param("~blue_h_max", 130))
        b_s_min = int(rospy.get_param("~blue_s_min", 80))
        b_v_min = int(rospy.get_param("~blue_v_min", 80))

        self.lower_blue = np.array([b_h_min, b_s_min, b_v_min], dtype=np.uint8)
        self.upper_blue = np.array([b_h_max, 255, 255], dtype=np.uint8)

        # Morphology kernel
        self.kernel = np.ones((3, 3), np.uint8)

        # Thread-safe shared state
        self.frame_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.latest_overlay = None

        # Manual calibration state
        self.click_points = []
        self.board_quad = self.load_corners_from_param()

        # Publisher
        self.pub = rospy.Publisher("/board_state", Int8MultiArray, queue_size=1)

        # UI (created in main thread)
        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, self.on_mouse)

        rospy.on_shutdown(self.on_shutdown)

        # Subscriber LAST, after all callback-used attributes exist
        self.sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)

        rospy.loginfo(f"[vision_node] Subscribed to: {self.image_topic}")
        rospy.loginfo("[vision_node] Publishing: /board_state (0 empty, 1 red, 2 blue)")
        if self.board_quad is None:
            rospy.loginfo("[vision_node] Click the 4 board corners to calibrate.")
        else:
            rospy.loginfo("[vision_node] Loaded board corners from ~board_corners.")
        rospy.loginfo("[vision_node] Press 'r' to recalibrate, 'q' to quit.")

    def on_shutdown(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    def load_corners_from_param(self):
        """
        Optional ROS param:
        ~board_corners:
          - [x1, y1]
          - [x2, y2]
          - [x3, y3]
          - [x4, y4]
        or flat list [x1,y1,x2,y2,x3,y3,x4,y4]
        """
        corners = rospy.get_param("~board_corners", None)
        if corners is None:
            return None

        pts = None
        try:
            arr = np.array(corners, dtype=np.float32)
            if arr.shape == (4, 2):
                pts = arr
            elif arr.shape == (8,):
                pts = arr.reshape(4, 2)
        except Exception:
            pts = None

        if pts is None:
            rospy.logwarn("[vision_node] Invalid ~board_corners format. Ignoring.")
            return None

        return self.order_points(pts)

    def on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        with self.state_lock:
            if self.board_quad is not None:
                return

            if len(self.click_points) < 4:
                self.click_points.append([x, y])
                rospy.loginfo(f"[vision_node] Corner {len(self.click_points)}: ({x}, {y})")

            if len(self.click_points) == 4:
                pts = np.array(self.click_points, dtype=np.float32)
                self.board_quad = self.order_points(pts)
                flat = self.board_quad.reshape(-1).astype(int).tolist()
                rospy.loginfo("[vision_node] Calibration complete.")
                rospy.loginfo(f"[vision_node] Save these corners as ~board_corners: {flat}")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"CV Bridge error: {e}")
            return

        board, overlay_frame = self.detect_board_and_overlay(frame)

        msg_out = Int8MultiArray()
        msg_out.layout.dim = [
            MultiArrayDimension(label="rows", size=ROWS, stride=ROWS * COLS),
            MultiArrayDimension(label="cols", size=COLS, stride=COLS),
        ]
        msg_out.layout.data_offset = 0
        msg_out.data = board.flatten().astype(np.int8).tolist()
        self.pub.publish(msg_out)

        # Store latest frame for UI thread
        with self.frame_lock:
            self.latest_overlay = overlay_frame.copy()

    def run_ui(self):
        rate = rospy.Rate(60)

        while not rospy.is_shutdown():
            frame_to_show = None

            with self.frame_lock:
                if self.latest_overlay is not None:
                    frame_to_show = self.latest_overlay.copy()

            if frame_to_show is not None:
                cv2.imshow(WINDOW_NAME, frame_to_show)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                rospy.signal_shutdown("User pressed q")
                break
            elif key == ord("r"):
                with self.state_lock:
                    self.board_quad = None
                    self.click_points = []
                rospy.loginfo("[vision_node] Calibration reset. Click 4 corners again.")

            rate.sleep()

    def order_points(self, pts):
        """Return points as top-left, top-right, bottom-right, bottom-left."""
        pts = np.asarray(pts, dtype=np.float32)
        rect = np.zeros((4, 2), dtype=np.float32)

        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]   # top-left
        rect[2] = pts[np.argmax(s)]   # bottom-right

        diff = np.diff(pts, axis=1).reshape(-1)
        rect[1] = pts[np.argmin(diff)]  # top-right
        rect[3] = pts[np.argmax(diff)]  # bottom-left

        return rect

    def make_circle_polygon(self, cx, cy, radius, n_pts):
        angles = np.linspace(0, 2.0 * np.pi, n_pts, endpoint=False)
        pts = np.stack(
            [cx + radius * np.cos(angles), cy + radius * np.sin(angles)],
            axis=1
        ).astype(np.float32)
        return pts

    def draw_calibration_view(self, frame):
        out = frame.copy()

        with self.state_lock:
            points = list(self.click_points)

        for i, p in enumerate(points):
            x, y = int(p[0]), int(p[1])
            cv2.circle(out, (x, y), 5, (0, 255, 255), -1)
            cv2.putText(out, str(i + 1), (x + 8, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        if len(points) >= 2:
            pts = np.array(points, dtype=np.int32)
            cv2.polylines(out, [pts], False, (0, 255, 255), 2)

        cv2.putText(out, "Click 4 board corners", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(out, "Suggested: TL, TR, BR, BL", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(out, "Press r to reset, q to quit", (10, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return out

    def detect_cells_on_warped(self, warped_bgr):
        hsv = cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2HSV)
        warp_h, warp_w = warped_bgr.shape[:2]

        cell_h = warp_h // ROWS
        cell_w = warp_w // COLS

        board = np.zeros((ROWS, COLS), dtype=np.int8)
        cell_visuals = []

        for r in range(ROWS):
            for c in range(COLS):
                y1 = r * cell_h
                y2 = (r + 1) * cell_h
                x1 = c * cell_w
                x2 = (c + 1) * cell_w

                roi = hsv[y1:y2, x1:x2]
                roi_h, roi_w = roi.shape[:2]

                circle_mask = np.zeros((roi_h, roi_w), dtype=np.uint8)
                radius = max(1, int(min(roi_w, roi_h) * self.slot_radius_ratio))
                cx_local = roi_w // 2
                cy_local = roi_h // 2
                cv2.circle(circle_mask, (cx_local, cy_local), radius, 255, -1)

                mask_red1 = cv2.inRange(roi, self.lower_red1, self.upper_red1)
                mask_red2 = cv2.inRange(roi, self.lower_red2, self.upper_red2)
                mask_red = cv2.bitwise_or(mask_red1, mask_red2)
                mask_blue = cv2.inRange(roi, self.lower_blue, self.upper_blue)

                mask_red = cv2.bitwise_and(mask_red, circle_mask)
                mask_blue = cv2.bitwise_and(mask_blue, circle_mask)

                mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, self.kernel, iterations=1)
                mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, self.kernel, iterations=1)

                red_px = cv2.countNonZero(mask_red)
                blue_px = cv2.countNonZero(mask_blue)

                if red_px > self.px_thresh and red_px > (blue_px + self.dominance_margin):
                    state = 1
                    fill_bgr = (0, 0, 255)
                    text = "R"
                elif blue_px > self.px_thresh and blue_px > (red_px + self.dominance_margin):
                    state = 2
                    fill_bgr = (255, 0, 0)
                    text = "B"
                else:
                    state = 0
                    fill_bgr = (70, 70, 70)
                    text = "."

                board[r, c] = state

                cell_poly = np.array(
                    [
                        [x1, y1],
                        [x2 - 1, y1],
                        [x2 - 1, y2 - 1],
                        [x1, y2 - 1],
                    ],
                    dtype=np.float32
                )

                cx_global = x1 + (cell_w / 2.0)
                cy_global = y1 + (cell_h / 2.0)
                slot_poly = self.make_circle_polygon(cx_global, cy_global, radius, self.circle_pts)

                cell_visuals.append(
                    {
                        "r": r,
                        "c": c,
                        "cell_poly": cell_poly,
                        "slot_poly": slot_poly,
                        "center": np.array([[cx_global, cy_global]], dtype=np.float32),
                        "fill_bgr": fill_bgr,
                        "text": text,
                    }
                )

        return board, cell_visuals

    def detect_board_and_overlay(self, frame):
        with self.state_lock:
            quad = None if self.board_quad is None else self.board_quad.copy()

        if quad is None:
            board = np.zeros((ROWS, COLS), dtype=np.int8)
            return board, self.draw_calibration_view(frame)

        warp_w = COLS * self.warp_cell_px
        warp_h = ROWS * self.warp_cell_px

        dst_quad = np.array(
            [
                [0, 0],
                [warp_w - 1, 0],
                [warp_w - 1, warp_h - 1],
                [0, warp_h - 1],
            ],
            dtype=np.float32
        )

        M = cv2.getPerspectiveTransform(quad, dst_quad)
        M_inv = cv2.getPerspectiveTransform(dst_quad, quad)

        warped = cv2.warpPerspective(frame, M, (warp_w, warp_h))
        board, cell_visuals = self.detect_cells_on_warped(warped)

        overlay = frame.copy()
        projected_visuals = []

        for cell in cell_visuals:
            warped_cell_poly = np.array([cell["cell_poly"]], dtype=np.float32)
            warped_slot_poly = np.array([cell["slot_poly"]], dtype=np.float32)
            warped_center = np.array([cell["center"]], dtype=np.float32)

            img_cell_poly = cv2.perspectiveTransform(warped_cell_poly, M_inv)[0]
            img_slot_poly = cv2.perspectiveTransform(warped_slot_poly, M_inv)[0]
            img_center = cv2.perspectiveTransform(warped_center, M_inv)[0][0]

            img_cell_poly_int = np.round(img_cell_poly).astype(np.int32)
            img_slot_poly_int = np.round(img_slot_poly).astype(np.int32)
            img_center_int = tuple(np.round(img_center).astype(np.int32))

            cv2.fillConvexPoly(overlay, img_slot_poly_int, cell["fill_bgr"])

            projected_visuals.append(
                {
                    "r": cell["r"],
                    "c": cell["c"],
                    "cell_poly": img_cell_poly_int,
                    "slot_poly": img_slot_poly_int,
                    "center": img_center_int,
                    "text": cell["text"],
                }
            )

        out = cv2.addWeighted(overlay, self.alpha_fill, frame, 1.0 - self.alpha_fill, 0)

        cv2.polylines(out, [np.round(quad).astype(np.int32)], True, (0, 255, 255), 2)

        for vis in projected_visuals:
            cv2.polylines(out, [vis["cell_poly"]], True, (0, 255, 0), 1)
            cv2.polylines(out, [vis["slot_poly"]], True, (255, 255, 255), 1)

            cx, cy = vis["center"]
            cv2.putText(out, vis["text"], (cx - 10, cy + 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            if self.show_rc:
                anchor = vis["cell_poly"][0]
                cv2.putText(out, f"{vis['r']},{vis['c']}",
                            (int(anchor[0]) + 4, int(anchor[1]) + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        cv2.putText(out, "Overlay: R=1 (red), B=2 (blue), .=0 (empty)",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(out, "Press r to recalibrate, q to quit",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        return board, out


if __name__ == "__main__":
    try:
        node = VisionNode()
        node.run_ui()
    except rospy.ROSInterruptException:
        pass
