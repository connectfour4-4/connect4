#!/usr/bin/env python3
"""
vision_node.py (ROS1)

Improved Connect4 detector:
- Manual 4-corner board calibration
- Color-based token detection
- Shape scoring (filled-disc / circularity)
- Optional depth confirmation using aligned depth
- Temporal smoothing to reduce flicker

Controls:
- Left click: add corner point
- r: reset calibration
- e: relearn empty-slot depth reference from current frame
- q: quit
"""

import threading
from collections import deque

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
        rospy.init_node("visualize", anonymous=True)

        self.bridge = CvBridge()

        # Topics
        self.image_topic = rospy.get_param("~color_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )

        # Detection tuning
        self.px_thresh = int(rospy.get_param("~px_thresh", 80))
        self.dominance_margin = int(rospy.get_param("~dominance_margin", 18))
        self.min_color_ratio = float(rospy.get_param("~min_color_ratio", 0.15))
        self.min_mean_sat = float(rospy.get_param("~min_mean_sat", 85.0))
        self.min_mean_val = float(rospy.get_param("~min_mean_val", 65.0))

        # Shape tuning
        self.min_blob_area_ratio = float(rospy.get_param("~min_blob_area_ratio", 0.10))
        self.min_circularity = float(rospy.get_param("~min_circularity", 0.45))
        self.max_center_offset_ratio = float(rospy.get_param("~max_center_offset_ratio", 0.35))
        self.score_threshold = float(rospy.get_param("~score_threshold", 0.52))

        # Depth tuning
        self.use_depth = bool(rospy.get_param("~use_depth", True))
        self.depth_presence_margin_mm = float(
            rospy.get_param("~depth_presence_margin_mm", 14.0)
        )
        self.min_valid_depth_ratio = float(
            rospy.get_param("~min_valid_depth_ratio", 0.30)
        )
        self.require_depth_confirmation = bool(
            rospy.get_param("~require_depth_confirmation", False)
        )

        # Warped board resolution
        self.warp_cell_px = int(rospy.get_param("~warp_cell_px", 100))

        # Circular slot sampling
        self.slot_radius_ratio = float(rospy.get_param("~slot_radius_ratio", 0.26))
        self.inner_radius_ratio = float(rospy.get_param("~inner_radius_ratio", 0.18))
        self.circle_pts = int(rospy.get_param("~circle_pts", 24))

        # Overlay
        self.alpha_fill = float(rospy.get_param("~alpha_fill", 0.35))
        self.show_rc = bool(rospy.get_param("~show_rc", False))
        self.show_debug_scores = bool(rospy.get_param("~show_debug_scores", False))

        # Temporal smoothing
        self.history_len = int(rospy.get_param("~history_len", 4))

        # HSV thresholds
        r1_h_min = int(rospy.get_param("~red1_h_min", 0))
        r1_h_max = int(rospy.get_param("~red1_h_max", 10))
        r2_h_min = int(rospy.get_param("~red2_h_min", 170))
        r2_h_max = int(rospy.get_param("~red2_h_max", 179))
        r_s_min = int(rospy.get_param("~red_s_min", 100))
        r_v_min = int(rospy.get_param("~red_v_min", 90))

        self.lower_red1 = np.array([r1_h_min, r_s_min, r_v_min], dtype=np.uint8)
        self.upper_red1 = np.array([r1_h_max, 255, 255], dtype=np.uint8)
        self.lower_red2 = np.array([r2_h_min, r_s_min, r_v_min], dtype=np.uint8)
        self.upper_red2 = np.array([r2_h_max, 255, 255], dtype=np.uint8)

        b_h_min = int(rospy.get_param("~blue_h_min", 95))
        b_h_max = int(rospy.get_param("~blue_h_max", 130))
        b_s_min = int(rospy.get_param("~blue_s_min", 100))
        b_v_min = int(rospy.get_param("~blue_v_min", 80))

        self.lower_blue = np.array([b_h_min, b_s_min, b_v_min], dtype=np.uint8)
        self.upper_blue = np.array([b_h_max, 255, 255], dtype=np.uint8)

        self.kernel = np.ones((3, 3), np.uint8)

        # Shared state
        self.frame_lock = threading.Lock()
        self.state_lock = threading.Lock()

        self.latest_overlay = None
        self.latest_depth = None

        self.click_points = []
        self.board_quad = self.load_corners_from_param()

        # Empty-slot depth reference on warped board, mm
        self.empty_depth_ref = None
        self.need_empty_depth_relearn = True

        # Temporal history per cell
        self.state_history = [
            [deque(maxlen=self.history_len) for _ in range(COLS)] for _ in range(ROWS)
        ]

        self.pub = rospy.Publisher("/board_state", Int8MultiArray, queue_size=1)

        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, self.on_mouse)

        rospy.on_shutdown(self.on_shutdown)

        self.color_sub = rospy.Subscriber(
            self.image_topic, Image, self.image_callback, queue_size=1
        )
        self.depth_sub = rospy.Subscriber(
            self.depth_topic, Image, self.depth_callback, queue_size=1
        )

        rospy.loginfo(f"[vision_node] Color topic: {self.image_topic}")
        rospy.loginfo(f"[vision_node] Depth topic: {self.depth_topic}")
        rospy.loginfo("[vision_node] Publishing: /board_state (0 empty, 1 red, 2 blue)")
        if self.board_quad is None:
            rospy.loginfo("[vision_node] Click the 4 board corners to calibrate.")
        else:
            rospy.loginfo("[vision_node] Loaded board corners from ~board_corners.")
        rospy.loginfo("[vision_node] Press 'r' to recalibrate, 'e' to relearn empty depth, 'q' to quit.")

    def on_shutdown(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    def load_corners_from_param(self):
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
                self.need_empty_depth_relearn = True
                rospy.loginfo("[vision_node] Calibration complete.")
                rospy.loginfo(f"[vision_node] Save these corners as ~board_corners: {flat}")

    def depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.latest_depth = depth.copy()
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"[vision_node] Depth conversion failed: {e}")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"CV Bridge error: {e}")
            return

        depth_frame = None
        if self.use_depth and self.latest_depth is not None:
            depth_frame = self.latest_depth.copy()

        board, overlay_frame = self.detect_board_and_overlay(frame, depth_frame)

        msg_out = Int8MultiArray()
        msg_out.layout.dim = [
            MultiArrayDimension(label="rows", size=ROWS, stride=ROWS * COLS),
            MultiArrayDimension(label="cols", size=COLS, stride=COLS),
        ]
        msg_out.layout.data_offset = 0
        msg_out.data = board.flatten().astype(np.int8).tolist()
        self.pub.publish(msg_out)

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
                    self.empty_depth_ref = None
                    self.need_empty_depth_relearn = True
                rospy.loginfo("[vision_node] Calibration reset. Click 4 corners again.")
            elif key == ord("e"):
                self.need_empty_depth_relearn = True
                rospy.loginfo("[vision_node] Empty-depth relearn requested.")

            rate.sleep()

    def order_points(self, pts):
        pts = np.asarray(pts, dtype=np.float32)
        rect = np.zeros((4, 2), dtype=np.float32)

        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]

        diff = np.diff(pts, axis=1).reshape(-1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]

        return rect

    def make_circle_polygon(self, cx, cy, radius, n_pts):
        angles = np.linspace(0, 2.0 * np.pi, n_pts, endpoint=False)
        pts = np.stack(
            [cx + radius * np.cos(angles), cy + radius * np.sin(angles)],
            axis=1
        ).astype(np.float32)
        return pts

    def make_circle_mask(self, h, w, radius_ratio):
        mask = np.zeros((h, w), dtype=np.uint8)
        radius = max(1, int(min(w, h) * radius_ratio))
        cv2.circle(mask, (w // 2, h // 2), radius, 255, -1)
        return mask, radius

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
        cv2.putText(out, "r=reset  e=relearn empty depth  q=quit", (10, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return out

    def get_depth_mm(self, warped_depth):
        if warped_depth is None:
            return None

        depth = warped_depth.astype(np.float32)

        if np.nanmax(depth) < 20.0:
            depth = depth * 1000.0

        return depth

    def stable_vote(self, r, c, new_state):
        hist = self.state_history[r][c]
        hist.append(int(new_state))
        vals, counts = np.unique(np.array(hist, dtype=np.int32), return_counts=True)
        return int(vals[np.argmax(counts)])

    def analyze_color_blob(self, color_mask, circle_mask):
        masked = cv2.bitwise_and(color_mask, circle_mask)
        contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        roi_area = float(max(1, cv2.countNonZero(circle_mask)))
        roi_h, roi_w = color_mask.shape[:2]
        roi_center = np.array([roi_w / 2.0, roi_h / 2.0], dtype=np.float32)
        roi_radius = 0.5 * min(roi_w, roi_h)

        best = {
            "found": False,
            "area_px": 0.0,
            "area_ratio": 0.0,
            "circularity": 0.0,
            "center_offset_ratio": 999.0,
            "score": 0.0,
            "contour": None,
        }

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area <= 1.0:
                continue

            perimeter = cv2.arcLength(cnt, True)
            circularity = 0.0
            if perimeter > 1e-6:
                circularity = float(4.0 * np.pi * area / (perimeter * perimeter))

            M = cv2.moments(cnt)
            if M["m00"] <= 1e-6:
                continue

            cx = float(M["m10"] / M["m00"])
            cy = float(M["m01"] / M["m00"])
            center = np.array([cx, cy], dtype=np.float32)

            center_offset = float(np.linalg.norm(center - roi_center))
            center_offset_ratio = center_offset / max(1.0, roi_radius)

            area_ratio = area / roi_area

            # Weighted score
            score = (
                0.45 * np.clip(area_ratio / max(self.min_blob_area_ratio, 1e-6), 0.0, 1.5) +
                0.35 * np.clip(circularity / max(self.min_circularity, 1e-6), 0.0, 1.5) +
                0.20 * np.clip(1.0 - center_offset_ratio / max(self.max_center_offset_ratio, 1e-6), 0.0, 1.0)
            )

            if score > best["score"]:
                best = {
                    "found": True,
                    "area_px": float(area),
                    "area_ratio": float(area_ratio),
                    "circularity": float(circularity),
                    "center_offset_ratio": float(center_offset_ratio),
                    "score": float(score),
                    "contour": cnt,
                }

        return best

    def confirm_depth_presence(self, warped_depth_mm, y1, y2, x1, x2, inner_mask, r, c):
        if warped_depth_mm is None:
            return None, np.nan, np.nan

        roi_depth = warped_depth_mm[y1:y2, x1:x2]
        depth_samples = roi_depth[inner_mask > 0]
        depth_samples = depth_samples[np.isfinite(depth_samples)]
        depth_samples = depth_samples[depth_samples > 0]

        valid_ratio = float(depth_samples.size) / float(max(1, cv2.countNonZero(inner_mask)))
        if valid_ratio < self.min_valid_depth_ratio or depth_samples.size == 0:
            return False, np.nan, np.nan

        current_depth_mm = float(np.median(depth_samples))

        if self.empty_depth_ref is None and self.need_empty_depth_relearn:
            self.empty_depth_ref = np.zeros((ROWS, COLS), dtype=np.float32)

        if self.empty_depth_ref is not None and self.need_empty_depth_relearn:
            self.empty_depth_ref[r, c] = current_depth_mm

        empty_ref_mm = np.nan
        if self.empty_depth_ref is not None:
            empty_ref_mm = float(self.empty_depth_ref[r, c])

        if not np.isfinite(empty_ref_mm) or empty_ref_mm <= 0:
            return None, current_depth_mm, empty_ref_mm

        present = current_depth_mm < (empty_ref_mm - self.depth_presence_margin_mm)
        return present, current_depth_mm, empty_ref_mm

    def detect_cells_on_warped(self, warped_bgr, warped_depth_mm=None):
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

                roi_hsv = hsv[y1:y2, x1:x2]
                roi_h, roi_w = roi_hsv.shape[:2]

                circle_mask, radius = self.make_circle_mask(roi_h, roi_w, self.slot_radius_ratio)
                inner_mask, _ = self.make_circle_mask(roi_h, roi_w, self.inner_radius_ratio)

                mask_red1 = cv2.inRange(roi_hsv, self.lower_red1, self.upper_red1)
                mask_red2 = cv2.inRange(roi_hsv, self.lower_red2, self.upper_red2)
                mask_red = cv2.bitwise_or(mask_red1, mask_red2)
                mask_blue = cv2.inRange(roi_hsv, self.lower_blue, self.upper_blue)

                mask_red = cv2.bitwise_and(mask_red, circle_mask)
                mask_blue = cv2.bitwise_and(mask_blue, circle_mask)

                mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, self.kernel, iterations=1)
                mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, self.kernel, iterations=1)
                mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, self.kernel, iterations=1)
                mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, self.kernel, iterations=1)

                red_px = cv2.countNonZero(mask_red)
                blue_px = cv2.countNonZero(mask_blue)
                roi_area = max(1, cv2.countNonZero(circle_mask))
                red_ratio = red_px / float(roi_area)
                blue_ratio = blue_px / float(roi_area)

                sat = roi_hsv[:, :, 1]
                val = roi_hsv[:, :, 2]
                masked_sat = sat[circle_mask > 0]
                masked_val = val[circle_mask > 0]
                mean_sat = float(np.mean(masked_sat)) if masked_sat.size else 0.0
                mean_val = float(np.mean(masked_val)) if masked_val.size else 0.0

                red_blob = self.analyze_color_blob(mask_red, circle_mask)
                blue_blob = self.analyze_color_blob(mask_blue, circle_mask)

                red_color_ok = (
                    red_px > self.px_thresh and
                    red_ratio > self.min_color_ratio and
                    red_px > (blue_px + self.dominance_margin) and
                    mean_sat > self.min_mean_sat and
                    mean_val > self.min_mean_val
                )

                blue_color_ok = (
                    blue_px > self.px_thresh and
                    blue_ratio > self.min_color_ratio and
                    blue_px > (red_px + self.dominance_margin) and
                    mean_sat > self.min_mean_sat and
                    mean_val > self.min_mean_val
                )

                red_shape_ok = (
                    red_blob["found"] and
                    red_blob["area_ratio"] >= self.min_blob_area_ratio and
                    red_blob["circularity"] >= self.min_circularity and
                    red_blob["center_offset_ratio"] <= self.max_center_offset_ratio and
                    red_blob["score"] >= self.score_threshold
                )

                blue_shape_ok = (
                    blue_blob["found"] and
                    blue_blob["area_ratio"] >= self.min_blob_area_ratio and
                    blue_blob["circularity"] >= self.min_circularity and
                    blue_blob["center_offset_ratio"] <= self.max_center_offset_ratio and
                    blue_blob["score"] >= self.score_threshold
                )

                depth_present, current_depth_mm, empty_ref_mm = self.confirm_depth_presence(
                    warped_depth_mm, y1, y2, x1, x2, inner_mask, r, c
                )

                red_ok = red_color_ok and red_shape_ok
                blue_ok = blue_color_ok and blue_shape_ok

                if self.use_depth:
                    if self.require_depth_confirmation:
                        red_ok = red_ok and (depth_present is True)
                        blue_ok = blue_ok and (depth_present is True)
                    else:
                        # Depth is soft confirmation
                        if depth_present is False:
                            red_ok = False
                            blue_ok = False

                if red_ok and not blue_ok:
                    raw_state = 1
                elif blue_ok and not red_ok:
                    raw_state = 2
                elif red_ok and blue_ok:
                    # Tie-break by higher blob score and dominance
                    red_score = red_blob["score"] + 0.2 * (red_ratio - blue_ratio)
                    blue_score = blue_blob["score"] + 0.2 * (blue_ratio - red_ratio)
                    raw_state = 1 if red_score >= blue_score else 2
                else:
                    raw_state = 0

                state = self.stable_vote(r, c, raw_state)
                board[r, c] = state

                if state == 1:
                    fill_bgr = (0, 0, 255)
                    text = "R"
                elif state == 2:
                    fill_bgr = (255, 0, 0)
                    text = "B"
                else:
                    fill_bgr = (70, 70, 70)
                    text = "."

                debug_text = text
                if self.show_debug_scores:
                    if state == 1:
                        debug_text = f"R {red_blob['score']:.2f}"
                    elif state == 2:
                        debug_text = f"B {blue_blob['score']:.2f}"
                    else:
                        rs = red_blob["score"]
                        bs = blue_blob["score"]
                        debug_text = f". {rs:.2f}/{bs:.2f}"

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
                        "text": debug_text if not self.show_rc else f"{debug_text} {r},{c}",
                    }
                )

        if self.need_empty_depth_relearn and self.empty_depth_ref is not None:
            self.need_empty_depth_relearn = False
            rospy.loginfo("[vision_node] Empty-depth reference learned from current frame.")

        return board, cell_visuals

    def detect_board_and_overlay(self, frame, depth_frame=None):
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

        warped_depth_mm = None
        if self.use_depth and depth_frame is not None:
            try:
                warped_depth = cv2.warpPerspective(depth_frame, M, (warp_w, warp_h))
                warped_depth_mm = self.get_depth_mm(warped_depth)
            except Exception as e:
                rospy.logwarn_throttle(2.0, f"[vision_node] Depth warp failed: {e}")

        board, cell_visuals = self.detect_cells_on_warped(warped, warped_depth_mm)

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
            cv2.putText(out, vis["text"], (cx - 18, cy + 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 2)

        cv2.putText(out, "Overlay: R=1 (red), B=2 (blue), .=0 (empty)",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(out, "r=reset  e=relearn empty depth  q=quit",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        return board, out


if __name__ == "__main__":
    try:
        node = VisionNode()
        node.run_ui()
    except rospy.ROSInterruptException:
        pass
    