#!/usr/bin/env python3
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
        rospy.init_node("visualize_board", anonymous=True)

        self.bridge = CvBridge()

        self.image_topic = rospy.get_param("realsense_image", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")

        self.stable_time = float(rospy.get_param("~stable_time", 4.0))
        self.min_token_depth = float(rospy.get_param("~min_token_depth", 0.05))
        self.max_token_depth = float(rospy.get_param("~max_token_depth", 2.0))
        self.min_publish_gap = float(rospy.get_param("~min_publish_gap", 2.0))

        self.vote_frames = int(rospy.get_param("~vote_frames", 30))
        self.vote_required = int(rospy.get_param("~vote_required", 24))

        self.board_history = deque(maxlen=self.vote_frames)
        self.last_filtered_board = np.zeros((ROWS, COLS), dtype=np.int8)

        self.latest_depth = None
        self.last_candidate_board = None
        self.last_candidate_time = None
        self.last_published_board = None
        self.last_publish_time = None

        self.px_thresh = int(rospy.get_param("~px_thresh", 300)) # raise from 200 to 300
        self.dominance_margin = int(rospy.get_param("~dominance_margin", 110))
        self.min_ratio = float(rospy.get_param("~min_ratio", 0.15)) # from 0.10 to 0.15

        self.warp_cell_px = int(rospy.get_param("~warp_cell_px", 120))
        self.slot_radius_ratio = float(rospy.get_param("~slot_radius_ratio", 0.30))
        self.circle_pts = int(rospy.get_param("~circle_pts", 32))

        self.alpha_fill = float(rospy.get_param("~alpha_fill", 0.35))
        self.show_rc = bool(rospy.get_param("~show_rc", False))

        # This now only affects what is sent to /board_state.
        # Internal detection, voting, and gravity validation stay top-to-bottom.
        self.flip_board = bool(rospy.get_param("~flip_board", False))

        self.blur_kernel = int(rospy.get_param("~blur_kernel", 7))
        if self.blur_kernel % 2 == 0:
            self.blur_kernel += 1

        self.lower_red1 = np.array([
            int(rospy.get_param("~red1_h_min", 0)),
            int(rospy.get_param("~red_s_min", 150)),
            int(rospy.get_param("~red_v_min", 130))
        ], dtype=np.uint8)

        self.upper_red1 = np.array([
            int(rospy.get_param("~red1_h_max", 12)),
            255,
            255
        ], dtype=np.uint8)

        self.lower_red2 = np.array([
            int(rospy.get_param("~red2_h_min", 168)),
            int(rospy.get_param("~red_s_min", 100)),
            int(rospy.get_param("~red_v_min", 80))
        ], dtype=np.uint8)

        self.upper_red2 = np.array([
            int(rospy.get_param("~red2_h_max", 179)),
            255,
            255
        ], dtype=np.uint8)

        self.lower_blue = np.array([
            int(rospy.get_param("~blue_h_min", 90)),
            int(rospy.get_param("~blue_s_min", 200)), # from 120 to 200
            int(rospy.get_param("~blue_v_min", 50)) # FROM 50 TO 30
        ], dtype=np.uint8)

        self.upper_blue = np.array([
            int(rospy.get_param("~blue_h_max", 135)),
            255,
            255
        ], dtype=np.uint8)

        self.kernel = np.ones((5, 5), np.uint8)

        self.frame_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.latest_overlay = None

        self.click_points = []
        self.board_quad = self.load_corners_from_param()

        self.pub = rospy.Publisher("/board_state", Int8MultiArray, queue_size=1)

        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, self.on_mouse)

        rospy.on_shutdown(self.on_shutdown)

        self.sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        self.depth_sub = rospy.Subscriber(
            self.depth_topic,
            Image,
            self.depth_callback,
            queue_size=1,
            buff_size=2**24
        )

        rospy.loginfo(f"[vision_node] Subscribed to: {self.image_topic}")
        rospy.loginfo(f"[vision_node] Subscribed to depth: {self.depth_topic}")
        rospy.loginfo("[vision_node] Stable RealSense detection enabled.")

        if self.board_quad is None:
            rospy.loginfo("[vision_node] Click the 4 OUTER board corners.")
        else:
            rospy.loginfo("[vision_node] Loaded board corners from ~board_corners.")

    def terminal_log_board(self, title, board):
        msg = (
            f"\n================ {title} ================\n"
            f"{board}\n"
            f"=========================================\n"
        )
        print(msg, flush=True)
        rospy.loginfo(msg)

    def on_shutdown(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    def load_corners_from_param(self):
        corners = rospy.get_param("~board_corners", None)

        if corners is None:
            return None

        try:
            arr = np.array(corners, dtype=np.float32)

            if arr.shape == (4, 2):
                return self.order_points(arr)

            if arr.shape == (8,):
                return self.order_points(arr.reshape(4, 2))

        except Exception:
            pass

        rospy.logwarn("[vision_node] Invalid ~board_corners format.")
        return None

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

                self.board_history.clear()
                self.last_filtered_board = np.zeros((ROWS, COLS), dtype=np.int8)
                self.last_candidate_board = None
                self.last_candidate_time = None
                self.last_published_board = None
                self.last_publish_time = None

                rospy.loginfo("[vision_node] Calibration complete.")
                rospy.loginfo(f"[vision_node] Save corners as ~board_corners: {flat}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            with self.frame_lock:
                if self.latest_depth is None:
                    depth = None
                    rospy.logwarn_throttle(5.0, "[vision_node] No depth data received yet.")
                else:
                    depth = self.latest_depth.copy()

        except Exception as e:
            rospy.logerr(f"[vision_node] CV Bridge error: {e}")
            return

        board, overlay_frame = self.detect_board_and_overlay(frame, depth)

        with self.state_lock:
            calibrated = self.board_quad is not None

        if calibrated:
            # Do NOT flip here.
            # Voting and gravity validation expect row 0 = top and row ROWS - 1 = bottom.
            filtered_board = self.filter_board_with_votes(board)

            if filtered_board is not None:
                self.update_stable_board(filtered_board)

        with self.frame_lock:
            self.latest_overlay = overlay_frame.copy()

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(f"[vision_node] Depth CV Bridge error: {e}")
            return

        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) / 1000.0
        else:
            depth = depth.astype(np.float32)

        with self.frame_lock:
            self.latest_depth = depth

    def filter_board_with_votes(self, board):
        self.board_history.append(board.copy())

        if len(self.board_history) < self.vote_frames:
            return None

        stack = np.stack(list(self.board_history), axis=0)
        filtered = self.last_filtered_board.copy()

        for r in range(ROWS):
            for c in range(COLS):
                values = stack[:, r, c]
                counts = np.bincount(values.astype(np.int32), minlength=3)

                best_value = int(np.argmax(counts))
                best_count = int(counts[best_value])

                if best_count >= self.vote_required:
                    filtered[r, c] = best_value

        if not self.is_gravity_valid(filtered):
            rospy.logwarn_throttle(
                1.0,
                "[vision_node] Rejected unstable board: invalid gravity."
            )
            return None

        self.last_filtered_board = filtered.copy()
        return filtered

    def is_gravity_valid(self, board):
        for c in range(COLS):
            seen_empty = False

            # Start at bottom row and move upward.
            for r in range(ROWS - 1, -1, -1):
                if board[r, c] == 0:
                    seen_empty = True
                elif seen_empty:
                    return False

        return True

    def update_stable_board(self, board):
        now = rospy.Time.now()

        if self.last_candidate_board is None:
            self.last_candidate_board = board.copy()
            self.last_candidate_time = now
            rospy.loginfo("[vision_node] First candidate board detected. Waiting for stability.")
            return

        if not np.array_equal(board, self.last_candidate_board):
            self.last_candidate_board = board.copy()
            self.last_candidate_time = now
            rospy.loginfo("[vision_node] Board changed. Waiting for stable board...")
            return

        stable_duration = (now - self.last_candidate_time).to_sec()

        if stable_duration < self.stable_time:
            return

        is_new_board = (
            self.last_published_board is None or
            not np.array_equal(board, self.last_published_board)
        )

        if self.last_publish_time is not None:
            if (now - self.last_publish_time).to_sec() < self.min_publish_gap:
                return

        if is_new_board:
            self.terminal_log_board("VISION: NEW STABLE BOARD DETECTED", board)
        else:
            rospy.loginfo("[vision_node] Stable board unchanged. Republishing /board_state.")

        self.publish_board(board)

        self.last_published_board = board.copy()
        self.last_publish_time = now

    def publish_board(self, board):
        msg_out = Int8MultiArray()

        msg_out.layout.dim = [
            MultiArrayDimension(label="rows", size=ROWS, stride=ROWS * COLS),
            MultiArrayDimension(label="cols", size=COLS, stride=COLS),
        ]

        msg_out.layout.data_offset = 0

        # Flip only the message sent to the logic node.
        # The overlay and internal gravity check remain unchanged.
        if self.flip_board:
            board_to_publish = np.flipud(board)
        else:
            board_to_publish = board

        msg_out.data = board_to_publish.flatten().astype(np.int8).tolist()

        self.pub.publish(msg_out)
        rospy.loginfo("[vision_node] Published stable board to /board_state.")

        if self.flip_board:
            self.terminal_log_board("VISION: PUBLISHED FLIPPED BOARD", board_to_publish)

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

                self.board_history.clear()
                self.last_filtered_board = np.zeros((ROWS, COLS), dtype=np.int8)
                self.last_candidate_board = None
                self.last_candidate_time = None
                self.last_published_board = None
                self.last_publish_time = None

                rospy.loginfo("[vision_node] Calibration reset. Click 4 corners again.")

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

    def draw_calibration_view(self, frame):
        out = frame.copy()

        with self.state_lock:
            points = list(self.click_points)

        for i, p in enumerate(points):
            x, y = int(p[0]), int(p[1])

            cv2.circle(out, (x, y), 5, (0, 255, 255), -1)

            cv2.putText(
                out,
                str(i + 1),
                (x + 8, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )

        if len(points) >= 2:
            pts = np.array(points, dtype=np.int32)
            cv2.polylines(out, [pts], False, (0, 255, 255), 2)

        cv2.putText(
            out,
            "Click 4 OUTER board corners",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )

        cv2.putText(
            out,
            "Suggested: TL, TR, BR, BL",
            (10, 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2
        )

        cv2.putText(
            out,
            "Press r to reset, q to quit",
            (10, 85),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        return out

    def clean_mask(self, mask):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)
        return mask

    def detect_cells_on_warped(self, warped_bgr, warped_depth=None):
        if self.blur_kernel > 1:
            warped_bgr = cv2.GaussianBlur(
                warped_bgr,
                (self.blur_kernel, self.blur_kernel),
                0
            )
                    
        # CLAHE preprocessing for normalising the board 
        lab = cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=1.2, tileGridSize=(8,8))
        l = clahe.apply(l)
        warped_bgr = cv2.cvtColor(cv2.merge([l,a,b]), cv2.COLOR_LAB2BGR)

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

                if warped_depth is not None:
                    depth_roi = warped_depth[y1:y2, x1:x2]
                    depth_valid = (
                        np.isfinite(depth_roi) &
                        (depth_roi >= self.min_token_depth) &
                        (depth_roi <= self.max_token_depth)
                    )
                    depth_mask = depth_valid.astype(np.uint8) * 255
                    circle_mask = cv2.bitwise_and(circle_mask, depth_mask)

                valid_area = cv2.countNonZero(circle_mask)
                if valid_area <= 0:
                    continue

                mask_red1 = cv2.inRange(roi, self.lower_red1, self.upper_red1)
                mask_red2 = cv2.inRange(roi, self.lower_red2, self.upper_red2)
                mask_red = cv2.bitwise_or(mask_red1, mask_red2)

                mask_blue = cv2.inRange(roi, self.lower_blue, self.upper_blue)

                mask_red = cv2.bitwise_and(mask_red, circle_mask)
                mask_blue = cv2.bitwise_and(mask_blue, circle_mask)

                mask_red = self.clean_mask(mask_red)
                mask_blue = self.clean_mask(mask_blue)

                red_px = cv2.countNonZero(mask_red)
                blue_px = cv2.countNonZero(mask_blue)

                red_ratio = red_px / float(valid_area)
                blue_ratio = blue_px / float(valid_area)

                if (
                    red_px > self.px_thresh and
                    red_ratio > self.min_ratio and
                    red_px > blue_px + self.dominance_margin
                ):
                    state = 1
                    fill_bgr = (0, 0, 255)
                    text = "R"

                elif (
                    blue_px > self.px_thresh and
                    blue_ratio > self.min_ratio and
                    blue_px > red_px + self.dominance_margin
                ):
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

                slot_poly = self.make_circle_polygon(
                    cx_global,
                    cy_global,
                    radius,
                    self.circle_pts
                )

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

    def detect_board_and_overlay(self, frame, depth=None):
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

        if depth is not None and depth.shape[:2] == frame.shape[:2]:
            warped_depth = cv2.warpPerspective(
                depth,
                M,
                (warp_w, warp_h),
                flags=cv2.INTER_NEAREST
            )
        else:
            warped_depth = None

        board, cell_visuals = self.detect_cells_on_warped(warped, warped_depth)

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

        out = cv2.addWeighted(
            overlay,
            self.alpha_fill,
            frame,
            1.0 - self.alpha_fill,
            0
        )

        cv2.polylines(
            out,
            [np.round(quad).astype(np.int32)],
            True,
            (0, 255, 255),
            2
        )

        for vis in projected_visuals:
            cv2.polylines(out, [vis["cell_poly"]], True, (0, 255, 0), 1)
            cv2.polylines(out, [vis["slot_poly"]], True, (255, 255, 255), 1)

            cx, cy = vis["center"]

            cv2.putText(
                out,
                vis["text"],
                (cx - 10, cy + 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

            if self.show_rc:
                anchor = vis["cell_poly"][0]

                cv2.putText(
                    out,
                    f"{vis['r']},{vis['c']}",
                    (int(anchor[0]) + 4, int(anchor[1]) + 16),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 255, 255),
                    1
                )

        cv2.putText(
            out,
            "Stable mode: voting + continuous publish",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        cv2.putText(
            out,
            "R=1 red, B=2 blue, .=0 empty",
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

        cv2.putText(
            out,
            "Press r to recalibrate, q to quit",
            (10, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

        return board, out


if __name__ == "__main__":
    try:
        node = VisionNode()
        node.run_ui()
    except rospy.ROSInterruptException:
        pass