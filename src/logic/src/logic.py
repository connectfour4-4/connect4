#!/usr/bin/env python3
"""
connect4_game_logic.py (ROS1)

Reads board state from the vision node:
    /board_state  (std_msgs/Int8MultiArray)
where:
    0 = empty
    1 = red   (human)
    2 = blue  (robot)

Behavior:
- Trusts vision as the source of truth
- Computes whose turn it is from piece counts
- Publishes the robot's next move (column index 0..6)
- If someone physically places the robot's blue piece by hand, the logic still continues
  because it only reacts to the observed board state

Outputs:
- /robot_next_move   (std_msgs/Int8)  always publishes the recommended column
- /robot_play_col    (std_msgs/Int8)  optional auto-execute topic for your robot node

Notes:
- This does NOT directly control real MoveIt.
- A separate motion node should subscribe to /robot_play_col (or /robot_next_move)
  and do the actual robot movement.
"""

import rospy
import numpy as np
from std_msgs.msg import Int8MultiArray, Int8

ROWS = 6
COLS = 7

EMPTY = 0
HUMAN = 1   # red
ROBOT = 2   # blue


class GameLogic:
    def __init__(self):
        rospy.init_node("game_logic")

        self.depth = int(rospy.get_param("~depth", 4))
        self.human_starts = bool(rospy.get_param("~human_starts", True))

        self.next_move_topic = rospy.get_param("~next_move_topic", "/robot_next_move")
        self.auto_execute = bool(rospy.get_param("~auto_execute", False))
        self.execute_topic = rospy.get_param("~execute_topic", "/robot_play_col")

        self.board = np.zeros((ROWS, COLS), dtype=np.int8)
        self.have_board = False

        self.next_move_pub = rospy.Publisher(self.next_move_topic, Int8, queue_size=1)
        self.exec_pub = None
        if self.auto_execute:
            self.exec_pub = rospy.Publisher(self.execute_topic, Int8, queue_size=1)

        rospy.Subscriber("/board_state", Int8MultiArray, self.board_callback, queue_size=1)

        rospy.loginfo("game_logic] Node ready.")
        rospy.loginfo("game_logic Waiting for /board_state from vision node...")
        rospy.loginfo(f"[connect4_game_logic] Publishing suggested move on: {self.next_move_topic}")
        if self.auto_execute:
            rospy.loginfo(f"[connect4_game_logic] Auto execute enabled on: {self.execute_topic}")

    # ------------------------------------------------------------
    # ROS callback
    # ------------------------------------------------------------
    def board_callback(self, msg):
        new_board = self.parse_board(msg)
        if new_board is None:
            return

        if not self.is_board_state_valid(new_board):
            rospy.logwarn_throttle(1.0, "[connect4_game_logic] Ignoring invalid board from vision.")
            return

        # First valid board received
        if not self.have_board:
            self.board = new_board
            self.have_board = True
            rospy.loginfo("[connect4_game_logic] First board received from vision.")
            self.handle_position()
            return

        # Ignore duplicate frames
        if np.array_equal(new_board, self.board):
            return

        old_board = self.board.copy()
        self.board = new_board

        move_info = self.describe_single_new_piece(old_board, new_board)
        if move_info is not None:
            player, row, col = move_info
            who = "HUMAN(red)" if player == HUMAN else "ROBOT(blue)"
            rospy.loginfo(f"[connect4_game_logic] Detected move: {who} -> row={row}, col={col}")
        else:
            rospy.loginfo("[connect4_game_logic] Board changed.")

        self.handle_position()

    # ------------------------------------------------------------
    # Parsing / validation
    # ------------------------------------------------------------
    def parse_board(self, msg):
        if len(msg.data) != ROWS * COLS:
            rospy.logwarn_throttle(
                1.0,
                f"[connect4_game_logic] Expected {ROWS * COLS} cells, got {len(msg.data)}."
            )
            return None

        try:
            board = np.array(msg.data, dtype=np.int8).reshape((ROWS, COLS))
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[connect4_game_logic] Bad board reshape: {e}")
            return None

        return board

    def is_board_state_valid(self, board):
        # Values must be 0,1,2 only
        if not np.all(np.isin(board, [EMPTY, HUMAN, ROBOT])):
            return False

        # Gravity check: no floating pieces
        for c in range(COLS):
            seen_empty = False
            for r in range(ROWS - 1, -1, -1):  # bottom -> top
                if board[r, c] == EMPTY:
                    seen_empty = True
                elif seen_empty:
                    return False

        red_count = int(np.count_nonzero(board == HUMAN))
        blue_count = int(np.count_nonzero(board == ROBOT))

        # Turn-count validity
        if self.human_starts:
            if blue_count > red_count:
                return False
            if red_count - blue_count > 1:
                return False
        else:
            if red_count > blue_count:
                return False
            if blue_count - red_count > 1:
                return False

        # Both players cannot simultaneously have winning lines
        if self.check_win(board, HUMAN) and self.check_win(board, ROBOT):
            return False

        return True

    def whose_turn(self, board):
        red_count = int(np.count_nonzero(board == HUMAN))
        blue_count = int(np.count_nonzero(board == ROBOT))

        if self.human_starts:
            if red_count == blue_count:
                return HUMAN
            if red_count == blue_count + 1:
                return ROBOT
        else:
            if red_count == blue_count:
                return ROBOT
            if blue_count == red_count + 1:
                return HUMAN

        return None

    def describe_single_new_piece(self, old_board, new_board):
        diff = np.argwhere(old_board != new_board)
        if len(diff) != 1:
            return None

        r, c = diff[0]
        old_val = int(old_board[r, c])
        new_val = int(new_board[r, c])

        if old_val != EMPTY or new_val not in (HUMAN, ROBOT):
            return None

        # Must be the lowest empty row in that column
        expected_row = None
        for rr in range(ROWS - 1, -1, -1):
            if old_board[rr, c] == EMPTY:
                expected_row = rr
                break

        if expected_row != int(r):
            return None

        return new_val, int(r), int(c)

    # ------------------------------------------------------------
    # Game flow
    # ------------------------------------------------------------
    def handle_position(self):
        if self.check_win(self.board, HUMAN):
            rospy.loginfo("[connect4_game_logic] Human wins.")
            return

        if self.check_win(self.board, ROBOT):
            rospy.loginfo("[connect4_game_logic] Robot wins.")
            return

        valid = self.valid_moves(self.board)
        if len(valid) == 0:
            rospy.loginfo("[connect4_game_logic] Draw.")
            return

        turn = self.whose_turn(self.board)
        if turn is None:
            rospy.logwarn("[connect4_game_logic] Invalid turn state.")
            return

        if turn == HUMAN:
            rospy.loginfo_throttle(1.0, "[connect4_game_logic] Waiting for human move.")
            return

        # Robot turn
        col, score = self.minimax(self.board, self.depth, -1e9, 1e9, True)
        if col is None:
            rospy.loginfo("[connect4_game_logic] No legal move.")
            return

        rospy.loginfo(f"[connect4_game_logic] Robot should play column {col} (score {score:.1f})")

        # Always publish the recommendation
        self.next_move_pub.publish(Int8(data=int(col)))

        # Optional: also publish to execution topic
        if self.auto_execute and self.exec_pub is not None:
            self.exec_pub.publish(Int8(data=int(col)))
            rospy.loginfo(f"[connect4_game_logic] Sent column {col} to {self.execute_topic}")

    # ------------------------------------------------------------
    # Connect4 helpers
    # ------------------------------------------------------------
    def valid_moves(self, board):
        return [c for c in range(COLS) if board[0, c] == EMPTY]

    def ordered_valid_moves(self, board):
        center = COLS // 2
        return sorted(self.valid_moves(board), key=lambda c: abs(c - center))

    def drop(self, board, col, player):
        new_board = board.copy()
        for r in range(ROWS - 1, -1, -1):
            if new_board[r, col] == EMPTY:
                new_board[r, col] = player
                return new_board
        return None

    # ------------------------------------------------------------
    # Win check
    # ------------------------------------------------------------
    def check_win(self, board, p):
        # Horizontal
        for r in range(ROWS):
            for c in range(COLS - 3):
                if np.all(board[r, c:c + 4] == p):
                    return True

        # Vertical
        for r in range(ROWS - 3):
            for c in range(COLS):
                if np.all(board[r:r + 4, c] == p):
                    return True

        # Diagonal /
        for r in range(3, ROWS):
            for c in range(COLS - 3):
                if all(board[r - i, c + i] == p for i in range(4)):
                    return True

        # Diagonal \
        for r in range(ROWS - 3):
            for c in range(COLS - 3):
                if all(board[r + i, c + i] == p for i in range(4)):
                    return True

        return False

    # ------------------------------------------------------------
    # Evaluation
    # ------------------------------------------------------------
    def evaluate_window(self, window, player):
        score = 0
        opp = HUMAN if player == ROBOT else ROBOT

        player_count = np.count_nonzero(window == player)
        opp_count = np.count_nonzero(window == opp)
        empty_count = np.count_nonzero(window == EMPTY)

        if player_count == 4:
            score += 100000
        elif player_count == 3 and empty_count == 1:
            score += 50
        elif player_count == 2 and empty_count == 2:
            score += 10

        if opp_count == 4:
            score -= 100000
        elif opp_count == 3 and empty_count == 1:
            score -= 80
        elif opp_count == 2 and empty_count == 2:
            score -= 8

        return score

    def score_position(self, board, player):
        score = 0

        # Center preference
        center_col = board[:, COLS // 2]
        score += np.count_nonzero(center_col == player) * 6

        # Horizontal
        for r in range(ROWS):
            for c in range(COLS - 3):
                score += self.evaluate_window(board[r, c:c + 4], player)

        # Vertical
        for c in range(COLS):
            for r in range(ROWS - 3):
                score += self.evaluate_window(board[r:r + 4, c], player)

        # Diagonal \
        for r in range(ROWS - 3):
            for c in range(COLS - 3):
                window = np.array([board[r + i, c + i] for i in range(4)])
                score += self.evaluate_window(window, player)

        # Diagonal /
        for r in range(3, ROWS):
            for c in range(COLS - 3):
                window = np.array([board[r - i, c + i] for i in range(4)])
                score += self.evaluate_window(window, player)

        return score

    # ------------------------------------------------------------
    # Minimax
    # ------------------------------------------------------------
    def minimax(self, board, depth, alpha, beta, maximizing):
        valid = self.ordered_valid_moves(board)

        terminal = (
            self.check_win(board, HUMAN) or
            self.check_win(board, ROBOT) or
            len(valid) == 0
        )

        if depth == 0 or terminal:
            if self.check_win(board, ROBOT):
                return None, 1e6
            elif self.check_win(board, HUMAN):
                return None, -1e6
            elif len(valid) == 0:
                return None, 0
            else:
                return None, self.score_position(board, ROBOT)

        if maximizing:
            value = -1e9
            best_col = valid[0]

            for col in valid:
                child = self.drop(board, col, ROBOT)
                _, new_score = self.minimax(child, depth - 1, alpha, beta, False)

                if new_score > value:
                    value = new_score
                    best_col = col

                alpha = max(alpha, value)
                if alpha >= beta:
                    break

            return best_col, value

        else:
            value = 1e9
            best_col = valid[0]

            for col in valid:
                child = self.drop(board, col, HUMAN)
                _, new_score = self.minimax(child, depth - 1, alpha, beta, True)

                if new_score < value:
                    value = new_score
                    best_col = col

                beta = min(beta, value)
                if alpha >= beta:
                    break

            return best_col, value


if __name__ == "__main__":
    try:
        GameLogic()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
