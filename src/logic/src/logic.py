#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Int8MultiArray, Int8

ROWS = 6
COLS = 7

EMPTY = 0
ROBOT = 1
HUMAN = 2


class GameLogic:
    def __init__(self):
        rospy.init_node("game_logic")

        self.depth = int(rospy.get_param("~depth", 4))
        self.human_starts = bool(rospy.get_param("~human_starts", True))
        self.flip_board = bool(rospy.get_param("~flip_board", False))

        self.next_move_topic = rospy.get_param("~next_move_topic", "/robot_next_move")

        self.board = np.zeros((ROWS, COLS), dtype=np.int8)
        self.have_board = False

        self.robot_command_pending = False
        self.pending_robot_col = None

        self.game_over = False

        self.last_human_wait_time = None
        self.human_reminder_gap = float(rospy.get_param("~human_reminder_gap", 10.0))

        self.next_move_pub = rospy.Publisher(self.next_move_topic, Int8, queue_size=1)

        rospy.Subscriber("/board_state", Int8MultiArray, self.board_callback, queue_size=1)

        rospy.loginfo("[connect4_game_logic] Node ready.")
        rospy.loginfo("[connect4_game_logic] Waiting for /board_state from vision.")
        rospy.loginfo(f"[connect4_game_logic] Publishing robot move on: {self.next_move_topic}")

    def terminal_log(self, title, text=""):
        msg = (
            f"\n================ {title} ================\n"
            f"{text}\n"
            f"=========================================\n"
        )
        print(msg, flush=True)
        rospy.loginfo(msg)

    def terminal_log_board(self, title, board):
        msg = (
            f"\n================ {title} ================\n"
            f"{board}\n"
            f"=========================================\n"
        )
        print(msg, flush=True)
        rospy.loginfo(msg)

    def board_callback(self, msg):
        if self.game_over:
            return

        new_board = self.parse_board(msg)

        if new_board is None:
            return

        if not self.is_board_state_valid(new_board):
            rospy.logwarn_throttle(
                1.0,
                "[connect4_game_logic] Ignoring invalid board from vision."
            )
            return

        self.terminal_log_board("LOGIC: VALID BOARD RECEIVED FROM VISION", new_board)

        if not self.have_board:
            self.board = new_board
            self.have_board = True
            rospy.loginfo("[connect4_game_logic] First valid board received.")
            self.handle_position()
            return

        if np.array_equal(new_board, self.board):
            self.handle_position()
            return

        old_board = self.board.copy()
        self.board = new_board

        move_info = self.describe_single_new_piece(old_board, new_board)

        if move_info is not None:
            player, row, col = move_info
            who = "HUMAN / BLUE" if player == HUMAN else "ROBOT / RED"

            self.terminal_log(
                "LOGIC: MOVE DETECTED FROM VISION",
                f"Detected move by: {who}\nrow={row}\ncol={col}"
            )
        else:
            self.terminal_log_board("LOGIC: BOARD CHANGED", new_board)

        if self.robot_command_pending:
            old_robot_count = int(np.count_nonzero(old_board == ROBOT))
            new_robot_count = int(np.count_nonzero(new_board == ROBOT))

            if new_robot_count > old_robot_count:
                self.terminal_log(
                    "LOGIC: ROBOT MOVE CONFIRMED BY VISION",
                    f"Robot piece count changed from {old_robot_count} to {new_robot_count}."
                )
                self.robot_command_pending = False
                self.pending_robot_col = None
            else:
                rospy.loginfo_throttle(
                    1.0,
                    "[connect4_game_logic] Robot command pending. Waiting for vision to see red piece."
                )
                return

        self.handle_position()

    def parse_board(self, msg):
        if len(msg.data) != ROWS * COLS:
            rospy.logwarn_throttle(
                1.0,
                f"[connect4_game_logic] Expected {ROWS * COLS} cells, got {len(msg.data)}."
            )
            return None

        try:
            board = np.array(msg.data, dtype=np.int8).reshape((ROWS, COLS))

            if self.flip_board:
                board = np.flipud(board)

            return board

        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[connect4_game_logic] Bad board reshape: {e}")
            return None

    def is_board_state_valid(self, board):
        if not np.all(np.isin(board, [EMPTY, HUMAN, ROBOT])):
            rospy.logwarn("[connect4_game_logic] Board contains invalid values.")
            return False

        for c in range(COLS):
            seen_empty = False

            for r in range(ROWS - 1, -1, -1):
                if board[r, c] == EMPTY:
                    seen_empty = True
                elif seen_empty:
                    rospy.logwarn(
                        f"[connect4_game_logic] Invalid gravity in column {c}. "
                        f"Piece at row {r} is floating."
                    )
                    return False

        human_count = int(np.count_nonzero(board == HUMAN))
        robot_count = int(np.count_nonzero(board == ROBOT))

        rospy.loginfo_throttle(
            1.0,
            f"[connect4_game_logic] Counts: HUMAN={human_count}, ROBOT={robot_count}"
        )

        if self.human_starts:
            if robot_count > human_count:
                rospy.logwarn("[connect4_game_logic] Invalid count: robot has more pieces than human.")
                return False

            if human_count - robot_count > 1:
                rospy.logwarn("[connect4_game_logic] Invalid count: human is more than 1 move ahead.")
                return False

        else:
            if human_count > robot_count:
                rospy.logwarn("[connect4_game_logic] Invalid count: human has more pieces than robot.")
                return False

            if robot_count - human_count > 1:
                rospy.logwarn("[connect4_game_logic] Invalid count: robot is more than 1 move ahead.")
                return False

        if self.check_win(board, HUMAN) and self.check_win(board, ROBOT):
            rospy.logwarn("[connect4_game_logic] Invalid board: both players have winning lines.")
            return False

        return True

    def whose_turn(self, board):
        human_count = int(np.count_nonzero(board == HUMAN))
        robot_count = int(np.count_nonzero(board == ROBOT))

        if self.human_starts:
            if human_count == robot_count:
                return HUMAN

            if human_count == robot_count + 1:
                return ROBOT

        else:
            if human_count == robot_count:
                return ROBOT

            if robot_count == human_count + 1:
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

        expected_row = None

        for rr in range(ROWS - 1, -1, -1):
            if old_board[rr, c] == EMPTY:
                expected_row = rr
                break

        if expected_row != int(r):
            return None

        return new_val, int(r), int(c)

    def handle_position(self):
        if self.game_over:
            return

        if self.robot_command_pending:
            rospy.loginfo_throttle(
                1.0,
                "[connect4_game_logic] Robot move already sent. Waiting for vision confirmation."
            )
            return

        if self.check_win(self.board, HUMAN):
            self.game_over = True
            self.terminal_log(
                "GAME OVER: YOU WIN",
                "Human / BLUE wins.\nRobot / RED loses."
            )
            return

        if self.check_win(self.board, ROBOT):
            self.game_over = True
            self.terminal_log(
                "GAME OVER: YOU LOSE",
                "Robot / RED wins.\nHuman / BLUE loses."
            )
            return

        valid = self.valid_moves(self.board)

        if len(valid) == 0:
            self.game_over = True
            self.terminal_log(
                "GAME OVER: DRAW",
                "The board is full. No winner."
            )
            return

        turn = self.whose_turn(self.board)

        if turn is None:
            rospy.logwarn("[connect4_game_logic] Invalid turn state.")
            return

        if turn == HUMAN:
            now = rospy.Time.now()

            if self.last_human_wait_time is None:
                self.last_human_wait_time = now
                self.terminal_log(
                    "LOGIC: HUMAN TURN",
                    "Waiting for you to make a move."
                )
                return

            elapsed = (now - self.last_human_wait_time).to_sec()

            if elapsed >= self.human_reminder_gap:
                self.terminal_log(
                    "LOGIC: HURRY UP",
                    f"It has been {int(elapsed)} seconds. Please make your move."
                )
                self.last_human_wait_time = now
            else:
                rospy.loginfo_throttle(
                    2.0,
                    "[connect4_game_logic] Waiting for human move..."
                )

            return

        self.last_human_wait_time = None

        col_zero_based, score = self.minimax(self.board, self.depth, -1e9, 1e9, True)

        if col_zero_based is None:
            rospy.loginfo("[connect4_game_logic] No legal move.")
            return

        col_robot = col_zero_based + 1

        self.terminal_log(
            "LOGIC: SENDING MOVE TO MOVE NODE",
            f"Correct board state detected.\n"
            f"Internal column: {col_zero_based}\n"
            f"Robot column sent: {col_robot}\n"
            f"Score: {score:.1f}\n"
            f"Topic: {self.next_move_topic}"
        )

        self.next_move_pub.publish(Int8(data=int(col_robot)))

        self.robot_command_pending = True
        self.pending_robot_col = col_zero_based

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

    def check_win(self, board, p):
        for r in range(ROWS):
            for c in range(COLS - 3):
                if np.all(board[r, c:c + 4] == p):
                    return True

        for r in range(ROWS - 3):
            for c in range(COLS):
                if np.all(board[r:r + 4, c] == p):
                    return True

        for r in range(3, ROWS):
            for c in range(COLS - 3):
                if all(board[r - i, c + i] == p for i in range(4)):
                    return True

        for r in range(ROWS - 3):
            for c in range(COLS - 3):
                if all(board[r + i, c + i] == p for i in range(4)):
                    return True

        return False

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

        center_col = board[:, COLS // 2]
        score += np.count_nonzero(center_col == player) * 6

        for r in range(ROWS):
            for c in range(COLS - 3):
                score += self.evaluate_window(board[r, c:c + 4], player)

        for c in range(COLS):
            for r in range(ROWS - 3):
                score += self.evaluate_window(board[r:r + 4, c], player)

        for r in range(ROWS - 3):
            for c in range(COLS - 3):
                window = np.array([board[r + i, c + i] for i in range(4)])
                score += self.evaluate_window(window, player)

        for r in range(3, ROWS):
            for c in range(COLS - 3):
                window = np.array([board[r - i, c + i] for i in range(4)])
                score += self.evaluate_window(window, player)

        return score

    def minimax(self, board, depth, alpha, beta, maximizing):
        valid = self.ordered_valid_moves(board)

        terminal = (
            self.check_win(board, HUMAN)
            or self.check_win(board, ROBOT)
            or len(valid) == 0
        )

        if depth == 0 or terminal:
            if self.check_win(board, ROBOT):
                return None, 1e6

            if self.check_win(board, HUMAN):
                return None, -1e6

            if len(valid) == 0:
                return None, 0

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