#!/usr/bin/env python3

import rospy
import numpy as np
import threading
import time
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

        # Used only as a tie-breaker if both normal and flipped boards look valid.
        self.flip_board = bool(rospy.get_param("~flip_board", False))

        # If true, parse_board will test both normal and flipped board orientation.
        self.auto_detect_flip = bool(rospy.get_param("~auto_detect_flip", True))

        self.next_move_topic = rospy.get_param("~next_move_topic", "/robot_next_move")

        self.board = np.zeros((ROWS, COLS), dtype=np.int8)
        self.have_board = False

        self.robot_command_pending = False
        self.pending_robot_col = None

        self.game_over = False
        self.waiting_for_enter = False
        self.waiting_for_empty_board = False

        self.human_wins = 0
        self.robot_wins = 0
        self.draws = 0
        self.games_played = 0

        self.last_human_wait_time = None
        self.human_reminder_gap = float(rospy.get_param("~human_reminder_gap", 10.0))

        self.transposition_table = {}

        self.next_move_pub = rospy.Publisher(self.next_move_topic, Int8, queue_size=1)

        rospy.Subscriber("/board_state", Int8MultiArray, self.board_callback, queue_size=1)

        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        rospy.loginfo("[connect4_game_logic] Node ready.")
        rospy.loginfo("[connect4_game_logic] Waiting for /board_state from vision.")
        rospy.loginfo(f"[connect4_game_logic] Publishing robot move on: {self.next_move_topic}")
        rospy.loginfo(f"[connect4_game_logic] AI search depth: {self.depth}")
        rospy.loginfo(f"[connect4_game_logic] auto_detect_flip={self.auto_detect_flip}")
        rospy.loginfo(f"[connect4_game_logic] flip_board tie-breaker={self.flip_board}")

    def keyboard_loop(self):
        while not rospy.is_shutdown():
            if self.waiting_for_enter:
                try:
                    input("\nPress ENTER to start a new game...")
                    self.reset_for_new_game()
                except EOFError:
                    return
                except Exception as e:
                    rospy.logwarn(f"[connect4_game_logic] Keyboard input error: {e}")
                    time.sleep(0.5)
            else:
                time.sleep(0.1)

    def reset_for_new_game(self):
        self.board = np.zeros((ROWS, COLS), dtype=np.int8)
        self.have_board = False

        self.robot_command_pending = False
        self.pending_robot_col = None

        self.game_over = False
        self.waiting_for_enter = False
        self.waiting_for_empty_board = True

        self.last_human_wait_time = None
        self.transposition_table.clear()

        self.terminal_log(
            "NEW GAME REQUESTED",
            "Scoreboard is kept.\n"
            "Please clear the physical board.\n"
            "Waiting for vision to publish an empty board."
        )

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

    def terminal_log_scoreboard(self):
        self.terminal_log(
            "SCOREBOARD",
            f"Games played: {self.games_played}\n"
            f"Human wins:   {self.human_wins}\n"
            f"Robot wins:   {self.robot_wins}\n"
            f"Draws:        {self.draws}"
        )

    def finish_game(self, winner):
        if self.game_over:
            return

        self.game_over = True
        self.waiting_for_enter = True
        self.games_played += 1

        if winner == HUMAN:
            self.human_wins += 1
            self.terminal_log(
                "GAME OVER: YOU WIN",
                "Human / BLUE wins.\nRobot / RED loses."
            )

        elif winner == ROBOT:
            self.robot_wins += 1
            self.terminal_log(
                "GAME OVER: YOU LOSE",
                "Robot / RED wins.\nHuman / BLUE loses."
            )

        else:
            self.draws += 1
            self.terminal_log(
                "GAME OVER: DRAW",
                "The board is full. No winner."
            )

        self.terminal_log_scoreboard()

        self.terminal_log(
            "READY FOR NEW GAME",
            "Press ENTER in this terminal when you want to start a new game."
        )

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

        if self.waiting_for_empty_board:
            if np.count_nonzero(new_board) == 0:
                self.waiting_for_empty_board = False
                self.board = new_board
                self.have_board = True

                self.terminal_log(
                    "NEW GAME STARTED",
                    "Empty board detected.\nGame logic is ready."
                )

                self.handle_position()
            else:
                rospy.loginfo_throttle(
                    1.0,
                    "[connect4_game_logic] Waiting for empty board before starting new game."
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
            raw_board = np.array(msg.data, dtype=np.int8).reshape((ROWS, COLS))
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[connect4_game_logic] Bad board reshape: {e}")
            return None

        if not self.auto_detect_flip:
            board = np.flipud(raw_board) if self.flip_board else raw_board
            mode = "FLIPPED by parameter" if self.flip_board else "NORMAL by parameter"
            rospy.loginfo_throttle(
                1.0,
                f"[connect4_game_logic] Board orientation mode: {mode}"
            )
            return board

        normal_board = raw_board
        flipped_board = np.flipud(raw_board)

        normal_valid = self.is_board_state_valid_silent(normal_board)
        flipped_valid = self.is_board_state_valid_silent(flipped_board)

        if normal_valid and not flipped_valid:
            rospy.loginfo_throttle(
                1.0,
                "[connect4_game_logic] Board orientation auto-detected: NORMAL"
            )
            return normal_board

        if flipped_valid and not normal_valid:
            rospy.loginfo_throttle(
                1.0,
                "[connect4_game_logic] Board orientation auto-detected: FLIPPED"
            )
            return flipped_board

        if normal_valid and flipped_valid:
            chosen = flipped_board if self.flip_board else normal_board
            mode = "FLIPPED" if self.flip_board else "NORMAL"

            rospy.logwarn_throttle(
                1.0,
                f"[connect4_game_logic] Both NORMAL and FLIPPED boards look valid. "
                f"Using tie-breaker _flip_board={self.flip_board}: {mode}"
            )
            return chosen

        rospy.logwarn_throttle(
            1.0,
            "[connect4_game_logic] Neither NORMAL nor FLIPPED board passed validation."
        )

        rospy.logwarn_throttle(
            1.0,
            f"[connect4_game_logic] Raw board from vision:\n{raw_board}"
        )

        rospy.logwarn_throttle(
            1.0,
            f"[connect4_game_logic] Flipped candidate:\n{flipped_board}"
        )

        return None

    def is_board_state_valid_silent(self, board):
        if not np.all(np.isin(board, [EMPTY, HUMAN, ROBOT])):
            return False

        for c in range(COLS):
            seen_empty = False

            for r in range(ROWS - 1, -1, -1):
                if board[r, c] == EMPTY:
                    seen_empty = True
                elif seen_empty:
                    return False

        human_count = int(np.count_nonzero(board == HUMAN))
        robot_count = int(np.count_nonzero(board == ROBOT))

        if self.human_starts:
            if robot_count > human_count:
                return False

            if human_count - robot_count > 1:
                return False

        else:
            if human_count > robot_count:
                return False

            if robot_count - human_count > 1:
                return False

        if self.check_win(board, HUMAN) and self.check_win(board, ROBOT):
            return False

        return True

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
            self.finish_game(HUMAN)
            return

        if self.check_win(self.board, ROBOT):
            self.finish_game(ROBOT)
            return

        valid = self.valid_moves(self.board)

        if len(valid) == 0:
            self.finish_game(None)
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

        col_zero_based, score, reason = self.choose_best_robot_move(self.board)

        if col_zero_based is None:
            rospy.loginfo("[connect4_game_logic] No legal move.")
            return

        col_robot = col_zero_based + 1

        self.terminal_log(
            "LOGIC: SENDING MOVE TO MOVE NODE",
            f"Correct board state detected.\n"
            f"Reason: {reason}\n"
            f"Internal column: {col_zero_based}\n"
            f"Robot column sent: {col_robot}\n"
            f"Score: {score:.1f}\n"
            f"Topic: {self.next_move_topic}"
        )

        self.next_move_pub.publish(Int8(data=int(col_robot)))

        self.robot_command_pending = True
        self.pending_robot_col = col_zero_based

    def choose_best_robot_move(self, board):
        self.debug_immediate_wins(board)

        robot_winning_cols = self.find_winning_moves(board, ROBOT)
        human_winning_cols = self.find_winning_moves(board, HUMAN)

        if len(robot_winning_cols) > 0:
            col = robot_winning_cols[0]
            return col, 1e6, f"Immediate robot winning move at internal col {col}"

        if len(human_winning_cols) > 0:
            col = human_winning_cols[0]
            return col, 9e5, f"Immediate block against human winning move at internal col {col}"

        self.transposition_table.clear()

        col, score = self.minimax(
            board=board,
            depth=self.depth,
            alpha=-1e9,
            beta=1e9,
            maximizing=True
        )

        return col, score, "Minimax search"

    def debug_immediate_wins(self, board):
        rospy.logwarn("\n[DEBUG] ===== CHECKING IMMEDIATE WIN THREATS =====")
        rospy.logwarn(f"\n[DEBUG] Current interpreted board:\n{board}")

        human_cols = self.find_winning_moves(board, HUMAN)
        robot_cols = self.find_winning_moves(board, ROBOT)

        rospy.logwarn(f"[DEBUG] HUMAN immediate winning internal columns: {human_cols}")
        rospy.logwarn(f"[DEBUG] ROBOT immediate winning internal columns: {robot_cols}")

        if len(human_cols) > 0:
            rospy.logwarn(
                f"[DEBUG] HUMAN CAN WIN NOW. Robot must block internal col {human_cols[0]}, "
                f"which sends robot command column {human_cols[0] + 1}."
            )

        if len(robot_cols) > 0:
            rospy.logwarn(
                f"[DEBUG] ROBOT CAN WIN NOW. Robot should play internal col {robot_cols[0]}, "
                f"which sends robot command column {robot_cols[0] + 1}."
            )

        rospy.logwarn("[DEBUG] ===========================================\n")

    def valid_moves(self, board):
        return [c for c in range(COLS) if board[0, c] == EMPTY]

    def ordered_valid_moves(self, board):
        center = COLS // 2
        return sorted(self.valid_moves(board), key=lambda c: abs(c - center))

    def safe_robot_moves(self, board):
        safe = []

        for col in self.ordered_valid_moves(board):
            child = self.drop(board, col, ROBOT)

            if child is None:
                continue

            human_can_win = len(self.find_winning_moves(child, HUMAN, verbose=False)) > 0

            if not human_can_win:
                safe.append(col)

        return safe

    def drop(self, board, col, player):
        new_board = board.copy()

        for r in range(ROWS - 1, -1, -1):
            if new_board[r, col] == EMPTY:
                new_board[r, col] = player
                return new_board

        return None

    def find_winning_moves(self, board, player, verbose=True):
        winning_cols = []

        player_name = "ROBOT" if player == ROBOT else "HUMAN"

        for col in self.ordered_valid_moves(board):
            child = self.drop(board, col, player)

            if child is None:
                continue

            win = self.check_win(child, player)

            if verbose:
                rospy.loginfo(
                    f"[DEBUG] Test {player_name} drop in internal col {col}: win={win}"
                )

            if win:
                if verbose:
                    rospy.logwarn(
                        f"[DEBUG] {player_name} would win by dropping in internal col {col}. "
                        f"Command column would be {col + 1}."
                    )
                    rospy.logwarn(f"\n[DEBUG] Winning simulated board:\n{child}")

                winning_cols.append(col)

        return winning_cols

    def find_winning_move(self, board, player):
        winning_cols = self.find_winning_moves(board, player, verbose=False)

        if len(winning_cols) == 0:
            return None

        return winning_cols[0]

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
            score += 1000000
        elif player_count == 3 and empty_count == 1:
            score += 140
        elif player_count == 2 and empty_count == 2:
            score += 20
        elif player_count == 1 and empty_count == 3:
            score += 2

        if opp_count == 4:
            score -= 1000000
        elif opp_count == 3 and empty_count == 1:
            score -= 220
        elif opp_count == 2 and empty_count == 2:
            score -= 30
        elif opp_count == 1 and empty_count == 3:
            score -= 2

        return score

    def score_position(self, board, player):
        score = 0
        opp = HUMAN if player == ROBOT else ROBOT

        center_col = board[:, COLS // 2]
        score += np.count_nonzero(center_col == player) * 10
        score -= np.count_nonzero(center_col == opp) * 8

        near_center_left = board[:, (COLS // 2) - 1]
        near_center_right = board[:, (COLS // 2) + 1]

        score += np.count_nonzero(near_center_left == player) * 4
        score += np.count_nonzero(near_center_right == player) * 4

        score -= np.count_nonzero(near_center_left == opp) * 3
        score -= np.count_nonzero(near_center_right == opp) * 3

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

        score += self.count_potential_threats(board, player) * 60
        score -= self.count_potential_threats(board, opp) * 90

        return score

    def count_potential_threats(self, board, player):
        count = 0

        for col in self.valid_moves(board):
            child = self.drop(board, col, player)

            if child is not None and self.check_win(child, player):
                count += 1

        return count

    def minimax(self, board, depth, alpha, beta, maximizing):
        valid = self.ordered_valid_moves(board)

        terminal = (
            self.check_win(board, HUMAN)
            or self.check_win(board, ROBOT)
            or len(valid) == 0
        )

        key = (board.tobytes(), depth, maximizing)

        if key in self.transposition_table:
            return self.transposition_table[key]

        if depth == 0 or terminal:
            if self.check_win(board, ROBOT):
                result = (None, 1000000 + depth)
                self.transposition_table[key] = result
                return result

            if self.check_win(board, HUMAN):
                result = (None, -1000000 - depth)
                self.transposition_table[key] = result
                return result

            if len(valid) == 0:
                result = (None, 0)
                self.transposition_table[key] = result
                return result

            result = (None, self.score_position(board, ROBOT))
            self.transposition_table[key] = result
            return result

        if maximizing:
            value = -1e9
            best_col = valid[0]

            safe_moves = self.safe_robot_moves(board)
            search_moves = safe_moves if len(safe_moves) > 0 else valid

            for col in search_moves:
                child = self.drop(board, col, ROBOT)

                if child is None:
                    continue

                _, new_score = self.minimax(
                    board=child,
                    depth=depth - 1,
                    alpha=alpha,
                    beta=beta,
                    maximizing=False
                )

                if new_score > value:
                    value = new_score
                    best_col = col

                alpha = max(alpha, value)

                if alpha >= beta:
                    break

            result = (best_col, value)
            self.transposition_table[key] = result
            return result

        value = 1e9
        best_col = valid[0]

        for col in valid:
            child = self.drop(board, col, HUMAN)

            if child is None:
                continue

            _, new_score = self.minimax(
                board=child,
                depth=depth - 1,
                alpha=alpha,
                beta=beta,
                maximizing=True
            )

            if new_score < value:
                value = new_score
                best_col = col

            beta = min(beta, value)

            if alpha >= beta:
                break

        result = (best_col, value)
        self.transposition_table[key] = result
        return result


if __name__ == "__main__":
    try:
        GameLogic()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass