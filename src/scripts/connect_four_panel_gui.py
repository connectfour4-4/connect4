#!/usr/bin/env python3

import subprocess
import tkinter as tk
from pathlib import Path
from tkinter import messagebox


ROOT = Path("/home/project26-group3/connect-four")
PANEL = ROOT / "scriptpanel"

BUTTONS = [
    ("Start", PANEL / "start.sh", "#20d55a", "#111111"),
    ("Column 1", PANEL / "column-1.sh", "#1f7aff", "#ffffff"),
    ("Column 4", PANEL / "column-4.sh", "#1f7aff", "#ffffff"),
    ("Column 7", PANEL / "column-7.sh", "#1f7aff", "#ffffff"),
    ("Pick", PANEL / "pick.sh", "#4b5563", "#ffffff"),
    ("Open Gripper", PANEL / "open-gripper.sh", "#4b5563", "#ffffff"),
    ("Close Gripper", PANEL / "close-gripper.sh", "#4b5563", "#ffffff"),
]

COLUMNS_PER_ROW = 3


class ConnectFourPanel(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("Scriptpanel")
        self.geometry("600x410")
        self.configure(bg="#1f2329")
        self.resizable(False, False)

        title = tk.Label(
            self,
            text="Connect Four",
            font=("Liberation Mono", 22, "bold"),
            bg="#1f2329",
            fg="#f4f7fb",
        )
        title.pack(pady=(18, 4))

        frame = tk.Frame(self, bg="#1f2329")
        frame.pack(pady=16)

        for idx, (label, script, bg, fg) in enumerate(BUTTONS):
            row = idx // COLUMNS_PER_ROW
            col = idx % COLUMNS_PER_ROW
            button = tk.Button(
                frame,
                text=label,
                command=lambda name=label, path=script: self.run_script(name, path),
                width=16,
                height=3,
                bg=bg,
                fg=fg,
                activebackground=bg,
                activeforeground=fg,
                relief=tk.RAISED,
                borderwidth=2,
                font=("Liberation Mono", 11, "bold"),
            )
            button.grid(row=row, column=col, padx=8, pady=8)

        self.status = tk.Label(
            self,
            text="Ready",
            font=("Liberation Mono", 10),
            bg="#1f2329",
            fg="#d1d8e0",
        )
        self.status.pack(pady=(4, 0))

    def run_script(self, name, script):
        try:
            subprocess.Popen([str(script)], cwd=str(ROOT))
        except Exception as exc:
            self.status.config(text=f"{name} failed.")
            messagebox.showerror("ScriptPanel", f"Could not run {name}.\n\n{exc}")
            return

        self.status.config(text=f"{name} activated.")


if __name__ == "__main__":
    ConnectFourPanel().mainloop()
