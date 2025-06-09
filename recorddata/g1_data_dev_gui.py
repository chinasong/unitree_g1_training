import sys
import argparse
import datetime
import time
import csv

from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout, QFrame, QPushButton, QGridLayout
)
from PySide6.QtCore import QTimer, Qt

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

LEFT_ARM = {15: "L_SHOULDER_PITCH", 16: "L_SHOULDER_ROLL", 17: "L_SHOULDER_YAW", 18: "L_ELBOW", 19: "L_WRIST_ROLL", 20: "L_WRIST_PITCH", 21: "L_WRIST_YAW"}
RIGHT_ARM = {22: "R_SHOULDER_PITCH", 23: "R_SHOULDER_ROLL", 24: "R_SHOULDER_YAW", 25: "R_ELBOW", 26: "R_WRIST_ROLL", 27: "R_WRIST_PITCH", 28: "R_WRIST_YAW"}
WAIST = {12: "WAIST_YAW"}
LEFT_LEG = {0: "L_LEG_HIP_PITCH", 1: "L_LEG_HIP_ROLL", 2: "L_LEG_HIP_YAW", 3: "L_LEG_KNEE", 4: "L_LEG_ANKLE_PITCH", 5: "L_LEG_ANKLE_ROLL"}
RIGHT_LEG = {6: "R_LEG_HIP_PITCH", 7: "R_LEG_HIP_ROLL", 8: "R_LEG_HIP_YAW", 9: "R_LEG_KNEE", 10: "R_LEG_ANKLE_PITCH", 11: "R_LEG_ANKLE_ROLL"}

class ArmDevGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("G1 Joint Monitor and Recorder")
        self.resize(1100, 700)

        self.label_map = {}  # idx -> QLabel
        self.name_map = {}   # idx -> name
        self.state = None
        self.ready = False

        self.recording_flags = {
            "left_arm": False,
            "right_arm": False,
            "waist": False,
            "left_leg": False,
            "right_leg": False,
        }
        self.record_data = {
            "left_arm": [],
            "right_arm": [],
            "waist": [],
            "left_leg": [],
            "right_leg": [],
        }

        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.update_state, 1)

        self.layout = QGridLayout()
        self.setLayout(self.layout)

        self._add_group(0, 0, "Left Arm", LEFT_ARM, "left_arm")
        self._add_group(1, 0, "Right Arm", RIGHT_ARM, "right_arm")
        self._add_group(0, 1, "Left Leg", LEFT_LEG, "left_leg")
        self._add_group(1, 1, "Right Leg", RIGHT_LEG, "right_leg")
        self._add_group(0, 2, "Waist", WAIST, "waist")

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)

    def _add_group(self, row, col, title, joint_map, name_key):
        group = QVBoxLayout()

        header = QLabel(f"<b>{title}</b>")
        header.setAlignment(Qt.AlignCenter)
        header.setStyleSheet("font-size: 16px;")
        group.addWidget(header)

        for idx, name in joint_map.items():
            lbl = QLabel()
            lbl.setStyleSheet("font-family: monospace; font-size: 14px;")
            self.label_map[idx] = lbl
            self.name_map[idx] = name
            group.addWidget(lbl)

        btn = QPushButton(f"Start {title}")
        btn.setCheckable(True)
        btn.clicked.connect(lambda checked, key=name_key, b=btn: self.toggle_record(key, b))
        group.addWidget(btn)

        self.layout.addLayout(group, row, col)

    def update_state(self, msg: LowState_):
        self.state = msg
        self.ready = True

    def refresh(self):
        if not self.ready or self.state is None:
            return

        now = time.time()

        for idx, label in self.label_map.items():
            m = self.state.motor_state[idx]
            label.setText(
                f"<span style='color: yellow;'>{idx:02d}</span> {self.name_map[idx]}: "
                f"{m.q:+.3f} <span style='color:gray;'>(dq: {m.dq:+.3f}, τ̂: {m.tau_est:+.3f})</span>"
            )

        for key, flag in self.recording_flags.items():
            if flag:
                row = [now]
                for idx in self.get_joint_map(key):
                    m = self.state.motor_state[idx]
                    row += [m.q, m.dq, m.tau_est]
                self.record_data[key].append(row)

    def toggle_record(self, key, button: QPushButton):
        if not self.recording_flags[key]:
            self.record_data[key].clear()
            self.recording_flags[key] = True
            button.setText(f"Stop {button.text()[6:]}")
        else:
            self.recording_flags[key] = False
            button.setText(f"Start {button.text()[5:]}")
            self.save_csv(key)

    def get_joint_map(self, key):
        return {
            "left_arm": LEFT_ARM,
            "right_arm": RIGHT_ARM,
            "waist": WAIST,
            "left_leg": LEFT_LEG,
            "right_leg": RIGHT_LEG,
        }[key]

    def save_csv(self, key):
        now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"g1_{key}_{now}.csv"
        joint_map = self.get_joint_map(key)

        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            header = ["time"] + [
                f"{self.name_map[idx]}_{suf}" for idx in joint_map for suf in ("q", "dq", "tau")
            ]
            writer.writerow(header)
            writer.writerows(self.record_data[key])

        print(f"✅ Saved {filename} with {len(self.record_data[key])} frames.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", required=True, help="Network interface connected to G1")
    args = parser.parse_args()

    ChannelFactoryInitialize(0, args.iface)

    app = QApplication(sys.argv)
    gui = ArmDevGUI()
    gui.show()
    sys.exit(app.exec())