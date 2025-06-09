import sys
import argparse
import csv
import datetime
import time

from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout, QFrame, QPushButton
)
from PySide6.QtCore import QTimer, Qt

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

LEFT_ARM = {
    15: "L_SHOULDER_PITCH", 16: "L_SHOULDER_ROLL", 17: "L_SHOULDER_YAW",
    18: "L_ELBOW", 19: "L_WRIST_ROLL", 20: "L_WRIST_PITCH", 21: "L_WRIST_YAW"
}
WAIST = {12: "WAIST_YAW"}
RIGHT_ARM = {
    22: "R_SHOULDER_PITCH", 23: "R_SHOULDER_ROLL", 24: "R_SHOULDER_YAW",
    25: "R_ELBOW", 26: "R_WRIST_ROLL", 27: "R_WRIST_PITCH", 28: "R_WRIST_YAW"
}
LEFT_LEG = {
    0: "L_LEG_HIP_PITCH", 1: "L_LEG_HIP_ROLL", 2: "L_LEG_HIP_YAW",
    3: "L_LEG_KNEE", 4: "L_LEG_ANKLE_PITCH", 5: "L_LEG_ANKLE_ROLL"
}
RIGHT_LEG = {
    6: "R_LEG_HIP_PITCH", 7: "R_LEG_HIP_ROLL", 8: "R_LEG_HIP_YAW",
    9: "R_LEG_KNEE", 10: "R_LEG_ANKLE_PITCH", 11: "R_LEG_ANKLE_ROLL"
}

ALL_PARTS = {
    "left_arm": LEFT_ARM,
    "right_arm": RIGHT_ARM,
    "waist": WAIST,
    "left_leg": LEFT_LEG,
    "right_leg": RIGHT_LEG
}

class ArmDevGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("G1 Joint Recorder")
        self.resize(960, 600)

        self.label_map = {}
        self.name_map = {}
        self.layout = QHBoxLayout()
        self.layout.setSpacing(12)
        self.setLayout(self.layout)

        for title, part in ALL_PARTS.items():
            self.layout.addLayout(self._make_column(title.replace("_", " ").title(), part))
            self.layout.addWidget(self._vline())

        # buttons
        self.button_col = QVBoxLayout()
        for part_name in ALL_PARTS:
            btn = QPushButton(f"Save {part_name.replace('_', ' ').title()} CSV")
            btn.clicked.connect(lambda checked=False, name=part_name: self.save_part_csv(name))
            self.button_col.addWidget(btn)
        self.layout.addLayout(self.button_col)

        # dds sub
        self.state = None
        self.ready = False
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)

        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.update_state, 1)
        self.current_row = []

    def _make_column(self, title, joint_map):
        col = QVBoxLayout()
        header = QLabel(f"<b>{title}</b>")
        header.setAlignment(Qt.AlignCenter)
        header.setStyleSheet("font-size: 16px; margin-bottom: 10px;")
        col.addWidget(header)

        for idx, name in joint_map.items():
            lbl = QLabel()
            lbl.setTextFormat(Qt.RichText)
            lbl.setStyleSheet("font-family: monospace; font-size: 14px;")
            self.label_map[idx] = lbl
            self.name_map[idx] = name
            col.addWidget(lbl)

        col.addStretch()
        return col

    def _vline(self):
        line = QFrame()
        line.setFrameShape(QFrame.VLine)
        line.setFrameShadow(QFrame.Sunken)
        line.setFixedWidth(2)
        return line

    def update_state(self, msg: LowState_):
        self.state = msg
        self.ready = True

    def refresh(self):
        if not self.ready or self.state is None:
            return

        now = time.time()
        self.current_row = [now]
        for idx, label in self.label_map.items():
            m = self.state.motor_state[idx]
            label.setText(
                f"<span style='color: yellow;'>{idx:02d}</span> {self.name_map[idx]}: "
                f"{m.q:+.3f} <span style='color:gray;'>(dq: {m.dq:+.3f}, τ̂: {m.tau_est:+.3f})</span>"
            )
            self.current_row.extend([m.q, m.dq, m.tau_est])

    def save_part_csv(self, part_name):
        if not self.current_row:
            print("❌ 当前没有可保存的数据帧。")
            return

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"g1_{part_name}_{timestamp}.csv"
        joint_map = ALL_PARTS[part_name]

        header = ["time"]
        values = [self.current_row[0]]
        for idx in joint_map:
            name = self.name_map[idx]
            i = list(self.label_map.keys()).index(idx)
            values += self.current_row[1 + i * 3: 1 + (i + 1) * 3]
            header += [f"{name}_q", f"{name}_dq", f"{name}_tau"]

        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerow(values)

        print(f"✅ {part_name} 数据已保存为 {filename}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", required=True, help="Network interface connected to G1")
    args = parser.parse_args()

    ChannelFactoryInitialize(0, args.iface)
    app = QApplication(sys.argv)
    gui = ArmDevGUI()
    gui.show()
    sys.exit(app.exec())
