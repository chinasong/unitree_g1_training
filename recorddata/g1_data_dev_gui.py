import sys
import argparse
import csv
import datetime
import time

from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout, QFrame
)
from PySide6.QtWidgets import QPushButton
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

class ArmDevGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("G1 Arm Joint Monitor (read-only)")
        self.resize(880, 600)

        self.label_map = {}   # idx -> QLabel
        self.name_map = {}    # idx -> name

        layout = QHBoxLayout()
        layout.setSpacing(12)
        self.setLayout(layout)

        layout.addLayout(self._make_column("Left Leg", LEFT_LEG))
        layout.addWidget(self._vline())

        # ───── Left Arm ─────
        layout.addLayout(self._make_column("Left Arm", LEFT_ARM))
        layout.addWidget(self._vline())

        # ───── Waist ─────
        layout.addLayout(self._make_column("Waist", WAIST))
        layout.addWidget(self._vline())

        # ───── Right Arm ─────
        layout.addLayout(self._make_column("Right Arm", RIGHT_ARM))
        layout.addWidget(self._vline())

        layout.addLayout(self._make_column("Right Leg", RIGHT_LEG))

        # ───── DDS Subscribe ─────
        self.state = None
        self.ready = False
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)

        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.update_state, 1)

        self.record_data = []  # 用于存储采集到的数据
        self.recording = False  # 是否正在记录

        # 添加一个记录按钮（可选）
        self.record_button = QPushButton("Start Recording")
        self.record_button.clicked.connect(self.toggle_recording)
        layout.addWidget(self.record_button)

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

        row = [now]  # 当前帧的时间戳
        for idx, label in self.label_map.items():
            m = self.state.motor_state[idx]
            label.setText(
                f"<span style='color: yellow;'>{idx:02d}</span> {self.name_map[idx]}: "
                f"{m.q:+.3f} <span style='color:gray;'>(dq: {m.dq:+.3f}, τ̂: {m.tau_est:+.3f})</span>"
            )

            # ✅ 每帧记录一次所有关节数据
            if self.recording:
                row.extend([m.q, m.dq, m.tau_est])

        if self.recording:
            self.record_data.append(row)

    def toggle_recording(self):
        self.recording = not self.recording
        if self.recording:
            self.record_data.clear()
            self.record_button.setText("Stop Recording")
        else:
            self.record_button.setText("Start Recording")
            self.save_recorded_data()

    def save_recorded_data(self):
        import csv, datetime
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"g1_joint_framewise_{timestamp}.csv"

        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)

            # 构造表头
            header = ["time"]
            for idx in self.label_map:
                name = self.name_map[idx]
                header += [f"{name}_q", f"{name}_dq", f"{name}_tau"]
            writer.writerow(header)

            # 写入数据
            writer.writerows(self.record_data)

        print(f"✅ Framewise joint data saved to {filename}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", required=True, help="Network interface connected to G1")
    args = parser.parse_args()

    ChannelFactoryInitialize(0, args.iface)

    app = QApplication(sys.argv)
    gui = ArmDevGUI()
    gui.show()
    sys.exit(app.exec())