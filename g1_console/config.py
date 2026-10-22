from dataclasses import dataclass, field
import os
from pathlib import Path
import secrets

ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Settings:
    host: str = field(default_factory=lambda: os.getenv("G1_HOST", "127.0.0.1"))
    port: int = field(default_factory=lambda: int(os.getenv("G1_PORT", "8765")))
    token: str = field(default_factory=lambda: os.getenv("G1_API_TOKEN") or secrets.token_urlsafe(32))
    demo: bool = False
    read_only: bool = field(default_factory=lambda: os.getenv("G1_READ_ONLY", "").lower() in ("1", "true", "yes"))
    manual_remote: bool = field(default_factory=lambda: os.getenv("G1_MANUAL_REMOTE", "").lower() in ("1", "true", "yes"))
    verify_only: bool = field(default_factory=lambda: os.getenv("G1_VERIFY_ONLY", "").lower() in ("1", "true", "yes"))
    accessory_test: bool = field(default_factory=lambda: os.getenv("G1_ACCESSORY_TEST", "").lower() in ("1", "true", "yes"))
    damping_test: bool = field(default_factory=lambda: os.getenv("G1_DAMPING_TEST", "").lower() in ("1", "true", "yes"))
    camera_backend: str = field(default_factory=lambda: os.getenv("G1_CAMERA_BACKEND", "off"))
    camera_width: int = 960
    camera_fps: int = 5
    telemetry_ttl: float = 0.6
    sensor_ttl: float = 0.7
    lease_ttl: float = 2.0
    velocity_ttl: float = 0.25
    radius: float = 0.32
    data_dir: Path = field(default_factory=lambda: Path(os.getenv("G1_DATA_DIR", str(ROOT / ".g1-console"))))
    static_dir: Path = ROOT / "console/web/dist"
    allowed_hosts: tuple[str, ...] = field(default_factory=lambda: tuple(h.strip() for h in os.getenv("G1_ALLOWED_HOSTS", "").split(",") if h.strip()))

    def validate(self):
        if self.damping_test and (not self.manual_remote or self.read_only or self.verify_only or self.accessory_test):
            raise ValueError("单次阻尼测试需要独立人工遥控范围，不能与只读、接口验证或声光测试混用")
        if self.accessory_test and (not self.manual_remote or self.read_only or self.verify_only):
            raise ValueError("声光测试需要人工遥控模式，且不能同时设置只读或接口验证模式")
        if self.verify_only and (not self.manual_remote or self.read_only):
            raise ValueError("接口验证模式需要人工遥控模式，且不能同时设置只读模式")
        if self.camera_backend not in ("off", "unitree") or not 320 <= self.camera_width <= 1920 or not 1 <= self.camera_fps <= 10:
            raise ValueError("相机配置无效：G1_CAMERA_BACKEND 仅支持 off / unitree，宽度 320–1920，采样 1–10 Hz")
        if not 1024 <= self.port <= 65535:
            raise ValueError("G1_PORT 必须在 1024–65535 之间")
        if self.host not in ("localhost", "127.0.0.1", "::1") and len(os.getenv("G1_API_TOKEN", "")) < 24:
            raise ValueError("监听局域网前，请设置至少 24 字符的 G1_API_TOKEN；推荐使用 SSH 端口转发")
        if any("*" in host or "/" in host or "://" in host for host in self.allowed_hosts):
            raise ValueError("G1_ALLOWED_HOSTS 仅接受明确的主机名 / IP，不接受通配符或 URL")
        if self.host in ("0.0.0.0", "::") and not self.allowed_hosts:
            raise ValueError("监听所有网卡需要 G1_ALLOWED_HOSTS 明确列出访问主机名 / IP；推荐 SSH 转发")


class ConsoleError(Exception):
    def __init__(self, code: str, message: str, status: int = 409, *, sdk_code=None):
        self.code, self.message, self.status = code, message, status
        self.sdk_code = sdk_code
        super().__init__(message)
