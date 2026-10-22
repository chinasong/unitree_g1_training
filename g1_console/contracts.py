from typing import Annotated, Literal
from pydantic import BaseModel, ConfigDict, Field, model_validator

Finite = Annotated[float, Field(allow_inf_nan=False)]


class Contract(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)


class Connect(Contract):
    mode: Literal["demo", "robot"]
    interface: str = Field(default="eth0", pattern=r"^[a-zA-Z0-9_.:-]{1,40}$")


class Owner(Contract):
    owner: str = Field(pattern=r"^[a-zA-Z0-9-]{16,64}$")


class Acquire(Owner):
    floor_verified: bool = False


class Preflight(Owner):
    capability: str = Field(max_length=80)
    params: dict = Field(default_factory=dict)


class ManualArm(Owner):
    # These are operator observations, never inferred foot loads or foundation values.
    standing_observed: Literal[True]
    unsupported_observed: Literal[True]
    area_clear_observed: Literal[True]
    floor_verified: Literal[True]

    @model_validator(mode="before")
    @classmethod
    def actual_confirmations(cls, data):
        if isinstance(data, dict) and any(data.get(k) is not True for k in ("standing_observed", "unsupported_observed", "area_clear_observed", "floor_verified")):
            raise ValueError("现场确认必须逐项为 true")
        return data



class SequenceStart(Owner):
    # Operator observations for the official startup / shutdown route; never inferred from sensors.
    route: Literal["seat", "squat"]
    kind: Literal["start", "stop"]
    posture_observed: Literal[True]
    protection_ready: Literal[True]
    area_clear_observed: Literal[True]
    remote_in_hand: Literal[True]

    @model_validator(mode="before")
    @classmethod
    def actual_confirmations(cls, data):
        if isinstance(data, dict) and any(data.get(k) is not True for k in ("posture_observed", "protection_ready", "area_clear_observed", "remote_in_hand")):
            raise ValueError("现场确认必须逐项为 true")
        return data


class SequenceContinue(Owner):
    upright_held: Literal[True]


class Command(Owner):
    capability: str = Field(max_length=80)
    params: dict = Field(default_factory=dict)
    acknowledge_risk: bool = False
    request_id: str = Field(pattern=r"^[a-zA-Z0-9-]{16,64}$")
    issued_at: Finite
    sequence: int = Field(ge=0)


class Goal(Owner):
    x: Finite = Field(ge=-100, le=100)
    y: Finite = Field(ge=-100, le=100)
    request_id: str = Field(pattern=r"^[a-zA-Z0-9-]{16,64}$")
    issued_at: Finite
    sequence: int = Field(ge=0)


class Pose(Contract):
    x: Finite = Field(ge=-100, le=100)
    y: Finite = Field(ge=-100, le=100)
    yaw: Finite = Field(ge=-3.142, le=3.142)


class Grid(Contract):
    frame: str = Field(min_length=1, max_length=64)
    width: int = Field(ge=4, le=160)
    height: int = Field(ge=4, le=160)
    resolution: Finite = Field(ge=0.025, le=1)
    origin: tuple[Finite, Finite]
    cells: list[int] = Field(max_length=25600)
    revision: int = Field(ge=0)

    @model_validator(mode="after")
    def dimensions(self):
        if len(self.cells) != self.width * self.height or any(v < -1 or v > 100 for v in self.cells):
            raise ValueError("栅格尺寸不匹配或占用值超出 -1..100")
        return self


class Observation(Contract):
    source: str = Field(min_length=1, max_length=64)
    frame: str = Field(min_length=1, max_length=64)
    sequence: int = Field(ge=0)
    captured_at: Finite
    pose: Pose
    # Robot base frame: +x forward, +y left. Only finite returns, never inf/NaN.
    scan: list[tuple[Finite, Finite]] = Field(min_length=8, max_length=2160)
    coverage_radians: Finite = Field(ge=0, le=6.284)
    grid: Grid | None = None


class CameraFrame(Contract):
    source: str = Field(min_length=1, max_length=64)
    sequence: int = Field(ge=0)
    captured_at: Finite
    jpeg_base64: str = Field(max_length=700000)


# Project speed envelope; the only place these limits are defined.
MAX_VX, MAX_VY, MAX_OMEGA = 0.3, 0.2, 0.4


class Velocity(Contract):
    vx: Finite = Field(default=0, ge=-MAX_VX, le=MAX_VX)
    vy: Finite = Field(default=0, ge=-MAX_VY, le=MAX_VY)
    omega: Finite = Field(default=0, ge=-MAX_OMEGA, le=MAX_OMEGA)


class OneInt(Contract):
    value: int = Field(strict=True)


class Scalar(Contract):
    value: Finite


class Text(Contract):
    text: str = Field(min_length=1, max_length=300)
    speaker_id: int = Field(default=0, ge=0, le=1, strict=True)


class RGB(Contract):
    r: int = Field(ge=0, le=255, strict=True)
    g: int = Field(ge=0, le=255, strict=True)
    b: int = Field(ge=0, le=255, strict=True)


class PCM(Contract):
    # PlayStop must target exactly the same application as PlayStream.
    app_name: Literal["g1-console"] = "g1-console"
    stream_id: str = Field(default="console", pattern=r"^[a-zA-Z0-9_-]{1,32}$")
    pcm_base64: str = Field(min_length=4, max_length=128000)
