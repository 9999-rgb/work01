"""控制网关 API 的 Pydantic v2 请求模型。

字段名与旧版 ``web_server.py`` 完全一致（保持前端契约），仅用 Pydantic 取代
手工校验。全部 ``extra="forbid"``，等价于旧的 ``_reject_unknown_fields``。
"""

from __future__ import annotations

from typing import Literal

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    StrictBool,
    StrictFloat,
    StrictInt,
    StrictStr,
    field_validator,
    model_validator,
)

from app.api.validators import finite_number, nonempty_string

# 数值字段使用 StrictFloat：接受 int/float，拒绝 bool（旧版语义）。
_NUM = StrictFloat


class NavigateRequest(BaseModel):
    """POST /task/navigate 请求体。"""

    model_config = ConfigDict(extra="forbid")

    cabinet: str = Field(description="柜体实例名称，见 cabinet_instances.yaml")
    control_id: str | None = Field(
        default=None,
        description="可选：控件 ID，使用该控件的专用导航工位",
    )

    @field_validator("cabinet")
    @classmethod
    def _cabinet(cls, value: str) -> str:
        return nonempty_string(value, "cabinet")

    @field_validator("control_id")
    @classmethod
    def _control_id(cls, value: str | None) -> str | None:
        return nonempty_string(value, "control_id") if value is not None else None


class OperateRequest(BaseModel):
    """POST /task/operate 请求体。"""

    model_config = ConfigDict(extra="forbid")

    cabinet: str = Field(description="柜体实例名称")
    control_id: str = Field(description="控件 ID，如 box_8_button_1")
    command: Literal["press", "set_state", "set_position", "toggle"] = Field(
        description="操作命令：press / set_state / set_position / toggle"
    )
    target_state: str | None = Field(
        default=None,
        description="set_state 时的目标档位",
    )
    target_position: StrictFloat | None = Field(
        default=None,
        description="set_position 时的目标位置",
    )
    force: StrictFloat | None = Field(
        default=None,
        gt=0,
        description="按压力（N）。省略时使用控件目录 default_force",
    )

    @field_validator("cabinet")
    @classmethod
    def _cabinet(cls, value: str) -> str:
        return nonempty_string(value, "cabinet")

    @field_validator("control_id")
    @classmethod
    def _control_id(cls, value: str) -> str:
        return nonempty_string(value, "control_id")

    @field_validator("target_state")
    @classmethod
    def _target_state(cls, value: str | None) -> str | None:
        return nonempty_string(value, "target_state") if value is not None else None

    @field_validator("target_position")
    @classmethod
    def _target_position(cls, value: float | None) -> float | None:
        return finite_number(value, "target_position") if value is not None else None

    @field_validator("force")
    @classmethod
    def _force(cls, value: float | None) -> float | None:
        return finite_number(value, "force") if value is not None else None

    @model_validator(mode="after")
    def _command_targets(self) -> "OperateRequest":
        """Keep the command-specific target contract of the legacy API."""
        provided = self.model_fields_set
        if self.command == "set_state":
            if self.target_state is None:
                raise ValueError("target_state is required for set_state.")
            if "target_position" in provided:
                raise ValueError(
                    "target_position is only valid for set_position."
                )
        elif self.command == "set_position":
            if self.target_position is None:
                raise ValueError(
                    "target_position is required for set_position."
                )
            if "target_state" in provided:
                raise ValueError("target_state is only valid for set_state.")
        elif {"target_state", "target_position"} & provided:
            raise ValueError(
                "target_state and target_position are not valid for "
                f"{self.command}."
            )

        # The old handler treated an omitted force as "use catalog default",
        # while an explicitly supplied null still failed numeric validation.
        # Force itself was allowed for every command and remains so here.
        if "force" in provided and self.force is None:
            raise ValueError("force is required.")
        return self


class TaskResetRequest(BaseModel):
    """POST /task/reset 请求体。"""

    model_config = ConfigDict(extra="forbid")

    cabinet: str = Field(description="要重置的柜体实例名称")

    @field_validator("cabinet")
    @classmethod
    def _cabinet(cls, value: str) -> str:
        return nonempty_string(value, "cabinet")


class CmdVelRequest(BaseModel):
    """POST /cmd_vel 请求体。

    前端发送 6 分量（linear_x/y/z, angular_x/y/z），后端只取 linear_y
    与 angular_z。旧版 web_server.py 未拒多余字段，此处保持兼容。
    """

    model_config = ConfigDict(extra="ignore")

    linear_y: StrictFloat = Field(default=0.0, description="前后速度 (m/s)")
    angular_z: StrictFloat = Field(default=0.0, description="转向角速度 (rad/s)")

    @field_validator("linear_y", "angular_z")
    @classmethod
    def _number(cls, value: float) -> float:
        return finite_number(value, "linear_y/angular_z")


class JointTrajectoryRequest(BaseModel):
    """POST /joint_trajectory 请求体。"""

    model_config = ConfigDict(extra="forbid")

    positions: list[StrictFloat] = Field(
        min_length=1,
        description="各关节目标位置（数量须与适配器 joint_count 一致）",
    )

    @field_validator("positions")
    @classmethod
    def _positions(cls, values: list[float]) -> list[float]:
        if not values:
            raise ValueError("positions 必须是非空数字列表")
        return [finite_number(v, "positions[i]") for v in values]


class NavigationModeRequest(BaseModel):
    """POST /navigation/mode 请求体。"""

    model_config = ConfigDict(extra="forbid")

    enabled: StrictBool = Field(description="是否启用 Nav2 导航模式")


class CabinetPressRequest(BaseModel):
    """POST /cabinet/press 请求体。"""

    model_config = ConfigDict(extra="forbid")

    button_id: str = Field(description="按钮 ID")
    navigate_to_staging_pose: StrictBool = Field(
        default=True,
        description="操作前是否先导航到工位",
    )

    @field_validator("button_id")
    @classmethod
    def _button_id(cls, value: str) -> str:
        return nonempty_string(value, "button_id")


class CabinetOperateRequest(BaseModel):
    """POST /cabinet/operate 请求体（旧版单柜兼容接口）。"""

    model_config = ConfigDict(extra="forbid")

    control_id: str = Field(description="控件 ID")
    command: StrictStr | StrictInt = Field(description="命令码（字符串或整数）")
    target_state: str | None = Field(default=None)
    target_position: StrictFloat | None = Field(default=None)
    navigate_to_staging_pose: StrictBool = Field(default=True)

    @field_validator("control_id")
    @classmethod
    def _control_id(cls, value: str) -> str:
        return nonempty_string(value, "control_id")

    @field_validator("target_state")
    @classmethod
    def _target_state(cls, value: str | None) -> str | None:
        return nonempty_string(value, "target_state") if value is not None else None

    @field_validator("target_position")
    @classmethod
    def _target_position(cls, value: float | None) -> float | None:
        return finite_number(value, "target_position") if value is not None else None


class RecordingStartRequest(BaseModel):
    """POST /recording/start 请求体。"""

    model_config = ConfigDict(extra="forbid")

    name: str | None = Field(
        default=None,
        description="录制名称（安全 ID：小写字母数字、连字符、下划线，≤64 字符）",
    )
    include_sensors: StrictBool = Field(
        default=False,
        description="是否包含传感器话题",
    )


class DataPlaybackStartRequest(BaseModel):
    """POST /replay/data/start 请求体。"""

    model_config = ConfigDict(extra="forbid")

    recording_id: str = Field(description="录制 ID")
    rate: StrictFloat = Field(default=1.0, gt=0, description="回放倍速（0.1-10.0）")

    @field_validator("recording_id")
    @classmethod
    def _recording_id(cls, value: str) -> str:
        return nonempty_string(value, "recording_id")

    @field_validator("rate")
    @classmethod
    def _rate(cls, value: float) -> float:
        return finite_number(value, "rate")


class DataPlaybackRateRequest(BaseModel):
    """POST /replay/data/rate 请求体。"""

    model_config = ConfigDict(extra="forbid")

    rate: StrictFloat = Field(gt=0, description="回放倍速")

    @field_validator("rate")
    @classmethod
    def _rate(cls, value: float) -> float:
        return finite_number(value, "rate")


class TaskReplayStartRequest(BaseModel):
    """POST /replay/task/start 请求体。"""

    model_config = ConfigDict(extra="forbid")

    recording_id: str = Field(description="录制 ID")

    @field_validator("recording_id")
    @classmethod
    def _recording_id(cls, value: str) -> str:
        return nonempty_string(value, "recording_id")


class SceneSwitchRequest(BaseModel):
    """POST /scene/switch 请求体。"""

    model_config = ConfigDict(extra="forbid")

    name: str = Field(description="目标场景标识，见 scenes.yaml")

    @field_validator("name")
    @classmethod
    def _name(cls, value: str) -> str:
        return nonempty_string(value, "name")


class AssetSelectionRequest(BaseModel):
    """POST /assets/selection 请求体。

    三个字段均可选；``null`` 或空串表示该维度不选择（保持现状）。
    名称将在后端再次校验：scene / cabinet 必须已在资产库 catalog 中。
    ``toolset`` 只能是 A / B（大小写不敏感），null 表示不改变。
    """

    model_config = ConfigDict(extra="forbid")

    scene: str | None = Field(
        default=None, description="场景资产名；null 表示不选择场景"
    )
    cabinet: str | None = Field(
        default=None, description="柜体资产名；null 表示不选择柜体"
    )
    toolset: str | None = Field(
        default=None, description="末端工具套装 A/B；null 表示不改变"
    )

    @field_validator("scene", "cabinet")
    @classmethod
    def _name(cls, value: str | None) -> str | None:
        if value is None:
            return None
        if not isinstance(value, str):
            raise ValueError("必须是字符串或 null")
        stripped = value.strip()
        return stripped or None

    @field_validator("toolset")
    @classmethod
    def _toolset(cls, value: str | None) -> str | None:
        if value is None:
            return None
        if not isinstance(value, str):
            raise ValueError("必须是字符串或 null")
        normalized = value.strip().upper()
        if normalized not in ("A", "B"):
            raise ValueError("toolset 只能是 A 或 B")
        return normalized


class ToolsetSwitchRequest(BaseModel):
    """POST /robot/toolset/switch 的乐观并发切换请求。"""

    model_config = ConfigDict(extra="forbid")

    toolset: StrictStr = Field(description="目标末端工具套装 A 或 B")
    expected_generation: StrictInt = Field(
        default=0,
        ge=0,
        description="最近一次 /robot/toolset/status 的 generation；0 表示不校验",
    )

    @field_validator("toolset")
    @classmethod
    def _toolset(cls, value: str) -> str:
        normalized = nonempty_string(value, "toolset").upper()
        if normalized not in {"A", "B"}:
            raise ValueError("toolset 只能是 A 或 B")
        return normalized
