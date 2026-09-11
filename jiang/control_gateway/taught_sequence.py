"""声明式「教学动作序列」执行器。

现场教学出来的动作（存成 ``xczs_inspection_robot_control/config/taught_poses/*.yaml``）
以**声明式步骤**描述在 ``taught_poses/db1_sequence.yaml`` 里，本模块负责加载、校验并
执行，供 Web 任务层使用：Web 上点"打开 db1" → 任务层命中本序列 → 逐步执行。

设计要点
--------
* **配置驱动**：步骤类型固定三种（preposition_base / restore_pose / pull_drawer），
  参数全部显式写在 YAML 里，缺失即报错、不做默认值兜底——宁可启动就炸，也不要
  现场改坏了却静默跑一半。
* **事件同形**：执行过程中往调用方给的事件队列里推 ``feedback`` / ``terminal``
  事件，字段与 ``CabinetClient`` 产出的一致（含 ``generation``）。这样任务管理、
  SSE 进度、超时与取消逻辑**一行都不用改**。
* **可取消**：``cancel()`` 置标志，工作线程在每步之间与抽拉过程中检查。

待办（明确的后续工作，不是遗留 bug）
------------------------------------
本模块与 ``scripts/tools/`` 下的现场工具共享控制器映射与笛卡尔规划逻辑。
目前通过 ``_bridge_tools_path()`` 复用 ``scripts/tools/xczs_controllers.py``；
正确做法是把这部分提升成一个独立的 Python 包（例如
``xczs_inspection_robot_taught``），工具与任务层同时依赖它。过渡期保留桥接，
避免现在就复制一份出来。
"""
from __future__ import annotations

import copy
import math
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, Mapping, Optional

import yaml

WORKSPACE = Path(__file__).resolve().parents[2]
TAUGHT_POSES_DIR = (
    WORKSPACE / "xczs_inspection_robot_control" / "config" / "taught_poses"
)
DEFAULT_SEQUENCE_FILE = TAUGHT_POSES_DIR / "db1_sequence.yaml"
SUPPORTED_STEPS = ("preposition_base", "restore_pose", "pull_drawer",
                    "retract_rods", "go_home", "translate_tool")
SUPPORTED_SCHEMA = 1


def _bridge_tools_path() -> None:
    """把 scripts/tools 挂进 sys.path，复用 xczs_controllers（见模块 docstring）。"""
    tools = str(WORKSPACE / "scripts" / "tools")
    if tools not in sys.path:
        sys.path.insert(0, tools)


class TaughtSequenceError(RuntimeError):
    """序列加载或执行失败。"""


# --------------------------------------------------------------------- 加载


def load_sequences(path: Optional[Path] = None) -> Dict[str, Dict[str, Dict[str, Any]]]:
    """读序列文件 → ``{control_id: {command: {"display_name", "steps"}}}``。"""
    path = Path(path) if path else DEFAULT_SEQUENCE_FILE
    if not path.is_file():
        raise TaughtSequenceError(f"教学序列文件不存在: {path}")
    with path.open("r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle)
    if not isinstance(document, Mapping):
        raise TaughtSequenceError(f"{path} 顶层必须是映射")
    version = document.get("schema_version")
    if version != SUPPORTED_SCHEMA:
        raise TaughtSequenceError(
            f"{path} schema_version={version!r}，本模块只认 {SUPPORTED_SCHEMA}"
        )
    raw = document.get("sequences")
    if not isinstance(raw, Mapping) or not raw:
        raise TaughtSequenceError(f"{path} 缺少非空的 sequences 段")

    sequences: Dict[str, Dict[str, Dict[str, Any]]] = {}
    for control_id, commands in raw.items():
        if not isinstance(commands, Mapping):
            raise TaughtSequenceError(f"{path}: sequences.{control_id} 必须是映射")
        sequences[str(control_id)] = {}
        for command, entry in commands.items():
            if not isinstance(entry, Mapping):
                raise TaughtSequenceError(
                    f"{path}: sequences.{control_id}.{command} 必须是映射"
                )
            steps = entry.get("steps")
            if not isinstance(steps, list) or not steps:
                raise TaughtSequenceError(
                    f"{path}: sequences.{control_id}.{command}.steps 必须是非空列表"
                )
            for index, step in enumerate(steps):
                where = f"sequences.{control_id}.{command}.steps[{index}]"
                if not isinstance(step, Mapping):
                    raise TaughtSequenceError(f"{path}: {where} 必须是映射")
                step_type = step.get("type")
                if step_type not in SUPPORTED_STEPS:
                    raise TaughtSequenceError(
                        f"{path}: {where}.type={step_type!r} 不支持"
                        f"（可用: {', '.join(SUPPORTED_STEPS)}）"
                    )
            sequences[str(control_id)][str(command)] = {
                "display_name": str(entry.get("display_name") or command),
                "steps": [dict(step) for step in steps],
            }
    return sequences


def find_sequence(
    sequences: Mapping[str, Mapping[str, Dict[str, Any]]],
    control_id: str,
    command: str,
    target_state: Optional[str],
) -> Optional[Dict[str, Any]]:
    """按 (控件, 命令) 找序列；``set_state`` 用 target_state 区分开/合。

    只有显式登记的组合才命中——没登记的一律返回 None，维持原路径。
    """
    by_command = sequences.get(control_id)
    if not by_command:
        return None
    if command == "set_state":
        if not target_state:
            return None
        return by_command.get(f"set_state:{target_state}") or by_command.get(
            target_state
        )
    return by_command.get(command)


# ------------------------------------------------------------------- 执行器


class TaughtSequenceRunner:
    """把一条序列跑完，并往事件队列推与 CabinetClient 同形的事件。

    参数
    ----
    emit          : 事件入队回调（调用方给的是 queue.Queue 的 put）。
    generation    : 与 submission 一起回给调用方的代数号，事件必须带上它，
                    否则监视线程会按"过期事件"丢弃。
    cabinet       : 场景实例名，用于拼服务/话题名。
    """

    def __init__(
        self,
        cabinet: str,
        control_id: str,
        command: str,
        target_state: Optional[str],
        steps: list,
        emit: Callable[[Dict[str, Any]], None],
        generation: int,
        display_name: str = "",
        sequence_dir: Path = TAUGHT_POSES_DIR,
        context: Any = None,
    ) -> None:
        self._context = context
        self._cabinet = cabinet
        self._control_id = control_id
        self._command = command
        self._target_state = target_state
        self._steps = steps
        self._emit = emit
        self._generation = generation
        self._display_name = display_name or command
        self._sequence_dir = Path(sequence_dir)
        self._cancel = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._finished = False

    # ------------------------------------------------------------- 对外接口
    def start(self) -> None:
        self._thread = threading.Thread(
            target=self._run, name=f"taught-{self._cabinet}-{self._control_id}",
            daemon=True,
        )
        self._thread.start()

    def cancel(self) -> None:
        self._cancel.set()

    def join(self, timeout: Optional[float] = None) -> None:
        if self._thread is not None:
            self._thread.join(timeout)

    # --------------------------------------------------------------- 事件
    def _feedback(self, phase: str, progress: float, message: str,
                  current_position: Optional[float] = None,
                  target_position: Optional[float] = None) -> None:
        if self._cancel.is_set() or self._finished:
            return
        self._emit({
            "event": "feedback",
            "cabinet": self._cabinet,
            "generation": self._generation,
            "timestamp": time.time(),
            "phase": phase,
            "phase_code": None,
            "progress": max(0.0, min(1.0, float(progress))),
            "message": message,
            "current_position": current_position,
            "target_position": target_position,
            "current_state": "",
        })

    def _terminal(self, outcome: str, message: str,
                  failure_code: Optional[str] = None,
                  result: Optional[Mapping[str, Any]] = None) -> None:
        self._finished = True
        failure_reason = None
        if outcome != "success":
            failure_reason = message
            if not failure_code:
                failure_code = "operation_failed"
        else:
            failure_code = None
        payload: Dict[str, Any] = {
            "event": "terminal",
            "cabinet": self._cabinet,
            "generation": self._generation,
            "timestamp": time.time(),
            "outcome": outcome,
            "success": outcome == "success",
            "message": message,
            "failure_code": failure_code,
            "failure_reason": failure_reason,
            "error_code": failure_code,
            "result": dict(result or {}),
        }
        self._emit(copy.deepcopy(payload))

    # --------------------------------------------------------------- 主流程
    def _run(self) -> None:
        started = time.monotonic()
        node = None
        try:
            _bridge_tools_path()
            from taught_sequence_ros import TaughtRosWorker  # noqa: WPS433

            self._feedback("starting", 0.02,
                           f"开始执行教学动作：{self._display_name}")
            node = TaughtRosWorker(self._cabinet, self._cancel,
                                   context=self._context)
            node.wait_ready()
            total = len(self._steps)
            for index, step in enumerate(self._steps):
                if self._cancel.is_set():
                    self._terminal("canceled", "教学动作被取消。")
                    return
                step_type = str(step.get("type"))
                base = index / float(total)
                span = 1.0 / float(total)
                label = f"步骤 {index + 1}/{total}：{step_type}"
                self._feedback("taught", base, f"{label} 开始")
                handler = getattr(node, f"step_{step_type}", None)
                if handler is None:
                    raise TaughtSequenceError(f"执行器没有实现步骤 {step_type}")
                handler(
                    step,
                    on_progress=lambda fraction, text, b=base, s=span: (
                        self._feedback("taught", b + s * float(fraction), text)
                    ),
                )
                self._feedback("taught", base + span, f"{label} 完成")

            elapsed = time.monotonic() - started
            # 与 operator 的结果字段保持同形，前端/录像的消费方不用区分来源。
            self._terminal(
                "success",
                f"教学动作完成：{self._display_name}（{elapsed:.1f} s）",
                result={
                    "cabinet": self._cabinet,
                    "control_id": self._control_id,
                    "command": self._command,
                    "execution_backend": "taught",
                    "duration_seconds": elapsed,
                },
            )
        except Exception as error:  # noqa: BLE001
            # 裸异常字符串（例如只有 "__enter__"）看不出处，必须留完整堆栈。
            import traceback

            detail = traceback.format_exc()
            print("教学动作异常详情:\n%s" % detail, flush=True)
            self._terminal("failed", f"教学动作失败：{error}",
                           result={"traceback": detail[-2000:]})
        finally:
            if node is not None:
                try:
                    node.shutdown()
                except Exception:  # noqa: BLE001
                    pass
