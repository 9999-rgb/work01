"""Regression for crashes hidden by successful operation endpoints."""
import importlib.util
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location("two_scene_audit", Path(__file__).with_name("validate_two_scene_web.py"))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.mark.parametrize("line", [
    "[move_group-16] Segmentation fault (Address not mapped to object [0x7c81800ed798])",
    "[ERROR] [move_group-16]: process has died [pid 2888531, exit code -11]",
    'Thread 1 "gzserver" received signal SIGABRT, Aborted.',
    'Thread 2 "gzserver" received signal SIGSEGV, Segmentation fault.',
])
def test_crashed_retired_node_fails_acceptance(tmp_path, line):
    log = tmp_path / "runtime.log"
    log.write_text("Goal reached, success!\n" + line + "\n")
    result = module.audit_runtime_shutdown(log)
    assert not result["success"]
    assert result["fatal_lines"] == [line]


@pytest.mark.parametrize("line", [
    "[ERROR] [amcl-18]: process has died [pid 2888535, exit code -2]",
    "[move_group-16] [ERROR] Computed path is not valid. Invalid states at index [73]",
])
def test_intentional_interrupt_and_rejected_candidate_are_not_crashes(tmp_path, line):
    log = tmp_path / "runtime.log"
    log.write_text(line + "\n")
    assert module.audit_runtime_shutdown(log)["success"]


def test_missing_log_does_not_silently_pass(tmp_path):
    with pytest.raises(FileNotFoundError):
        module.audit_runtime_shutdown(tmp_path / "missing.log")
