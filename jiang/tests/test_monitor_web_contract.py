"""Behavioral contract tests for the cabinet workflow in ``monitor.html``."""

from __future__ import annotations

import re
import shutil
import subprocess
import textwrap
import unittest
from pathlib import Path


MONITOR_PATH = Path(__file__).resolve().parents[1] / "monitor.html"


def _inline_script() -> str:
    html = MONITOR_PATH.read_text(encoding="utf-8")
    matches = re.findall(r"<script>(.*?)</script>", html, flags=re.DOTALL)
    if not matches:
        raise AssertionError("monitor.html does not contain an inline script")
    return matches[-1]


def _source_block(start_marker: str, end_marker: str) -> str:
    source = _inline_script()
    start = source.index(start_marker)
    end = source.index(end_marker, start)
    return source[start:end]


@unittest.skipUnless(shutil.which("node"), "Node.js is required for Web tests")
class MonitorWebContractTest(unittest.TestCase):
    maxDiff = None

    def run_node(self, source: str) -> None:
        result = subprocess.run(
            ["node", "-"],
            input=source,
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(
            0,
            result.returncode,
            msg=f"Node test failed:\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}",
        )

    def test_inline_javascript_parses(self) -> None:
        result = subprocess.run(
            ["node", "--check", "-"],
            input=_inline_script(),
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(0, result.returncode, msg=result.stderr)

    def test_all_33_controls_remain_selectable(self) -> None:
        apply_controls = _source_block(
            "function applyCabinetControls(payload)",
            "\nfunction renderCabinetTarget()",
        )
        self.run_node(textwrap.dedent(f"""
            const assert = require('node:assert/strict');
            class Element {{
              constructor(tag) {{
                this.tagName = tag;
                this.children = [];
                this.value = '';
                this.textContent = '';
                this.disabled = false;
              }}
              appendChild(child) {{ this.children.push(child); }}
              replaceChildren(...children) {{ this.children = children; }}
            }}
            const document = {{createElement: tag => new Element(tag)}};
            const cabinetTarget = new Element('select');
            cabinetTarget.value = 'limited_control';
            const cabinetControlTypeOrder = [0, 1, 2, 3];
            const cabinetControlTypeLabels = {{
              0: '按钮', 1: '旋钮', 2: '总开关', 3: '柜门'
            }};
            const cabinetCommandSupport = {{press: 1}};
            let cabinetControls = [];
            let cabinetCatalogReceived = false;
            let cabinetAvailableFromHealth = false;
            let cabinetOperationAvailable = false;
            let cabinetResetAvailable = false;
            let cabinetStatus = null;
            function isCabinetOperationActive() {{ return false; }}
            function renderCabinetTarget() {{}}
            function selectedCabinetName() {{ return 'cabinet_a'; }}
            {apply_controls}

            const controls = Array.from({{length: 33}}, (_, index) => ({{
              control_id: index === 17 ? 'limited_control' : `control_${{index}}`,
              display_name: `控件 ${{index}}`,
              control_type: index % 4,
              supported_commands: 1,
              operable: index !== 17,
              unavailable_reason: index === 17 ? '当前机器人运动学不可达' : ''
            }}));
            assert.equal(applyCabinetControls({{
              cabinet: 'cabinet_a',
              catalog_received: true,
              selected_control_id: 'control_0',
              controls
            }}), true);
            assert.equal(cabinetControls.length, 33);
            assert.equal(cabinetTarget.value, 'limited_control');
            const options = cabinetTarget.children.flatMap(group => group.children);
            assert.equal(options.length, 33);
            const limited = options.find(option => option.value === 'limited_control');
            assert.ok(limited);
            assert.equal(limited.disabled, false);
            assert.match(limited.textContent, /需实时规划验证/);
            assert.doesNotMatch(limited.textContent, /不可操作/);
        """))

    def test_every_control_requires_navigation_and_action_backends(self) -> None:
        update_interlocks = _source_block(
            "function updateControlInterlocks()",
            "\nconst manualTakeoverRequestTimeoutMs",
        )
        self.run_node(textwrap.dedent(f"""
            const assert = require('node:assert/strict');
            class Element {{
              constructor() {{
                this.disabled = false;
                this.value = '';
              }}
            }}
            const elements = new Map();
            const $ = id => {{
              if (!elements.has(id)) elements.set(id, new Element());
              return elements.get(id);
            }};
            const document = {{querySelectorAll: () => []}};
            const cabinetTarget = new Element();
            const cabinetInstance = new Element();
            const navCabinetInstance = new Element();
            let selected = {{control_id: 'limited', control_type: 1, operable: false}};
            let ctrlConnected = true;
            let cabinetCatalogReceived = true;
            let cabinetOperationAvailable = true;
            let cabinetResetAvailable = true;
            let cabinetAvailable = true;
            let cabinetInterlockWasActive = false;
            let manualTakeoverState = 'idle';
            const navigationStatus = {{available: false}};
            const cabinetStatus = null;
            const cabinetInstances = [{{name: 'cabinet_a'}}];
            const armJointSpecs = [];
            const gripperJointSpecs = [];
            function selectedCabinetControl() {{ return selected; }}
            function selectedCabinetName() {{ return 'cabinet_a'; }}
            function isCabinetAvailable() {{ return cabinetAvailable; }}
            function isCabinetOperationActive() {{ return false; }}
            function isNavigationActive() {{ return false; }}
            function hasRetiringNavigationGoals() {{ return false; }}
            function isManualControlReady() {{ return true; }}
            function clearDirectionInputs() {{}}
            function renderBaseControlStatus() {{}}
            {update_interlocks}

            updateControlInterlocks();
            assert.equal(
              $('btnCabinetPress').disabled,
              true,
              'policy-limited controls still require the navigation backend'
            );
            assert.equal(
              $('btnCabinetReset').disabled,
              false,
              'a ready scene reset must remain available'
            );
            navigationStatus.available = true;
            updateControlInterlocks();
            assert.equal(
              $('btnCabinetPress').disabled,
              false,
              'a limited control is submit-capable when both backends are ready'
            );
            cabinetOperationAvailable = false;
            cabinetAvailable = false;
            selected = {{control_id: 'physical', control_type: 1, operable: true}};
            updateControlInterlocks();
            assert.equal(
              $('btnCabinetPress').disabled,
              true,
              'a physical operation still requires its backend action server'
            );
        """))

    def test_limited_control_navigates_then_submits_live_validation(self) -> None:
        send_operation = _source_block(
            "async function sendCabinetOperation()",
            "\nasync function cancelCabinetOperation()",
        )
        self.run_node(textwrap.dedent(f"""
            const assert = require('node:assert/strict');
            let cabinetOperationWorkflow = null;
            let cabinetOperationWorkflowSequence = 0;
            let cabinetCatalogReceived = true;
            let manualTakeoverState = 'idle';
            let target = {{
              control_id: 'limited_button',
              control_type: 0,
              display_name: '受限按钮',
              supported_commands: 1,
              operable: false,
              unavailable_reason: '机械臂无法到达',
              max_force: 20
            }};
            const elements = {{
              cabinetForce: {{value: '5'}},
              cabinetStateField: {{hidden: true}},
              cabinetTargetState: {{value: ''}},
              cabinetTargetPosition: {{value: '0'}}
            }};
            const $ = id => elements[id] || (elements[id] = {{}});
            const requests = [];
            function isCabinetOperationActive() {{ return false; }}
            function isNavigationActive() {{ return false; }}
            function hasRetiringNavigationGoals() {{ return false; }}
            function selectedCabinetControl() {{ return target; }}
            function selectedCabinetName() {{ return 'cabinet_a'; }}
            function cabinetControlSupports(control, command) {{
              return command === 'press' && Boolean(control.supported_commands & 1);
            }}
            function toast() {{}}
            function updateCabinetWorkflowStatus() {{}}
            function clearDirectionInputs() {{ return Promise.resolve(); }}
            function renderBaseControlStatus() {{}}
            function updateControlInterlocks() {{}}
            function waitForControlDelay() {{ return Promise.resolve(); }}
            function setCabinetWorkflowTask(workflow, stage, task) {{
              workflow.stage = workflow.cancelRequested ? 'canceling' : stage;
              workflow.taskId = task.task_id;
            }}
            function applyTaskSnapshot() {{}}
            function requestCabinetWorkflowCancellation() {{ return Promise.resolve(); }}
            function waitForCabinetWorkflowTaskRelease() {{
              return Promise.resolve({{status: 'success', progress: 1}});
            }}
            function refreshAutonomyStatus() {{}}
            function cabinetWorkflowIsCurrent(workflow) {{
              return cabinetOperationWorkflow === workflow;
            }}
            function renderCabinetStatus() {{}}
            async function controlRequest(path, method, body) {{
              requests.push({{path, method, body}});
              if (path === '/task/navigate') {{
                return {{task: {{task_id: 'navigation_1', type: 'navigate', status: 'accepted'}}}};
              }}
              return {{task: {{task_id: 'operation_1', type: 'operate', status: 'accepted'}}}};
            }}
            {send_operation}

            (async () => {{
              await sendCabinetOperation();
              assert.deepEqual(
                requests.map(request => request.path),
                ['/task/navigate', '/task/operate']
              );
              assert.equal(requests[0].body.cabinet, 'cabinet_a');
              assert.equal(requests[0].body.control_id, 'limited_button');
              assert.equal(requests[1].body.control_id, 'limited_button');
              assert.equal(requests[1].body.force, 5);
              assert.equal(cabinetOperationWorkflow, null);
            }})().catch(error => {{ console.error(error); process.exitCode = 1; }});
        """))

    def test_validation_diagnostics_are_rendered(self) -> None:
        diagnostics = _source_block(
            "function cabinetValidationDetailLines(status)",
            "\nfunction cabinetStateLabel",
        )
        self.run_node(textwrap.dedent(f"""
            const assert = require('node:assert/strict');
            {diagnostics}

            const normal = cabinetValidationDetailLines({{
              validation_performed: false,
              operation_executed: true,
              diagnostic_stage: '',
              path_fraction: 0,
              required_fraction: 0,
              moveit_error_code: 0,
              policy_reason: ''
            }});
            assert.deepEqual(normal, ['机械臂/柜体物理操作: 已执行']);

            const failed = cabinetValidationDetailLines({{
              validation_performed: true,
              operation_executed: false,
              diagnostic_stage: 'approach',
              path_fraction: 0,
              required_fraction: 0.98,
              moveit_error_code: -1,
              policy_reason: '工作空间不可达'
            }});
            assert.deepEqual(failed, [
              '实时规划验证: 已执行 · 阶段: approach',
              '路径完成度: 0.0% / 要求 98.0%',
              '机械臂/柜体物理操作: 未执行',
              '验证原因: 工作空间不可达',
              'MoveIt 错误码: -1'
            ]);

            const dockingFailure = cabinetValidationDetailLines({{
              validation_performed: false,
              operation_executed: false,
              diagnostic_stage: 'docking',
              path_fraction: 0,
              required_fraction: 0,
              moveit_error_code: 0,
              policy_reason: ''
            }});
            assert.deepEqual(dockingFailure, [
              '操作失败阶段: docking',
              '机械臂/柜体物理操作: 未执行'
            ]);
        """))

    def test_cancel_prevents_posts_in_both_workflow_gaps(self) -> None:
        send_operation = _source_block(
            "async function sendCabinetOperation()",
            "\nasync function cancelCabinetOperation()",
        )
        cancel_operation = _source_block(
            "async function cancelCabinetOperation()",
            "\nasync function resetCabinetControls()",
        )
        self.run_node(textwrap.dedent(f"""
            const assert = require('node:assert/strict');
            let cabinetOperationWorkflow = null;
            let cabinetOperationWorkflowSequence = 0;
            let cabinetCatalogReceived = true;
            let manualTakeoverState = 'idle';
            let cabinetStatus = null;
            let cabinetStatusCurrent = false;
            let activeTaskId = null;
            let target = {{
              control_id: 'physical_button',
              control_type: 0,
              display_name: '可执行按钮',
              supported_commands: 1,
              operable: true,
              max_force: 20
            }};
            const elements = {{
              cabinetForce: {{value: '5'}},
              cabinetStateField: {{hidden: true}},
              cabinetTargetState: {{value: ''}},
              cabinetTargetPosition: {{value: '0'}}
            }};
            const $ = id => elements[id] || (elements[id] = {{}});
            let requests = [];
            let clearDirectionInputs = () => Promise.resolve();
            let waitForControlDelay = () => Promise.resolve();
            function isCabinetOperationActive() {{
              return Boolean(cabinetOperationWorkflow);
            }}
            function isNavigationActive() {{ return false; }}
            function hasRetiringNavigationGoals() {{ return false; }}
            function selectedCabinetControl() {{ return target; }}
            function selectedCabinetName() {{ return 'cabinet_a'; }}
            function cabinetControlSupports(control, command) {{
              return command === 'press' && Boolean(control.supported_commands & 1);
            }}
            function toast() {{}}
            function updateCabinetWorkflowStatus() {{}}
            function renderBaseControlStatus() {{}}
            function updateControlInterlocks() {{}}
            function setCabinetWorkflowTask(workflow, stage, task) {{
              workflow.stage = workflow.cancelRequested ? 'canceling' : stage;
              workflow.taskId = task.task_id;
            }}
            function applyTaskSnapshot() {{}}
            function requestCabinetWorkflowCancellation() {{ return Promise.resolve(); }}
            function waitForCabinetWorkflowTaskRelease() {{
              return Promise.resolve({{status: 'success', progress: 1}});
            }}
            function refreshAutonomyStatus() {{}}
            function cabinetWorkflowIsCurrent(workflow) {{
              return cabinetOperationWorkflow === workflow;
            }}
            function renderCabinetStatus() {{}}
            async function controlRequest(path, method, body) {{
              requests.push({{path, method, body}});
              if (path === '/task/navigate') {{
                return {{task: {{task_id: 'navigation_1', type: 'navigate', status: 'accepted'}}}};
              }}
              return {{task: {{task_id: 'operation_1', type: 'operate', status: 'accepted'}}}};
            }}
            {send_operation}
            {cancel_operation}

            (async () => {{
              let releaseInitial;
              clearDirectionInputs = () => new Promise(resolve => {{
                releaseInitial = resolve;
              }});
              const canceledBeforeNavigation = sendCabinetOperation();
              await Promise.resolve();
              await cancelCabinetOperation();
              releaseInitial();
              await canceledBeforeNavigation;
              assert.deepEqual(requests, []);

              requests = [];
              clearDirectionInputs = () => Promise.resolve();
              let releaseOperationGap;
              waitForControlDelay = () => new Promise(resolve => {{
                releaseOperationGap = resolve;
              }});
              const canceledBetweenTasks = sendCabinetOperation();
              for (let attempt = 0; attempt < 20; attempt += 1) {{
                if (cabinetOperationWorkflow &&
                    cabinetOperationWorkflow.stage === 'operation_starting') break;
                await Promise.resolve();
              }}
              assert.equal(cabinetOperationWorkflow.stage, 'operation_starting');
              await cancelCabinetOperation();
              releaseOperationGap();
              await canceledBetweenTasks;
              assert.deepEqual(
                requests.map(request => request.path),
                ['/task/navigate']
              );
            }})().catch(error => {{ console.error(error); process.exitCode = 1; }});
        """))

    def test_scene_reset_button_submits_scoped_task(self) -> None:
        html = MONITOR_PATH.read_text(encoding="utf-8")
        self.assertIn("重置场景（机器人 + 所选柜体归零）", html)
        self.assertRegex(
            html,
            r'class="command-row scene-reset-row">\s*'
            r'<button class="btn btn-outline scene-reset-button"\s*'
            r'id="btnCabinetReset" disabled>',
        )
        self.assertRegex(
            html,
            r"\.scene-reset-row \.scene-reset-button:disabled\s*\{\s*"
            r"opacity:\s*0\.72;",
        )
        reset = _source_block(
            "async function resetCabinetControls()",
            "\n\n// ─── Chassis Control",
        )
        self.run_node(textwrap.dedent(f"""
            const assert = require('node:assert/strict');
            let replayControlReadOnly = false;
            let cabinetResetAvailable = true;
            let cleared = 0;
            let refreshed = 0;
            const requests = [];
            const snapshots = [];
            const messages = [];
            function isCabinetOperationActive() {{ return false; }}
            function isNavigationActive() {{ return false; }}
            function hasRetiringNavigationGoals() {{ return false; }}
            function selectedCabinetName() {{ return 'cabinet_b'; }}
            async function clearDirectionInputs() {{ cleared += 1; }}
            async function controlRequest(path, method, body) {{
              requests.push({{path, method, body}});
              return {{
                task_id: 'reset_1',
                type: 'reset',
                status: 'accepted',
                request: {{cabinet: 'cabinet_b'}}
              }};
            }}
            function applyTaskSnapshot(task) {{ snapshots.push(task); }}
            function toast(message, kind) {{ messages.push({{message, kind}}); }}
            function refreshAutonomyStatus() {{ refreshed += 1; }}
            {reset}

            (async () => {{
              await resetCabinetControls();
              assert.equal(cleared, 1);
              assert.deepEqual(requests, [{{
                path: '/task/reset',
                method: 'POST',
                body: {{cabinet: 'cabinet_b'}}
              }}]);
              assert.equal(snapshots[0].type, 'reset');
              assert.equal(refreshed, 1);
              assert.match(messages[0].message, /机器人 \+ cabinet_b/);

              cabinetResetAvailable = false;
              await resetCabinetControls();
              assert.equal(requests.length, 1);
              assert.match(messages.at(-1).message, /尚未就绪/);
            }})().catch(error => {{
              console.error(error);
              process.exitCode = 1;
            }});
        """))

    def test_reset_task_snapshot_owns_interlock_and_renders_resetting(self) -> None:
        apply_task = _source_block(
            "function applyTaskSnapshot(task)",
            "\nfunction connectTaskEvents()",
        )
        self.run_node(textwrap.dedent(f"""
            const assert = require('node:assert/strict');
            const taskSnapshotVersions = new Map();
            const cabinetStatusByName = new Map();
            let activeTaskId = null;
            let activeTaskType = null;
            let navigationTaskStatus = null;
            let cabinetStatus = null;
            let cabinetStatusCurrent = false;
            let rendered = 0;
            let interlocks = 0;
            function selectedCabinetName() {{ return 'cabinet_a'; }}
            function renderNavigationStatus() {{}}
            function renderCabinetStatus() {{ rendered += 1; }}
            function updateControlInterlocks() {{ interlocks += 1; }}
            {apply_task}

            const accepted = {{
              task_id: 'reset_1',
              type: 'reset',
              status: 'accepted',
              request: {{cabinet: 'cabinet_a'}},
              phase: 'accepted',
              progress: 0,
              reservation_active: true,
              updated_at: 1
            }};
            applyTaskSnapshot(accepted);
            assert.equal(activeTaskId, 'reset_1');
            assert.equal(activeTaskType, 'reset');
            assert.equal(cabinetStatus.state, 'resetting');
            assert.equal(cabinetStatus.scene_reset, true);
            assert.equal(cabinetStatusByName.get('cabinet_a').active, true);

            applyTaskSnapshot({{
              ...accepted,
              status: 'success',
              phase: 'completed',
              progress: 1,
              reservation_active: false,
              updated_at: 2,
              result: {{scene_reset: true}}
            }});
            assert.equal(activeTaskId, null);
            assert.equal(activeTaskType, null);
            assert.equal(cabinetStatus.state, 'succeeded');
            assert.equal(cabinetStatus.success, true);
            assert.equal(rendered, 2);
            assert.equal(interlocks, 2);
        """))


if __name__ == "__main__":
    unittest.main()
