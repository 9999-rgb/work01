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
            assert.match(limited.textContent, /当前机器人预计失败/);
            assert.doesNotMatch(limited.textContent, /不可操作/);
        """))

    def test_robot_limited_control_can_submit_without_action_server(self) -> None:
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
            let cabinetOperationAvailable = false;
            let cabinetInterlockWasActive = false;
            let manualTakeoverState = 'idle';
            const navigationStatus = {{available: false}};
            const cabinetStatus = null;
            const cabinetInstances = [{{name: 'cabinet_a'}}];
            const armJointSpecs = [];
            const gripperJointSpecs = [];
            function selectedCabinetControl() {{ return selected; }}
            function selectedCabinetName() {{ return 'cabinet_a'; }}
            function isCabinetAvailable() {{ return false; }}
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
              false,
              'operable=false must remain submit-capable for local failure'
            );
            selected = {{control_id: 'physical', control_type: 1, operable: true}};
            updateControlInterlocks();
            assert.equal(
              $('btnCabinetPress').disabled,
              true,
              'a physical operation still requires its backend action server'
            );
        """))

    def test_limited_control_skips_navigation_and_submits_operation(self) -> None:
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
              return {{task: {{task_id: 'operation_1', type: 'operate', status: 'accepted'}}}};
            }}
            {send_operation}

            (async () => {{
              await sendCabinetOperation();
              assert.deepEqual(requests.map(request => request.path), ['/task/operate']);
              assert.equal(requests[0].body.control_id, 'limited_button');
              assert.equal(requests[0].body.force, 5);
              assert.equal(cabinetOperationWorkflow, null);
            }})().catch(error => {{ console.error(error); process.exitCode = 1; }});
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


if __name__ == "__main__":
    unittest.main()
