"""Behavioral contract tests for the cabinet workflow in ``monitor.html``."""

from __future__ import annotations

import re
import shutil
import subprocess
import textwrap
import unittest
from pathlib import Path


MONITOR_PATH = Path(__file__).resolve().parents[1] / "app" / "monitor.html"


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


class MonitorSecurityContractTest(unittest.TestCase):
    def test_dynamic_content_never_uses_html_injection_sinks(self) -> None:
        source = _inline_script()
        for sink in (
            ".innerHTML",
            ".outerHTML",
            "insertAdjacentHTML",
            "document.write",
        ):
            self.assertNotIn(sink, source)

    def test_stream_urls_and_sensor_health_carry_authentication(self) -> None:
        source = _inline_script()
        self.assertIn("url.searchParams.set('token', authToken)", source)
        self.assertIn("/sensors/health", source)
        self.assertIn("headers: authHeaders()", source)
        self.assertIn("sessionStorage.getItem('xczs_token')", source)
        self.assertIn("if (!authRequired || authToken) connectSensorStreams()", source)
        self.assertIn("authRequired = health.auth_required !== false", source)
        self.assertIn("if (authRequired && !authToken)", source)
        self.assertIn(
            "sensorEnabled && sensorConnectionKey === connectionKey",
            source,
        )

    def test_task_stream_overflow_resets_last_event_id_connection(self) -> None:
        source = _inline_script()
        self.assertIn(
            "addEventListener('stream_overflow'",
            source,
        )
        self.assertIn("staleSource.close()", source)
        self.assertIn(
            "refreshAutonomyStatus(autonomyStatusPollGeneration)",
            source,
        )


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

    def test_toolset_status_describes_world_preserving_hot_switch(self) -> None:
        toolset_status = _source_block(
            "function normalizedToolset(value)",
            "\nfunction updateAssetControlState()",
        )
        self.run_node(textwrap.dedent(f"""
            const assert = require('node:assert/strict');
            {toolset_status}

            const ready = describeToolsetStatus({{
              managed: true, state: 'ready', ready: true,
              active_toolset: 'A'
            }});
            assert.match(ready, /套装 A/);
            assert.match(ready, /一键切换/);
            assert.match(ready, /Gazebo 世界/);
            assert.match(ready, /控制栈/);

            const switching = describeToolsetStatus({{
              managed: true, state: 'switching', ready: false,
              active_toolset: 'A', target_toolset: 'B'
            }});
            assert.match(switching, /套装 A/);
            assert.match(switching, /套装 B/);
            assert.match(switching, /所有控制已锁定/);
            assert.doesNotMatch(switching, /重启/);

            const failed = describeToolsetStatus({{
              managed: true, state: 'failed', last_error: 'rollback failed'
            }});
            assert.match(failed, /控制保持锁定/);
            assert.match(failed, /rollback failed/);
        """))

    def test_toolset_switch_is_separate_from_asset_selection_save(self) -> None:
        save_selection = _source_block(
            "async function saveAssetSelection()",
            "\nconst toolsetStatusPollIntervalMs",
        )
        switch_toolset = _source_block(
            "async function switchToolset()",
            "\nasync function deleteSelectedAsset()",
        )
        self.assertIn("scene: assetSceneSelect.value || null", save_selection)
        self.assertIn("cabinet: assetCabinetSelect.value || null", save_selection)
        self.assertNotIn("toolset: assetToolsetSelect.value", save_selection)
        self.assertNotIn("/robot/toolset/switch", save_selection)

        self.assertIn("/robot/toolset/switch", switch_toolset)
        self.assertIn("expected_generation: expectedGeneration", switch_toolset)
        self.assertIn("accepted && accepted.toolset_status", switch_toolset)
        self.assertIn("waitForToolsetSwitchCompletion", switch_toolset)
        self.assertIn("refreshAfterToolsetSwitch", switch_toolset)
        self.assertNotIn("/assets/selection", switch_toolset)

    def test_toolset_switch_wait_executes_all_terminal_generation_branches(self) -> None:
        wait_source = _source_block(
            "const toolsetStatusPollIntervalMs",
            "\nasync function refreshAfterToolsetSwitch()",
        )
        self.run_node(textwrap.dedent(f"""
            const assert = require('node:assert/strict');
            let statuses = [];
            async function loadToolsetRuntimeStatus() {{
              assert.ok(statuses.length, 'unexpected extra status poll');
              return statuses.shift();
            }}
            function normalizedToolset(value) {{
              const normalized = String(value || '').trim().toUpperCase();
              return normalized === 'A' || normalized === 'B' ? normalized : '';
            }}
            {wait_source}

            (async () => {{
              statuses = [{{
                generation: 4, state: 'ready', ready: true,
                active_toolset: 'B', last_error: null
              }}];
              const success = await waitForToolsetSwitchCompletion('B', 4);
              assert.equal(success.active_toolset, 'B');

              statuses = [{{
                generation: 5, state: 'failed', ready: false,
                active_toolset: 'A', last_error: 'target startup failed'
              }}];
              await assert.rejects(
                waitForToolsetSwitchCompletion('B', 5),
                /target startup failed/
              );

              statuses = [{{
                generation: 6, state: 'ready', ready: true,
                active_toolset: 'A', last_error: 'toolset B rolled back'
              }}];
              await assert.rejects(
                waitForToolsetSwitchCompletion('B', 6),
                /toolset B rolled back/
              );

              statuses = [{{
                generation: 8, state: 'switching', ready: false,
                active_toolset: 'A', last_error: null
              }}];
              await assert.rejects(
                waitForToolsetSwitchCompletion('B', 7),
                /后续操作取代/
              );
            }})().catch(error => {{
              console.error(error);
              process.exitCode = 1;
            }});
        """))

    def test_connection_placeholders_use_the_unified_web_port(self) -> None:
        html = MONITOR_PATH.read_text(encoding="utf-8")

        self.assertIn(
            'id="restUrl" placeholder="http://localhost:8090"',
            html,
        )
        self.assertIn(
            'id="sensorUrl" placeholder="http://localhost:8090"',
            html,
        )
        self.assertNotIn('placeholder="http://localhost:8001"', html)
        self.assertNotIn('placeholder="http://localhost:8003"', html)

    def test_task_stream_overflow_closes_refreshes_and_reconnects_fresh(self) -> None:
        connect_events = _source_block(
            "function connectTaskEvents()",
            "\n// ─── Control Connection",
        )
        self.run_node(textwrap.dedent(f"""
            const assert = require('node:assert/strict');
            const sources = [];
            class EventSource {{
              constructor(url) {{
                this.url = url;
                this.listeners = new Map();
                this.closed = false;
                sources.push(this);
              }}
              addEventListener(name, callback) {{
                this.listeners.set(name, callback);
              }}
              close() {{ this.closed = true; }}
            }}
            let taskEventSource = null;
            let authRequired = false;
            let authToken = '';
            let ctrlConnected = true;
            let autonomyStatusPollGeneration = 7;
            let refreshes = 0;
            const ctrlBaseUrl = 'http://localhost:8090';
            function authenticatedUrl(base, path) {{ return base + path; }}
            function refreshAutonomyStatus(generation) {{
              assert.equal(generation, 7);
              refreshes += 1;
            }}
            function applyTaskSnapshot() {{}}
            function log() {{}}
            const timers = [];
            function setTimeout(callback) {{ timers.push(callback); return 1; }}
            {connect_events}

            connectTaskEvents();
            assert.equal(sources.length, 1);
            const stale = sources[0];
            connectTaskEvents();
            assert.equal(sources.length, 2);
            assert.equal(stale.closed, true);
            // A queued callback from the retired source must not close the
            // current connection or trigger another refresh.
            stale.listeners.get('stream_overflow')({{
              data: JSON.stringify({{reason: 'replay_history_gap'}})
            }});
            assert.equal(sources[1].closed, false);
            assert.equal(refreshes, 0);
            const current = sources[1];
            current.listeners.get('stream_overflow')({{
              data: JSON.stringify({{reason: 'replay_history_gap'}})
            }});
            assert.equal(current.closed, true);
            assert.equal(refreshes, 1);
            assert.equal(taskEventSource, null);
            timers.shift()();
            assert.equal(sources.length, 3);
            assert.equal(taskEventSource, sources[2]);
            assert.equal(sources[2].url, 'http://localhost:8090/task/events');
        """))

    def test_topic_stream_callbacks_and_timers_are_generation_scoped(self) -> None:
        topic_streams = _source_block(
            "function isCurrentTopicSource",
            "\nfunction ensureCard",
        )
        self.run_node(textwrap.dedent(f"""
            const assert = require('node:assert/strict');
            const sources = [];
            class EventSource {{
              constructor(url) {{
                this.url = url;
                this.listeners = new Map();
                this.closed = false;
                sources.push(this);
              }}
              addEventListener(name, callback) {{
                this.listeners.set(name, callback);
              }}
              close() {{ this.closed = true; }}
            }}
            const timers = [];
            function setTimeout(callback) {{
              const timer = {{callback, cleared: false}};
              timers.push(timer);
              return timer;
            }}
            function clearTimeout(timer) {{ timer.cleared = true; }}
            let connected = true;
            let authRequired = false;
            let authToken = '';
            const subs = new Map();
            const data = [];
            const logs = [];
            const dataGrid = {{
              children: [],
              style: {{}},
              replaceChildren() {{ this.children = []; }}
            }};
            const emptyState = {{style: {{}}}};
            const btnConnect = {{disabled: true}};
            const btnDisconnect = {{disabled: false}};
            const sessionInfo = {{textContent: 'connected'}};
            const document = {{getElementById() {{ return null; }}}};
            const $ = id => id === 'restUrl'
              ? {{value: 'http://localhost:8090'}} : null;
            function authenticatedUrl(base, path) {{ return base + path; }}
            function onData(topic, payload) {{ data.push([topic, payload]); }}
            function log(level, message) {{ logs.push([level, message]); }}
            function toast() {{}}
            function renderSubList() {{}}
            function ensureCard() {{}}
            function disconnectSensorStreams() {{}}
            function connectSensorStreams() {{}}
            function setStatus() {{}}
            {topic_streams}

            subscribe('xczs/odom');
            const first = sources[0];
            first.onerror();
            const firstTimer = timers[0];
            unsubscribe('xczs/odom');
            assert.equal(firstTimer.cleared, true);

            subscribe('xczs/odom');
            const second = sources[1];
            const logCount = logs.length;
            first.onopen();
            first.listeners.get('PUT')({{data: 'old'}});
            first.onerror();
            assert.equal(logs.length, logCount);
            assert.deepEqual(data, []);
            assert.equal(timers.length, 1);

            second.onerror();
            const secondTimer = timers[1];
            reconnectAuthenticatedStreams();
            const third = sources[2];
            assert.equal(second.closed, true);
            assert.equal(secondTimer.cleared, true);
            second.listeners.get('PUT')({{data: 'retired'}});
            assert.deepEqual(data, []);

            third.listeners.get('PUT')({{data: 'current'}});
            assert.deepEqual(data, [['xczs/odom', 'current']]);
            third.onerror();
            const thirdTimer = timers[2];
            doDisconnect();
            assert.equal(third.closed, true);
            assert.equal(thirdTimer.cleared, true);

            // Queued timeout callbacks remain harmless even if the host runs
            // them after clearTimeout due to an event-loop race.
            firstTimer.callback();
            secondTimer.callback();
            thirdTimer.callback();
            assert.equal(logs.filter(([level]) => level === 'err').length, 0);
        """))

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

    def test_missing_operable_capability_is_fail_closed(self) -> None:
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
            const cabinetControlTypeOrder = [0, 1, 2, 3];
            const cabinetControlTypeLabels = {{0: '按钮', 1: '旋钮', 2: '总开关', 3: '柜门'}};
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

            assert.equal(applyCabinetControls({{
              cabinet: 'cabinet_a',
              catalog_received: true,
              controls: [{{
                control_id: 'legacy_button',
                display_name: '旧目录按钮',
                control_type: 0,
                supported_commands: 1
              }}]
            }}), true);
            assert.equal(cabinetControls.length, 1);
            assert.equal(cabinetControls[0].operable, false);
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
            let toolsetInterlockWasActive = false;
            let toolsetLocked = false;
            let manualTakeoverState = 'idle';
            const navigationStatus = {{available: false}};
            const cabinetStatus = null;
            const cabinetInstances = [{{name: 'cabinet_a'}}];
            const manualJointSpecs = [];
            function selectedCabinetControl() {{ return selected; }}
            function selectedCabinetName() {{ return 'cabinet_a'; }}
            function isCabinetAvailable() {{ return cabinetAvailable; }}
            function isCabinetOperationActive() {{ return false; }}
            function isNavigationActive() {{ return false; }}
            function hasRetiringNavigationGoals() {{ return false; }}
            function isManualControlReady() {{ return true; }}
            function toolsetControlLocked() {{ return toolsetLocked; }}
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
            toolsetLocked = true;
            updateControlInterlocks();
            assert.equal($('btnNavSend').disabled, true);
            assert.equal($('btnManualMode').disabled, true);
            assert.equal($('btnArmZero').disabled, true);
            assert.equal(
              $('btnCabinetReset').disabled,
              true,
              'toolset replacement locks every task/control entry point'
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
            const waitedTaskIds = [];
            function waitForCabinetWorkflowTaskRelease(workflow, taskId) {{
              waitedTaskIds.push(taskId);
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
              assert.deepEqual(
                waitedTaskIds,
                ['navigation_1', 'operation_1'],
                'Web workflow must wait for both terminal task results'
              );
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
              assert.match(messages[0].message, /机器人 [+] cabinet_b/);

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

    def test_health_cannot_reenable_writes_with_incoherent_catalog(self) -> None:
        health = _source_block(
            "function applyCabinetHealth(health)",
            "\nfunction applyCabinetInstances(payload)",
        )
        self.run_node(textwrap.dedent(f"""
            const assert = require('node:assert/strict');
            let selected = 'cabinet_a';
            let cabinetOperationAvailable = false;
            let cabinetResetAvailable = false;
            let cabinetAvailableFromHealth = false;
            let cabinetCatalogCoherent = true;
            let cabinetCatalogCoherenceReason = '';
            let cabinetActiveFromHealth = false;
            function selectedCabinetName() {{ return selected; }}
            function applyToolsetRuntimeStatus() {{}}
            {health}

            applyCabinetHealth({{
              cabinets: {{cabinet_a: {{
                available: true,
                operation_available: true,
                reset_available: true,
                catalog_coherent: false,
                catalog_coherence_reason: '目录属于旧末端'
              }}}}
            }});
            assert.equal(cabinetAvailableFromHealth, false);
            assert.equal(cabinetCatalogCoherent, false);
            assert.equal(cabinetOperationAvailable, true);
            assert.equal(cabinetResetAvailable, true);
            assert.match(cabinetCatalogCoherenceReason, /旧末端/);
        """))

    def test_workflow_status_preserves_terminal_evidence_and_action_code(self) -> None:
        update = _source_block(
            "function updateCabinetWorkflowStatus(",
            "\nasync function waitForCabinetWorkflowTaskRelease",
        )
        self.run_node(textwrap.dedent(f"""
            const assert = require('node:assert/strict');
            const workflow = {{
              cabinet: 'cabinet_a', controlId: 'box_3', controlType: 0,
              command: 'press', targetState: null, targetPosition: null,
              force: 5
            }};
            let cabinetOperationWorkflow = workflow;
            let cabinetStatus = null;
            let cabinetStatusByName = new Map();
            let cabinetStatusCurrent = false;
            function cabinetWorkflowIsCurrent(value) {{
              return value === cabinetOperationWorkflow;
            }}
            function renderCabinetStatus() {{}}
            {update}

            updateCabinetWorkflowStatus(workflow, {{
              state: 'failed', phase: 'verifying', progress: 1,
              active: false, success: false,
              errorCode: 'insufficient_force',
              failureReason: '未达到触发阈值',
              failureDetails: {{peak_force: 2.1, evidence_source: 'backend'}},
              actionErrorCode: 13,
              result: {{physical_outcome_confirmed: false,
                final_state_verified: false}},
              message: '未达到触发阈值'
            }});
            assert.equal(cabinetStatus.failure_code, 'insufficient_force');
            assert.equal(cabinetStatus.failure_reason, '未达到触发阈值');
            assert.deepEqual(cabinetStatus.failure_details, {{
              peak_force: 2.1, evidence_source: 'backend'
            }});
            assert.equal(cabinetStatus.action_error_code, 13);
            assert.equal(cabinetStatus.error_code, 13);
            assert.equal(cabinetStatus.physical_outcome_confirmed, false);
            assert.equal(cabinetStatus.final_state_verified, false);
            assert.equal(cabinetStatus.result.physical_outcome_confirmed, false);
            assert.equal(cabinetStatusByName.get('cabinet_a'), cabinetStatus);
        """))


if __name__ == "__main__":
    unittest.main()
